// GoGoVernier — Phase 2.
//
// open() connects + subscribes (Phase 1) and then performs the D2PIO setup
// handshake: INIT → GET_DEVICE_INFO → GET_SENSOR_AVAILABLE_MASK →
// GET_SENSOR_INFO(i) for each set bit. start() / stop() send
// SET_MEASUREMENT_PERIOD + START_MEASUREMENTS / STOP_MEASUREMENTS, and
// the notify callback decodes RESPONSE_MEASUREMENT frames live into
// _channels[].value with sample_ready bookkeeping.
//
// Frame layout cross-checked against VernierST/godirect-py
// (BSD-3, © 2024 Vernier Science Education):
//   request:  [0x58][len][rcnt][checksum][cmd_id][payload...]
//   response: [op  ][len][rcnt][checksum][cmd_id|meas_type][payload...]
//
// Synchronisation between sendRequest() and the notify-rx callback is a
// single-shot binary semaphore + a copy buffer. Multi-device (Phase 4)
// will key this off conn_handle.

#include "GoGoVernier.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>

#include "transport/NimBleXport.h"

namespace gogo_vernier {

namespace {

constexpr uint32_t kRequestTimeoutMs = 3000;
constexpr uint16_t kRespBufSize      = 256;

// Deserialise a little-endian unsigned int from `buf[0..N-1]`.
template <typename T>
T leUnpack(const uint8_t* buf) {
    T v = 0;
    for (size_t i = 0; i < sizeof(T); ++i) {
        v |= static_cast<T>(buf[i]) << (i * 8);
    }
    return v;
}

}  // namespace

struct GoGoVernier::Impl {
    NimBleXport xport;

    // Connection / streaming state.
    bool         connected      = false;
    bool         scanning       = false;
    bool         streaming      = false;
    bool         sample_ready   = false;
    uint32_t     dropped        = 0;
    uint32_t     available      = 0;
    uint32_t     enabled        = 0;
    uint16_t     period_ms      = 1000;
    uint8_t      channel_count  = 0;
    ChannelInfo  channels[kMaxChannels] = {};
    DeviceInfo   info           = {};
    DeviceStatus status         = {};

    // Protocol state.
    uint8_t           rolling_counter = 0xFF;
    SemaphoreHandle_t resp_sem        = nullptr;     // single-shot, given by notify cb
    uint8_t           resp_buf[kRespBufSize] = {};
    uint16_t          resp_len        = 0;

    uint8_t nextRollingCounter() {
        // Pre-decrement, wrap 0x00 → 0xFF, matching godirect-py.
        if (rolling_counter == 0) rolling_counter = 0xFF;
        else                      --rolling_counter;
        return rolling_counter;
    }

    // Build a request frame in `out`. `payload` may be null if `payload_len`
    // is 0. Returns total frame length.
    uint8_t encode(uint8_t* out, uint8_t cmd_id,
                   const uint8_t* payload, uint8_t payload_len) {
        uint8_t total = static_cast<uint8_t>(kFrameHeaderSize + payload_len);
        out[0] = kFrameHeader;
        out[1] = total;
        out[2] = nextRollingCounter();
        out[3] = 0;            // checksum placeholder
        out[4] = cmd_id;
        if (payload && payload_len) memcpy(out + 5, payload, payload_len);
        out[3] = calculateChecksum(out, total);
        return total;
    }

    // Send `out[0..len-1]` and wait up to `timeout_ms` for a non-measurement
    // response frame. The notify callback drops measurement frames on the
    // floor (handles them in-place into channels[]) and only releases the
    // semaphore for a real reply.
    bool sendRequest(const uint8_t* out, uint8_t len, uint32_t timeout_ms) {
        // Drain any pending take so we don't accept a stale unblock.
        xSemaphoreTake(resp_sem, 0);
        resp_len = 0;
        if (!xport.write(out, len)) {
            log_e("xport.write failed cmd=0x%02X", out[4]);
            return false;
        }
        if (xSemaphoreTake(resp_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
            log_e("response timeout cmd=0x%02X", out[4]);
            return false;
        }
        return true;
    }

    void onNotify(const uint8_t* data, uint16_t len) {
        if (len < 5) return;

        // Measurement frames are pushed by the device unsolicited and at
        // the configured cadence — handle them here, never wake the
        // request semaphore on them.
        if (data[0] == kResponseMeasurement) {
            decodeMeasurement(data, len);
            return;
        }

        // Otherwise it's a reply to whatever request we last sent. Snapshot
        // and unblock the waiter.
        uint16_t copy_len = len > kRespBufSize ? kRespBufSize : len;
        memcpy(resp_buf, data, copy_len);
        resp_len = copy_len;
        xSemaphoreGive(resp_sem);
    }

    // RESPONSE_MEASUREMENT (0x20). Layout per godirect-py:
    //   [0]=0x20  [1]=len  [2]=rcnt  [3]=cksum  [4]=meas_type  [5..]=payload
    void decodeMeasurement(const uint8_t* data, uint16_t len) {
        if (len < 5) return;
        uint8_t meas_type = data[4];

        // For the common periodic types the layout is:
        //   NORMAL_REAL32:    [5..6] u16 sensor_mask, [7] count, [9..]=N*f32
        //   WIDE_REAL32:      [5..8] u32 sensor_mask, [9] count, [11..]=N*f32
        //   SINGLE_REAL32:    [6] sensor_no,          [7] count, [8..]=N*f32
        // We support all three because the same firmware needs to drive
        // GDX-LC (single-channel SINGLE) and GDX-ACC (multi-channel NORMAL).
        uint32_t mask     = 0;
        uint8_t  count    = 0;
        uint16_t idx      = 0;
        bool is_real32 = false;

        if (meas_type == MEAS_NORMAL_REAL32 && len >= 11) {
            mask  = static_cast<uint32_t>(leUnpack<uint16_t>(data + 5));
            count = data[7];
            idx   = 9;
            is_real32 = true;
        } else if (meas_type == MEAS_WIDE_REAL32 && len >= 13) {
            mask  = leUnpack<uint32_t>(data + 5);
            count = data[9];
            idx   = 11;
            is_real32 = true;
        } else if ((meas_type == MEAS_SINGLE_CHANNEL_REAL32 ||
                    meas_type == MEAS_APERIODIC_REAL32) && len >= 10) {
            uint8_t sno = data[6];
            count       = data[7];
            idx         = 8;
            mask        = (sno < kMaxChannels) ? (1u << sno) : 0;
            is_real32   = true;
        } else {
            return; // ignore int32 / start_time / dropped / period for now
        }

        if (!is_real32 || count == 0 || mask == 0) return;

        // Detect drop: previous sample wasn't drained.
        if (sample_ready) ++dropped;

        for (uint8_t n = 0; n < count; ++n) {
            for (uint8_t i = 0; i < kMaxChannels && idx + 4 <= len; ++i) {
                if (mask & (1u << i)) {
                    float v;
                    memcpy(&v, data + idx, 4);
                    channels[i].value = v;
                    idx += 4;
                }
            }
        }
        sample_ready = true;
    }

    // Decode CMD_GET_DEVICE_INFO response. godirect-py struct:
    //   "<xxxxxx16s16s32sHHBBBBHBBHBBBBBBI64s"
    // The first 6 bytes are header (op,len,rcnt,checksum,cmd_id,spare).
    void decodeDeviceInfo() {
        if (resp_len < 6 + 16 + 16 + 32) return;
        const uint8_t* p = resp_buf + 6;
        memset(info.order_code, 0, sizeof(info.order_code));
        memset(info.serial,     0, sizeof(info.serial));
        memset(info.name,       0, sizeof(info.name));
        memcpy(info.order_code, p,             16);
        info.order_code[15] = '\0';
        memcpy(info.serial,     p + 16,        16);
        info.serial[15]     = '\0';
        memcpy(info.name,       p + 32,
               sizeof(info.name) - 1);          // 32-byte field, our struct is 32
        info.name[sizeof(info.name) - 1] = '\0';
        // Manufacturer ID + dates + FW versions follow at offset 6+64; we
        // only consume what DeviceInfo currently exposes. Phase 3 can
        // widen the struct.
    }

    // Decode CMD_GET_SENSOR_AVAILABLE_MASK / CMD_GET_DEFAULT_SENSORS_MASK.
    // godirect-py: `mask = struct.unpack("<I", response[6:])[0]` — a
    // single u32 LE in the body.
    bool decodeMask(uint32_t* out_mask) {
        if (resp_len < 6 + 4) return false;
        *out_mask = leUnpack<uint32_t>(resp_buf + 6);
        return true;
    }

    // Decode CMD_GET_SENSOR_INFO response. struct:
    //   "<bBIBB60s32sdddIQIII"
    // Total decoded body = 1+1+4+1+1+60+32+8+8+8+4+8+4+4+4 = 148 bytes
    // after the 6-byte header.
    bool decodeSensorInfo() {
        if (resp_len < 6 + 148) return false;
        const uint8_t* p = resp_buf + 6;
        int8_t   sensor_no   = static_cast<int8_t>(p[0]);
        // p[1] = spare
        uint32_t sensor_id   = leUnpack<uint32_t>(p + 2);
        uint8_t  meas_type   = p[6];
        uint8_t  sampling    = p[7];
        const uint8_t* desc  = p + 8;            // 60 bytes
        const uint8_t* units = p + 8 + 60;       // 32 bytes
        const uint8_t* dbls  = p + 8 + 60 + 32;  // 3 doubles = 24 bytes
        const uint8_t* tail  = dbls + 24;        // u32 + u64 + u32 + u32 + u32

        if (sensor_no < 0 || sensor_no >= static_cast<int>(kMaxChannels)) {
            return false;
        }
        ChannelInfo& c = channels[sensor_no];
        c.number               = static_cast<uint8_t>(sensor_no);
        c.sensor_id            = sensor_id;
        c.measurement_type     = meas_type;
        c.sampling_mode        = sampling;
        memset(c.description, 0, sizeof(c.description));
        memset(c.units,       0, sizeof(c.units));
        memcpy(c.description, desc,
               sizeof(c.description) > 60 ? 60 : sizeof(c.description) - 1);
        c.description[sizeof(c.description) - 1] = '\0';
        memcpy(c.units, units,
               sizeof(c.units) > 32 ? 32 : sizeof(c.units) - 1);
        c.units[sizeof(c.units) - 1] = '\0';

        // doubles: uncertainty, min, max — narrow to f32 for the channel
        // struct (existing API contract).
        double d;
        memcpy(&d, dbls + 0,  8); c.measurement_uncertainty = static_cast<float>(d);
        memcpy(&d, dbls + 8,  8); c.min_measurement         = static_cast<float>(d);
        memcpy(&d, dbls + 16, 8); c.max_measurement         = static_cast<float>(d);

        // <IQIII> after the doubles. The Q is 8 bytes.
        c.min_period_us           = leUnpack<uint32_t>(tail + 0);
        // skip Q (max_period_us is 64-bit on the wire; clamp to u32)
        uint64_t maxp = leUnpack<uint64_t>(tail + 4);
        c.max_period_us           = (maxp > 0xFFFFFFFFu) ? 0xFFFFFFFFu
                                                          : static_cast<uint32_t>(maxp);
        c.typ_period_us           = leUnpack<uint32_t>(tail + 12);
        c.period_granularity_us   = leUnpack<uint32_t>(tail + 16);
        c.mutual_exclusion_mask   = leUnpack<uint32_t>(tail + 20);
        return true;
    }
};

GoGoVernier::GoGoVernier() : _impl(new Impl()) {
    _impl->resp_sem = xSemaphoreCreateBinary();
}

GoGoVernier::~GoGoVernier() {
    if (_impl->resp_sem) vSemaphoreDelete(_impl->resp_sem);
    delete _impl;
}

bool GoGoVernier::open(const char* name) {
    if (_impl->connected) close();

    constexpr uint32_t kScanMs = 5000;
    log_i("open name=\"%s\"", name ? name : "");
    if (!_impl->xport.connect(name, kScanMs)) {
        log_e("open failed (no peer or connect rejected)");
        return false;
    }

    Impl* self = _impl;
    _impl->xport.subscribe([self](const uint8_t* data, uint16_t len) {
        self->onNotify(data, len);
    });

    // Cache scan-time peer name + RSSI as a fallback before the device
    // sends its real DEVICE_INFO reply.
    const char* pname = _impl->xport.peerName();
    const char* paddr = _impl->xport.peerAddress();
    if (pname && *pname) {
        strncpy(_impl->info.name, pname, sizeof(_impl->info.name) - 1);
        _impl->info.name[sizeof(_impl->info.name) - 1] = '\0';
    } else if (paddr) {
        strncpy(_impl->info.name, paddr, sizeof(_impl->info.name) - 1);
    }
    _impl->status.rssi = static_cast<int8_t>(_impl->xport.rssi());
    _impl->connected   = true;
    _impl->dropped     = 0;
    _impl->rolling_counter = 0xFF;

    // ---- D2PIO handshake ----
    uint8_t buf[64];

    // CMD_INIT — godirect-py sends a 25-byte literal payload. The exact
    // sequence is opaque (likely a vendor handshake) but stable.
    static const uint8_t kInitPayload[] = {
        0xa5, 0x4a, 0x06, 0x49,
        0x07, 0x48, 0x08, 0x47,
        0x09, 0x46, 0x0a, 0x45,
        0x0b, 0x44, 0x0c, 0x43,
        0x0d, 0x42, 0x0e, 0x41,
    };
    {
        uint8_t n = _impl->encode(buf, CMD_INIT, kInitPayload, sizeof(kInitPayload));
        if (!_impl->sendRequest(buf, n, kRequestTimeoutMs)) {
            log_e("CMD_INIT failed");
            close();
            return false;
        }
        log_d("CMD_INIT ack");
    }

    // CMD_GET_DEVICE_INFO
    {
        uint8_t n = _impl->encode(buf, CMD_GET_DEVICE_INFO, nullptr, 0);
        if (!_impl->sendRequest(buf, n, kRequestTimeoutMs)) {
            log_e("CMD_GET_DEVICE_INFO failed");
        } else {
            _impl->decodeDeviceInfo();
            log_i("device order=\"%s\" serial=\"%s\" name=\"%s\"",
                  _impl->info.order_code, _impl->info.serial, _impl->info.name);
        }
    }

    // CMD_GET_SENSOR_AVAILABLE_MASK
    {
        uint8_t n = _impl->encode(buf, CMD_GET_SENSOR_AVAILABLE_MASK, nullptr, 0);
        if (!_impl->sendRequest(buf, n, kRequestTimeoutMs)) {
            log_e("CMD_GET_SENSOR_AVAILABLE_MASK failed");
            close();
            return false;
        }
        if (!_impl->decodeMask(&_impl->available)) {
            log_e("decodeMask failed (resp_len=%u)", _impl->resp_len);
            close();
            return false;
        }
        log_i("available channel mask=0x%08X", (unsigned)_impl->available);
    }

    // CMD_GET_SENSOR_INFO(i) for each set bit. Populates channels[].
    _impl->channel_count = 0;
    for (uint8_t i = 0; i < kMaxChannels; ++i) {
        if (!(_impl->available & (1u << i))) continue;
        uint8_t payload = i;
        uint8_t n = _impl->encode(buf, CMD_GET_SENSOR_INFO, &payload, 1);
        if (!_impl->sendRequest(buf, n, kRequestTimeoutMs)) {
            log_w("GET_SENSOR_INFO(%u) failed", i);
            continue;
        }
        if (_impl->decodeSensorInfo()) {
            ++_impl->channel_count;
            log_i("ch%u %s (%s)", i,
                  _impl->channels[i].description,
                  _impl->channels[i].units);
        }
    }

    // Default-enable everything available (mirrors the old GDXLib /
    // VernierAdapter behaviour). Phase 4 will respect mutual_exclusion_mask.
    _impl->enabled = _impl->available;

    return true;
}

void GoGoVernier::close() {
    log_i("close");
    if (_impl->streaming) stop();
    _impl->xport.unsubscribe();
    _impl->xport.disconnect();
    _impl->connected     = false;
    _impl->streaming     = false;
    _impl->sample_ready  = false;
    _impl->available     = 0;
    _impl->enabled       = 0;
    _impl->channel_count = 0;
    memset(&_impl->info, 0, sizeof(_impl->info));
    memset(&_impl->status, 0, sizeof(_impl->status));
    for (auto& c : _impl->channels) c = {};
}

bool GoGoVernier::isConnected() const                { return _impl->connected; }
bool GoGoVernier::isScanning() const                 { return _impl->scanning; }
void GoGoVernier::abortScan()                        {}

bool GoGoVernier::enableSensor(uint8_t ch) {
    if (ch >= kMaxChannels) return false;
    if (!(_impl->available & (1u << ch))) return false;
    _impl->enabled |= (1u << ch);
    _impl->channels[ch].enabled = true;
    return true;
}

bool GoGoVernier::disableSensor(uint8_t ch) {
    if (ch >= kMaxChannels) return false;
    _impl->enabled &= ~(1u << ch);
    _impl->channels[ch].enabled = false;
    return true;
}

uint32_t GoGoVernier::availableChannelMask() const   { return _impl->available; }
uint32_t GoGoVernier::enabledChannelMask() const     { return _impl->enabled; }
uint8_t  GoGoVernier::channelCount() const           { return _impl->channel_count; }

const ChannelInfo* GoGoVernier::channel(uint8_t ch) const {
    if (ch >= kMaxChannels) return nullptr;
    return &_impl->channels[ch];
}

bool GoGoVernier::start(uint32_t period_ms) {
    if (!_impl->connected) return false;
    if (period_ms) _impl->period_ms = static_cast<uint16_t>(period_ms);

    uint8_t buf[64];

    // CMD_SET_MEASUREMENT_PERIOD — payload = [0xFF][0x00][u32 period_us LE].
    {
        uint32_t period_us = static_cast<uint32_t>(_impl->period_ms) * 1000u;
        uint8_t payload[10] = { 0xFF, 0x00,
                                static_cast<uint8_t>((period_us >> 0)  & 0xFF),
                                static_cast<uint8_t>((period_us >> 8)  & 0xFF),
                                static_cast<uint8_t>((period_us >> 16) & 0xFF),
                                static_cast<uint8_t>((period_us >> 24) & 0xFF),
                                0, 0, 0, 0 };
        uint8_t n = _impl->encode(buf, CMD_SET_MEASUREMENT_PERIOD,
                                  payload, sizeof(payload));
        if (!_impl->sendRequest(buf, n, kRequestTimeoutMs)) {
            log_e("CMD_SET_MEASUREMENT_PERIOD failed");
            return false;
        }
    }

    // CMD_START_MEASUREMENTS — payload = [0xFF][0x01][u32 mask LE][8 zero bytes].
    {
        uint32_t mask = _impl->enabled;
        uint8_t payload[14] = { 0xFF, 0x01,
                                static_cast<uint8_t>((mask >> 0)  & 0xFF),
                                static_cast<uint8_t>((mask >> 8)  & 0xFF),
                                static_cast<uint8_t>((mask >> 16) & 0xFF),
                                static_cast<uint8_t>((mask >> 24) & 0xFF),
                                0, 0, 0, 0, 0, 0, 0, 0 };
        uint8_t n = _impl->encode(buf, CMD_START_MEASUREMENTS,
                                  payload, sizeof(payload));
        if (!_impl->sendRequest(buf, n, kRequestTimeoutMs)) {
            log_e("CMD_START_MEASUREMENTS failed");
            return false;
        }
    }

    _impl->streaming = true;
    log_i("streaming started period=%ums mask=0x%08X",
          (unsigned)_impl->period_ms, (unsigned)_impl->enabled);
    return true;
}

bool GoGoVernier::stop() {
    if (!_impl->connected || !_impl->streaming) {
        _impl->streaming = false;
        return true;
    }
    uint8_t buf[64];
    // CMD_STOP_MEASUREMENTS — payload [0xFF][0x00][0xFF * 4].
    uint8_t payload[6] = { 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0xFF };
    uint8_t n = _impl->encode(buf, CMD_STOP_MEASUREMENTS, payload, sizeof(payload));
    bool ok = _impl->sendRequest(buf, n, kRequestTimeoutMs);
    _impl->streaming = false;
    return ok;
}

bool GoGoVernier::isStreaming() const                 { return _impl->streaming; }

bool GoGoVernier::sampleReady() const                 { return _impl->sample_ready; }
bool GoGoVernier::copySample(float* out, uint8_t& count) {
    if (!_impl->sample_ready) { count = 0; return false; }
    uint8_t idx = 0;
    for (uint8_t i = 0; i < kMaxChannels; ++i) {
        if (_impl->enabled & (1u << i)) {
            if (out) out[idx] = _impl->channels[i].value;
            ++idx;
        }
    }
    count = idx;
    _impl->sample_ready = false;
    return true;
}
float GoGoVernier::measurement(uint8_t ch) const {
    if (ch >= kMaxChannels) return 0.0f;
    return _impl->channels[ch].value;
}
uint32_t GoGoVernier::droppedSamples() const          { return _impl->dropped; }

const DeviceInfo&   GoGoVernier::deviceInfo() const   { return _impl->info; }
const DeviceStatus& GoGoVernier::status() const       { return _impl->status; }

bool GoGoVernier::refreshStatus() {
    if (!_impl->connected) return false;
    // CMD_GET_STATUS isn't in the spec's published opcode list yet (godirect-py
    // hardcodes its own constant); skip until Phase 3 wires it in. For now
    // status.rssi can still be refreshed from the live BLE link.
    _impl->status.rssi = static_cast<int8_t>(_impl->xport.rssi());
    return true;
}

}  // namespace gogo_vernier
