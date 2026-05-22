// GoGoVernier — D2PIO session implementation.
//
// open() runs the full D2PIO setup handshake: BLE connect + subscribe →
// INIT → GET_DEVICE_INFO → GET_SENSOR_AVAILABLE_MASK → GET_SENSOR_INFO(i)
// for each set bit. start() / stop() send SET_MEASUREMENT_PERIOD +
// START / STOP_MEASUREMENTS. The notify callback decodes
// RESPONSE_MEASUREMENT frames live into _channels[].value, raises
// sample_ready, and fires the optional onSample push callback.
// refreshStatus() queries CMD_GET_STATUS for battery + charger state.
//
// Frame layout cross-checked against VernierST/godirect-py
// (BSD-3, © 2024 Vernier Science Education):
//   request:  [0x58][len][rcnt][checksum][cmd_id][payload...]
//   response: [op  ][len][rcnt][checksum][cmd_id|meas_type][payload...]
//
// Synchronisation between sendRequest() and the notify-rx callback is a
// single-shot binary semaphore + a copy buffer, scoped to this Impl.
// Multi-device runs as N independent instances — transport routes
// notifications per-session via lambda capture in NimBleXport, so this
// file holds no shared state across peers.

#include "GoGoVernier.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>

#include "transport/NimBleXport.h"

namespace gogo_vernier {

namespace {

// Default response timeout for D2PIO request frames. Most replies land
// within ~100 ms; CMD_GET_DEVICE_INFO is the outlier (see below).
constexpr uint32_t REQUEST_TIMEOUT_MS    = 3000;

// CMD_GET_DEVICE_INFO is consistently slow on GDX-LC (4–7 s observed).
// Use a wider per-call timeout for that one command only.
constexpr uint32_t DEVICE_INFO_TIMEOUT_MS = 8000;

// Largest response frame we buffer. Sized for CMD_GET_SENSOR_INFO,
// whose body is SENSOR_INFO_BODY_SIZE (148 B) after FRAME_BODY_OFFSET.
constexpr uint16_t RESP_BUF_SIZE         =  256;

// Sentinel for pending_rcnt / pending_cmd meaning "no request in
// flight". u16 so the value can't collide with any valid u8 byte the
// device might echo at us. Decimal is 65535.
constexpr uint16_t PENDING_NONE         = 0xFFFF;

// Default sampling period if the host never set one — also used as the
// initial value of period_ms before start() runs.
constexpr uint16_t DEFAULT_PERIOD_MS     = 1000;

// Initial value for the rolling counter. godirect-py initialises to
// 0xFF and decrements on each request, wrapping back to 0xFF after 0x00.
constexpr uint8_t  INITIAL_ROLLING_COUNTER = 0xFF;

// Rolling counter wrap value (also the sentinel after the wrap branch).
constexpr uint8_t  ROLLING_COUNTER_WRAP    = 0xFF;

// Microseconds-per-millisecond conversion for SET_MEASUREMENT_PERIOD,
// whose wire payload carries period_us as a u32 LE.
constexpr uint32_t US_PER_MS               = 1000;

// Extra slack added on top of the per-call response timeout when taking
// req_mutex. Lets the contended-caller wait long enough for the
// in-flight request's own timeout to fire and release the mutex.
constexpr uint32_t REQ_MUTEX_GRACE_MS      = 1000;

// Pre-handshake scan timeout. open() runs a single discovery pass
// before the BLE connect — 5 s catches GDX advertisers (advertise
// every ~100 ms) without keeping the radio busy for unconnected hosts.
constexpr uint32_t OPEN_SCAN_MS            = 5000;

// Compiler-only memory barrier for cross-task publication on a
// single-core RISC-V MCU (ESP32-C3). Forces the compiler to emit all
// preceding writes before any following store/load so a notify-task
// "sample ready" flag can't be observed before its data writes. No
// hardware fence is needed: there's only one core, FreeRTOS context
// switches insert their own kernel barriers, and the Xtensa `memw`
// instruction wouldn't assemble here. Use this exact form to stay
// consistent with vernier-firmware/src/main.cpp.
#define VERNIER_CROSS_TASK_BARRIER() __asm__ volatile("" ::: "memory")

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
    bool              connected      = false;
    // Set true only after open() finishes the full D2PIO handshake
    // (INIT, DEVICE_INFO, AVAILABLE_MASK, SENSOR_INFO×N) and the
    // channel array is populated. `connected` flips earlier (right
    // after the BLE link is up) which is too soon for callers to
    // start streaming.
    bool              ready          = false;
    bool              scanning       = false;
    bool              streaming      = false;
    // Cross-task flag: written by the NimBLE notify task in
    // decodeMeasurement(), read+cleared by the caller task in
    // copySample(). volatile prevents the compiler from caching a
    // polled read in a register. A compiler barrier in
    // decodeMeasurement() ensures all channels[].value stores complete
    // before the flag flips — see VERNIER_CROSS_TASK_BARRIER().
    volatile bool     sample_ready   = false;
    // Cross-task counter: incremented from the notify task on drop
    // (both the local "previous sample not drained" path and the
    // device-reported MEAS_DROPPED tally), read from caller tasks via
    // droppedSamples().
    volatile uint32_t dropped        = 0;
    uint32_t          available      = 0;
    uint32_t          enabled        = 0;
    uint16_t          period_ms      = DEFAULT_PERIOD_MS;
    uint8_t           channel_count  = 0;
    ChannelInfo       channels[MAX_CHANNELS] = {};
    DeviceInfo        info           = {};
    DeviceStatus      status         = {};

    // Protocol state.
    uint8_t           rolling_counter = INITIAL_ROLLING_COUNTER;
    SemaphoreHandle_t resp_sem        = nullptr;     // single-shot, given by notify cb
    uint8_t           resp_buf[RESP_BUF_SIZE] = {};
    uint16_t          resp_len        = 0;
    // Stamped by encode() so onNotify() can drop stale responses (the late
    // ACK to a previously-timed-out request must not satisfy the next
    // sendRequest). Both fields are u16 so the PENDING_NONE sentinel
    // (0xFFFF) can't collide with any valid u8 byte we might see.
    uint16_t          pending_rcnt    = PENDING_NONE;
    uint16_t          pending_cmd     = PENDING_NONE;
    // Serialises sendRequest. The host MCU can fire C_CONNECT/C_SET_PERIOD
    // on uartHandler while loop()-driven auto-connect is still mid-
    // handshake on the main task; without this, both tasks call encode()
    // (overwriting pending_rcnt/cmd for each other) and block on the same
    // resp_sem. Held only across one wire request, never around
    // open()/start() in their entirety.
    SemaphoreHandle_t req_mutex       = nullptr;

    // Serialises the entire open()/close()/start()/stop() bodies. Without
    // this, a concurrent startReading from a host-MCU C_CONNECT can
    // interleave SET_PERIOD writes between two SENSOR_INFO reads in the
    // open() handshake loop and confuse the device into dropping
    // subsequent SENSOR_INFO replies. session_mutex is taken at the top
    // of each public method, held across all wire requests for that
    // session phase. req_mutex still wraps individual sendRequest calls
    // so a notify cb landing during a multi-step phase is matched to
    // the right pending_cmd.
    SemaphoreHandle_t session_mutex   = nullptr;

    // Push-mode consumer. Fired from decodeMeasurement on the NimBLE
    // notify task. Empty (default) → push path is disabled and only
    // the polling sample_ready path runs.
    SampleCallback    on_sample;

    uint8_t nextRollingCounter() {
        // Pre-decrement, wrap 0x00 → 0xFF, matching godirect-py.
        if (rolling_counter == 0) rolling_counter = ROLLING_COUNTER_WRAP;
        else                      --rolling_counter;
        return rolling_counter;
    }

    // Build a request frame in `out`. `payload` may be null if `payload_len`
    // is 0. Returns total frame length. Does NOT stamp pending_rcnt /
    // pending_cmd — that happens inside sendRequest under req_mutex so
    // two tasks racing into encode + sendRequest can't clobber each
    // other's pending state.
    uint8_t encode(uint8_t* out, uint8_t cmd_id,
                   const uint8_t* payload, uint8_t payload_len) {
        uint8_t total = static_cast<uint8_t>(FRAME_HEADER_SIZE + payload_len);
        out[0] = FRAME_HEADER;
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
        // One in-flight request at a time, no matter which task called us.
        if (xSemaphoreTake(req_mutex,
                           pdMS_TO_TICKS(timeout_ms + REQ_MUTEX_GRACE_MS)) != pdTRUE) {
            log_e("req_mutex contended cmd=0x%02X", out[4]);
            return false;
        }
        struct GiveOnExit {
            SemaphoreHandle_t m;
            ~GiveOnExit() { xSemaphoreGive(m); }
        } _scope { req_mutex };

        // Stamp pending markers under the mutex so concurrent sendRequest
        // calls don't trample each other's pending_cmd. Previously this
        // stamping lived in encode(), outside the mutex, which produced
        // an observed race: thread A's INIT encode ran, then thread B's
        // SET_PERIOD encode overwrote pending_cmd, then INIT's notify
        // ack arrived but was dropped as "stale" (cmd mismatch).
        pending_rcnt = out[2];
        pending_cmd  = out[4];

        // Drain any pending take so we don't accept a stale unblock.
        xSemaphoreTake(resp_sem, 0);
        resp_len = 0;
        if (!xport.write(out, len)) {
            log_e("xport.write failed cmd=0x%02X", out[4]);
            pending_rcnt = pending_cmd = PENDING_NONE;
            return false;
        }
        if (xSemaphoreTake(resp_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
            log_e("response timeout cmd=0x%02X", out[4]);
            // Mark no pending so a late ACK gets dropped rather than
            // satisfying the next sendRequest.
            pending_rcnt = pending_cmd = PENDING_NONE;
            return false;
        }
        pending_rcnt = pending_cmd = PENDING_NONE;
        return true;
    }

    void onNotify(const uint8_t* data, uint16_t len) {
        if (len < 5) return;

        // Measurement frames are pushed by the device unsolicited and at
        // the configured cadence — handle them here, never wake the
        // request semaphore on them.
        if (data[0] == RESPONSE_MEASUREMENT) {
            decodeMeasurement(data, len);
            return;
        }

        // Otherwise it should be a reply to whatever request we last sent.
        // Match by echoed cmd_id only — the device does NOT echo the
        // request's rolling counter back at byte 2 (observed: we send
        // rcnt=0xFE, device replies rcnt=0x00 regardless). godirect-py's
        // _GDX_write_and_check_response and GDXLib's D2PIO_ReadBlocking
        // both skip rcnt validation; mirror that.
        if (pending_cmd == PENDING_NONE) {
            log_w("notify rx with no pending request (op=0x%02X cmd=0x%02X) — dropped",
                  data[0], data[4]);
            return;
        }
        if (data[4] != pending_cmd) {
            log_w("stale resp cmd=0x%02X (want 0x%02X) — dropped",
                  data[4], (unsigned)pending_cmd);
            return;
        }

        if (len > RESP_BUF_SIZE) {
            log_w("response truncated %u -> %u", len, (unsigned)RESP_BUF_SIZE);
        }
        uint16_t copy_len = len > RESP_BUF_SIZE ? RESP_BUF_SIZE : len;
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
            mask        = (sno < MAX_CHANNELS) ? (1u << sno) : 0;
            is_real32   = true;
        } else if (meas_type == MEAS_DROPPED && len >= 9) {
            // Device-reported drop notification. Layout per godirect-py
            // _GDX_decode_meas_response:
            //   [5..6] u16 sensor_mask LE   (which channels lost samples)
            //   [7..8] u16 drop_count LE    (how many samples lost)
            // Add to the local drop counter — distinct from our
            // "previous sample wasn't drained" path above; both feed the
            // same droppedSamples() accessor since the host only cares
            // about the aggregate. Explicit load+store to side-step the
            // C++20 `-Wdeprecated-volatile` on compound assignment.
            uint16_t drop_count = leUnpack<uint16_t>(data + 7);
            dropped = static_cast<uint32_t>(dropped) + drop_count;
            log_w("device-reported %u dropped samples (mask=0x%04X)",
                  drop_count, leUnpack<uint16_t>(data + 5));
            return;
        } else {
            return; // ignore int32 / start_time / period for now
        }

        if (!is_real32 || count == 0 || mask == 0) return;

        // Detect drop: previous sample wasn't drained. Only meaningful
        // for the polling path — push-mode consumers drain via the
        // on_sample callback synchronously below, so the sample_ready
        // flag stays false and this branch never trips spuriously.
        if (sample_ready) dropped = static_cast<uint32_t>(dropped) + 1;

        for (uint8_t n = 0; n < count; ++n) {
            for (uint8_t i = 0; i < MAX_CHANNELS && idx + sizeof(float) <= len; ++i) {
                if (mask & (1u << i)) {
                    float v;
                    memcpy(&v, data + idx, sizeof(float));
                    channels[i].value = v;
                    idx += sizeof(float);
                }
            }
        }

        // Push path. Build the dense Sample once and hand off. Iterate
        // `enabled` (not the frame's `mask`) so the layout matches
        // copySample() — the host wire protocol consumes the same shape
        // regardless of which path filled it. Frames carrying only a
        // subset of enabled channels still publish the most recent
        // value for every enabled channel since channels[i].value
        // sticks until the next frame updates it.
        //
        // When on_sample is bound, polling-mode sample_ready stays
        // false: the push consumer is the canonical drain path.
        // Without this, copySample() never being called would make
        // the drop-detection branch above trip every single frame
        // and spuriously inflate droppedSamples().
        if (on_sample) {
            // Push path. The callback typically forwards into a
            // FreeRTOS queue whose xQueueSend acts as the publication
            // barrier — no explicit fence needed here.
            Sample s = {};
            s.enabled_mask = enabled;
            uint8_t k = 0;
            for (uint8_t i = 0; i < MAX_CHANNELS; ++i) {
                if (enabled & (1u << i)) {
                    s.values[k++] = channels[i].value;
                }
            }
            s.count = k;
            on_sample(s);
        } else {
            // Polling path. Force the channels[].value stores above to
            // commit before the consumer-observed flag flips, otherwise
            // a copySample() racing the notify task on a single core
            // could read sample_ready=true and a not-yet-updated value.
            VERNIER_CROSS_TASK_BARRIER();
            sample_ready = true;
        }
    }

    // Decode CMD_GET_DEVICE_INFO response. Byte layout next to the
    // DEV_INFO_* constants in D2PIOProtocol.h.
    void decodeDeviceInfo() {
        if (resp_len < FRAME_BODY_OFFSET + DEV_INFO_BODY_MIN_SIZE) return;
        const uint8_t* p = resp_buf + FRAME_BODY_OFFSET;
        memset(info.order_code, 0, sizeof(info.order_code));
        memset(info.serial,     0, sizeof(info.serial));
        memset(info.name,       0, sizeof(info.name));
        static_assert(sizeof(info.order_code) >= DEV_INFO_ORDER_CODE_LEN,
                      "info.order_code must fit the on-wire field");
        static_assert(sizeof(info.serial)     >= DEV_INFO_SERIAL_LEN,
                      "info.serial must fit the on-wire field");
        memcpy(info.order_code, p + DEV_INFO_OFF_ORDER_CODE, DEV_INFO_ORDER_CODE_LEN);
        info.order_code[sizeof(info.order_code) - 1] = '\0';
        memcpy(info.serial, p + DEV_INFO_OFF_SERIAL, DEV_INFO_SERIAL_LEN);
        info.serial[sizeof(info.serial) - 1] = '\0';
        // info.name is the same width as the wire field today; copy
        // one fewer byte so the NUL terminator we set below always
        // wins even if the wire string is unterminated.
        memcpy(info.name, p + DEV_INFO_OFF_NAME,
               sizeof(info.name) > DEV_INFO_NAME_LEN
                   ? DEV_INFO_NAME_LEN
                   : sizeof(info.name) - 1);
        info.name[sizeof(info.name) - 1] = '\0';

        // The rest of the response is optional — only populate what we
        // have bytes for, in case some firmware variants ship a shorter
        // struct.
        if (resp_len >= FRAME_BODY_OFFSET + DEV_INFO_BODY_EXT_SIZE) {
            info.vid       = leUnpack<uint16_t>(p + DEV_INFO_OFF_VID);
            uint8_t major1 = p[DEV_INFO_OFF_PRIMARY_CPU];
            uint8_t minor1 = p[DEV_INFO_OFF_PRIMARY_CPU + 1];
            uint8_t major2 = p[DEV_INFO_OFF_SECONDARY_CPU];
            uint8_t minor2 = p[DEV_INFO_OFF_SECONDARY_CPU + 1];
            // Pack as (major << 8) | minor. Build numbers (u16 each
            // following minor) are intentionally dropped — DeviceInfo
            // exposes only the major.minor pair today; widen to a
            // string field if a downstream consumer ever needs the
            // full version triplet.
            info.primary_cpu_version   = static_cast<uint16_t>((major1 << 8) | minor1);
            info.secondary_cpu_version = static_cast<uint16_t>((major2 << 8) | minor2);
        }
        // pid: the protocol has no separate product id field — leave at 0
        // and let downstream code key off (vid, order_code) when it needs
        // a model identifier.
        info.pid = 0;
    }

    // Decode CMD_GET_SENSOR_AVAILABLE_MASK / CMD_GET_DEFAULT_SENSORS_MASK.
    // godirect-py: `mask = struct.unpack("<I", response[6:])[0]` — a
    // single u32 LE at the start of the body.
    bool decodeMask(uint32_t* out_mask) {
        if (resp_len < FRAME_BODY_OFFSET + SENSOR_MASK_BODY_SIZE) return false;
        *out_mask = leUnpack<uint32_t>(resp_buf + FRAME_BODY_OFFSET);
        return true;
    }

    // Decode CMD_GET_SENSOR_INFO response. Field offsets are
    // SENSOR_INFO_OFF_* in D2PIOProtocol.h; total body width is
    // SENSOR_INFO_BODY_SIZE (148 B).
    bool decodeSensorInfo() {
        if (resp_len < FRAME_BODY_OFFSET + SENSOR_INFO_BODY_SIZE) return false;
        const uint8_t* p = resp_buf + FRAME_BODY_OFFSET;
        int8_t   sensor_no   = static_cast<int8_t>(p[0]);
        // p[1] = spare
        uint32_t sensor_id   = leUnpack<uint32_t>(p + 2);
        uint8_t  meas_type   = p[6];
        uint8_t  sampling    = p[7];
        const uint8_t* desc  = p + SENSOR_INFO_OFF_DESC;
        const uint8_t* units = p + SENSOR_INFO_OFF_UNITS;
        const uint8_t* dbls  = p + SENSOR_INFO_OFF_DBLS;
        const uint8_t* tail  = p + SENSOR_INFO_OFF_TAIL;

        if (sensor_no < 0 || sensor_no >= static_cast<int>(MAX_CHANNELS)) {
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
               sizeof(c.description) > SENSOR_INFO_DESC_LEN
                   ? SENSOR_INFO_DESC_LEN
                   : sizeof(c.description) - 1);
        c.description[sizeof(c.description) - 1] = '\0';
        memcpy(c.units, units,
               sizeof(c.units) > SENSOR_INFO_UNITS_LEN
                   ? SENSOR_INFO_UNITS_LEN
                   : sizeof(c.units) - 1);
        c.units[sizeof(c.units) - 1] = '\0';

        // doubles: uncertainty, min, max — narrow to f32 for the channel
        // struct (existing API contract).
        double d;
        constexpr size_t kDbl = sizeof(double);
        memcpy(&d, dbls + 0 * kDbl, kDbl); c.measurement_uncertainty = static_cast<float>(d);
        memcpy(&d, dbls + 1 * kDbl, kDbl); c.min_measurement         = static_cast<float>(d);
        memcpy(&d, dbls + 2 * kDbl, kDbl); c.max_measurement         = static_cast<float>(d);

        // Tail layout: <IQIII> — u32 min_period, u64 max_period,
        // u32 typ_period, u32 period_granularity, u32 mut_excl_mask.
        constexpr uint8_t kTailMinPeriod  = 0;
        constexpr uint8_t kTailMaxPeriod  = kTailMinPeriod  + sizeof(uint32_t);
        constexpr uint8_t kTailTypPeriod  = kTailMaxPeriod  + sizeof(uint64_t);
        constexpr uint8_t kTailGranPeriod = kTailTypPeriod  + sizeof(uint32_t);
        constexpr uint8_t kTailMutExMask  = kTailGranPeriod + sizeof(uint32_t);
        c.min_period_us         = leUnpack<uint32_t>(tail + kTailMinPeriod);
        uint64_t maxp           = leUnpack<uint64_t>(tail + kTailMaxPeriod);
        c.max_period_us         = (maxp > 0xFFFFFFFFu) ? 0xFFFFFFFFu
                                                       : static_cast<uint32_t>(maxp);
        c.typ_period_us         = leUnpack<uint32_t>(tail + kTailTypPeriod);
        c.period_granularity_us = leUnpack<uint32_t>(tail + kTailGranPeriod);
        c.mutual_exclusion_mask = leUnpack<uint32_t>(tail + kTailMutExMask);
        return true;
    }
};

GoGoVernier::GoGoVernier() : _impl(new Impl()) {
    _impl->resp_sem      = xSemaphoreCreateBinary();
    _impl->req_mutex     = xSemaphoreCreateMutex();
    // Recursive so open() can call close() internally while holding the
    // session mutex without deadlocking.
    _impl->session_mutex = xSemaphoreCreateRecursiveMutex();
}

GoGoVernier::~GoGoVernier() {
    if (_impl->resp_sem)      vSemaphoreDelete(_impl->resp_sem);
    if (_impl->req_mutex)     vSemaphoreDelete(_impl->req_mutex);
    if (_impl->session_mutex) vSemaphoreDelete(_impl->session_mutex);
    delete _impl;
}

bool GoGoVernier::open(const char* name) {
    // Serialise the entire session phase. Concurrent open() / start() /
    // stop() / close() callers block here. Recursive mutex so the
    // close() call below can re-take without deadlocking.
    xSemaphoreTakeRecursive(_impl->session_mutex, portMAX_DELAY);
    struct GiveOnExit {
        SemaphoreHandle_t m;
        ~GiveOnExit() { xSemaphoreGiveRecursive(m); }
    } _scope { _impl->session_mutex };

    if (_impl->connected) close();

    log_i("open name=\"%s\"", name ? name : "");
    if (!_impl->xport.connect(name, OPEN_SCAN_MS)) {
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
    _impl->rolling_counter = INITIAL_ROLLING_COUNTER;

    // ---- D2PIO handshake ----
    uint8_t buf[MAX_REQUEST_FRAME_SIZE];

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
        if (!_impl->sendRequest(buf, n, REQUEST_TIMEOUT_MS)) {
            log_e("CMD_INIT failed");
            close();
            return false;
        }
        log_d("CMD_INIT ack");
    }

    // CMD_GET_DEVICE_INFO — observed to take noticeably longer to reply
    // than the other queries on GDX-LC (well beyond the 3 s default).
    // Use a wider timeout. Failure is non-fatal: the cached scan-time
    // peer name is preserved and downstream channel discovery continues.
    {
        uint8_t n = _impl->encode(buf, CMD_GET_DEVICE_INFO, nullptr, 0);
        if (!_impl->sendRequest(buf, n, DEVICE_INFO_TIMEOUT_MS)) {
            log_w("CMD_GET_DEVICE_INFO timed out — keeping advertised name only");
        } else {
            _impl->decodeDeviceInfo();
            log_i("device order=\"%s\" serial=\"%s\" name=\"%s\"",
                  _impl->info.order_code, _impl->info.serial, _impl->info.name);
        }
    }

    // CMD_GET_SENSOR_AVAILABLE_MASK
    {
        uint8_t n = _impl->encode(buf, CMD_GET_SENSOR_AVAILABLE_MASK, nullptr, 0);
        if (!_impl->sendRequest(buf, n, REQUEST_TIMEOUT_MS)) {
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
    for (uint8_t i = 0; i < MAX_CHANNELS; ++i) {
        if (!(_impl->available & (1u << i))) continue;
        uint8_t payload = i;
        uint8_t n = _impl->encode(buf, CMD_GET_SENSOR_INFO, &payload, 1);
        if (!_impl->sendRequest(buf, n, REQUEST_TIMEOUT_MS)) {
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

    // Default-enable: walk available bits in ascending order and skip any
    // whose mutual_exclusion_mask intersects bits already enabled. This
    // gives a deterministic default for devices like GDX-3MG (low+high
    // range mutually exclusive 3+3) and GDX-ACC (accel ranges) — first
    // bit wins, conflicts stay off. Caller can flip the choice later via
    // disable/enable.
    _impl->enabled = 0;
    for (uint8_t i = 0; i < MAX_CHANNELS; ++i) {
        if (!(_impl->available & (1u << i))) continue;
        uint32_t conflict = _impl->channels[i].mutual_exclusion_mask & ~(1u << i);
        if (conflict & _impl->enabled) {
            _impl->channels[i].enabled = false;
            log_d("ch%u skipped — conflicts with enabled mask 0x%08X", i,
                  (unsigned)_impl->enabled);
            continue;
        }
        _impl->enabled |= (1u << i);
        _impl->channels[i].enabled = true;
    }

    _impl->ready = true;
    return true;
}

void GoGoVernier::close() {
    xSemaphoreTakeRecursive(_impl->session_mutex, portMAX_DELAY);
    struct GiveOnExit {
        SemaphoreHandle_t m;
        ~GiveOnExit() { xSemaphoreGiveRecursive(m); }
    } _scope { _impl->session_mutex };

    log_i("close");
    if (_impl->streaming) stop();
    _impl->xport.unsubscribe();
    _impl->xport.disconnect();
    _impl->ready         = false;
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
bool GoGoVernier::isReady() const                    { return _impl->ready; }
bool GoGoVernier::isScanning() const                 { return _impl->scanning; }
void GoGoVernier::abortScan()                        {}

bool GoGoVernier::enableSensor(uint8_t ch) {
    if (ch >= MAX_CHANNELS) return false;
    if (!(_impl->available & (1u << ch))) return false;
    // Per spec, mutual_exclusion_mask lists other channels that cannot
    // coexist with this one (e.g. GDX-3MG's low/high range pairs,
    // GDX-ACC's accel ranges). Strip the self-bit defensively (godirect-py
    // never sets it, but the firmware-side struct has been seen with
    // conflicting layouts on third-party gear). Refuse rather than
    // silently disabling the conflict; caller must disableSensor() first
    // to make the choice explicit.
    uint32_t conflict = _impl->channels[ch].mutual_exclusion_mask & ~(1u << ch);
    if (conflict & _impl->enabled) {
        log_w("enableSensor(%u) conflicts with enabled mask 0x%08X (mut-ex 0x%08X)",
              ch, (unsigned)_impl->enabled, (unsigned)conflict);
        return false;
    }
    _impl->enabled |= (1u << ch);
    _impl->channels[ch].enabled = true;
    return true;
}

bool GoGoVernier::disableSensor(uint8_t ch) {
    if (ch >= MAX_CHANNELS) return false;
    _impl->enabled &= ~(1u << ch);
    _impl->channels[ch].enabled = false;
    return true;
}

uint32_t GoGoVernier::availableChannelMask() const   { return _impl->available; }
uint32_t GoGoVernier::enabledChannelMask() const     { return _impl->enabled; }
uint8_t  GoGoVernier::channelCount() const           { return _impl->channel_count; }

const ChannelInfo* GoGoVernier::channel(uint8_t ch) const {
    if (ch >= MAX_CHANNELS) return nullptr;
    return &_impl->channels[ch];
}

bool GoGoVernier::start(uint32_t period_ms) {
    // Take session mutex BEFORE checking state — concurrent open() may be
    // mid-handshake; we want to wait for it to finish (so available mask
    // is populated) rather than racing through with mask=0 and timing
    // out CMD_START_MEASUREMENTS.
    xSemaphoreTakeRecursive(_impl->session_mutex, portMAX_DELAY);
    struct GiveOnExit {
        SemaphoreHandle_t m;
        ~GiveOnExit() { xSemaphoreGiveRecursive(m); }
    } _scope { _impl->session_mutex };

    if (!_impl->connected) return false;
    if (_impl->available == 0) {
        log_w("start() refused: handshake incomplete (available mask=0)");
        return false;
    }
    if (_impl->streaming) {
        // Already streaming. Idempotent — caller is e.g. a duplicate
        // host-MCU connectAndReport. Don't re-encode SET_PERIOD +
        // START_MEAS, that wastes wire bandwidth and confuses the
        // device's stream state.
        log_d("start() called while already streaming — no-op");
        return true;
    }
    if (period_ms) _impl->period_ms = static_cast<uint16_t>(period_ms);

    uint8_t buf[MAX_REQUEST_FRAME_SIZE];

    // CMD_SET_MEASUREMENT_PERIOD — see REQ_PAYLOAD_SET_PERIOD_SIZE layout
    // beside the constant in D2PIOProtocol.h.
    {
        uint32_t period_us = static_cast<uint32_t>(_impl->period_ms) * US_PER_MS;
        uint8_t payload[REQ_PAYLOAD_SET_PERIOD_SIZE] = {
            REQ_FLAG_DEVICE_WIDE, REQ_SEL_SET_PERIOD,
            static_cast<uint8_t>((period_us >>  0) & 0xFF),
            static_cast<uint8_t>((period_us >>  8) & 0xFF),
            static_cast<uint8_t>((period_us >> 16) & 0xFF),
            static_cast<uint8_t>((period_us >> 24) & 0xFF),
            0, 0, 0, 0
        };
        uint8_t n = _impl->encode(buf, CMD_SET_MEASUREMENT_PERIOD,
                                  payload, sizeof(payload));
        if (!_impl->sendRequest(buf, n, REQUEST_TIMEOUT_MS)) {
            log_e("CMD_SET_MEASUREMENT_PERIOD failed");
            return false;
        }
    }

    // CMD_START_MEASUREMENTS — see REQ_PAYLOAD_START_MEAS_SIZE layout
    // beside the constant in D2PIOProtocol.h.
    {
        uint32_t mask = _impl->enabled;
        uint8_t payload[REQ_PAYLOAD_START_MEAS_SIZE] = {
            REQ_FLAG_DEVICE_WIDE, REQ_SEL_START_MEAS,
            static_cast<uint8_t>((mask >>  0) & 0xFF),
            static_cast<uint8_t>((mask >>  8) & 0xFF),
            static_cast<uint8_t>((mask >> 16) & 0xFF),
            static_cast<uint8_t>((mask >> 24) & 0xFF),
            0, 0, 0, 0, 0, 0, 0, 0
        };
        uint8_t n = _impl->encode(buf, CMD_START_MEASUREMENTS,
                                  payload, sizeof(payload));
        if (!_impl->sendRequest(buf, n, REQUEST_TIMEOUT_MS)) {
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
    xSemaphoreTakeRecursive(_impl->session_mutex, portMAX_DELAY);
    struct GiveOnExit {
        SemaphoreHandle_t m;
        ~GiveOnExit() { xSemaphoreGiveRecursive(m); }
    } _scope { _impl->session_mutex };

    if (!_impl->connected || !_impl->streaming) {
        _impl->streaming = false;
        return true;
    }
    uint8_t buf[MAX_REQUEST_FRAME_SIZE];
    // CMD_STOP_MEASUREMENTS — see REQ_PAYLOAD_STOP_MEAS_SIZE layout
    // beside the constant in D2PIOProtocol.h.
    uint8_t payload[REQ_PAYLOAD_STOP_MEAS_SIZE] = {
        REQ_FLAG_DEVICE_WIDE, REQ_SEL_STOP_MEAS,
        REQ_PAD_BYTE, REQ_PAD_BYTE, REQ_PAD_BYTE, REQ_PAD_BYTE
    };
    uint8_t n = _impl->encode(buf, CMD_STOP_MEASUREMENTS, payload, sizeof(payload));
    bool ok = _impl->sendRequest(buf, n, REQUEST_TIMEOUT_MS);
    _impl->streaming = false;
    return ok;
}

bool GoGoVernier::isStreaming() const                 { return _impl->streaming; }

bool GoGoVernier::sampleReady() const                 { return _impl->sample_ready; }
bool GoGoVernier::copySample(float* out, uint8_t& count) {
    if (!_impl->sample_ready) { count = 0; return false; }
    // Pair with the producer-side fence in decodeMeasurement(): make
    // sure we read the freshly-written channels[].value buffer, not a
    // pre-flag-flip snapshot the compiler kept in a register.
    VERNIER_CROSS_TASK_BARRIER();
    uint8_t idx = 0;
    for (uint8_t i = 0; i < MAX_CHANNELS; ++i) {
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
    if (ch >= MAX_CHANNELS) return 0.0f;
    return _impl->channels[ch].value;
}
uint32_t GoGoVernier::droppedSamples() const          { return _impl->dropped; }

void GoGoVernier::onSample(SampleCallback cb)        { _impl->on_sample = std::move(cb); }

const DeviceInfo&   GoGoVernier::deviceInfo() const   { return _impl->info; }
const DeviceStatus& GoGoVernier::status() const       { return _impl->status; }

bool GoGoVernier::refreshStatus() {
    if (!_impl->connected) return false;

    // Always refresh RSSI from the live BLE link — cheap and doesn't
    // require a round-trip to the device.
    _impl->status.rssi = static_cast<int8_t>(_impl->xport.rssi());

    // CMD_GET_STATUS — full response body layout next to STATUS_OFF_*
    // in D2PIOProtocol.h. We only need battery + charger today.
    uint8_t buf[FRAME_HEADER_SIZE];
    uint8_t n = _impl->encode(buf, CMD_GET_STATUS, nullptr, 0);
    if (!_impl->sendRequest(buf, n, REQUEST_TIMEOUT_MS)) {
        log_w("CMD_GET_STATUS timed out — keeping previous battery/charger");
        return false;
    }
    if (_impl->resp_len < FRAME_BODY_OFFSET + STATUS_BODY_MIN_SIZE) {
        log_w("CMD_GET_STATUS short response (resp_len=%u)", _impl->resp_len);
        return false;
    }
    _impl->status.battery_percent =
        _impl->resp_buf[FRAME_BODY_OFFSET + STATUS_OFF_BATTERY];
    const uint8_t cs =
        _impl->resp_buf[FRAME_BODY_OFFSET + STATUS_OFF_CHARGER];
    _impl->status.charger_state = (cs <= static_cast<uint8_t>(CHARGER_ERROR))
                                      ? static_cast<ChargerState>(cs)
                                      : CHARGER_ERROR;
    return true;
}

}  // namespace gogo_vernier
