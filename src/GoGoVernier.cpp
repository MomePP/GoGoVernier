// GoGoVernier — Phase 1.
//
// open() actually connects + subscribes via BundledBleXport. Protocol
// exchange (Init / GetDeviceInfo / GetSensorAvailableMask / etc.) lands
// in Phase 2. For now sample frames are not decoded, but the BLE link
// reaches a stable connected+subscribed state and basic peer metadata
// (address, scan-time RSSI) is populated.

#include "GoGoVernier.h"

#include <string.h>

#include "transport/BundledBleXport.h"

namespace gogo_vernier {

struct GoGoVernier::Impl {
    BundledBleXport xport;

    bool         connected      = false;
    bool         scanning       = false;
    bool         streaming      = false;
    bool         sample_ready   = false;
    uint32_t     dropped        = 0;
    uint32_t     available      = 0;
    uint32_t     enabled        = 0;
    uint8_t      channel_count  = 0;
    ChannelInfo  channels[kMaxChannels] = {};
    DeviceInfo   info           = {};
    DeviceStatus status         = {};
};

GoGoVernier::GoGoVernier() : _impl(new Impl()) {}

GoGoVernier::~GoGoVernier() { delete _impl; }

bool GoGoVernier::open(const char* name) {
    if (_impl->connected) close();

    constexpr uint32_t kScanMs = 5000;
    if (!_impl->xport.connect(name, kScanMs)) {
        return false;
    }

    // Subscribe to the response characteristic. Phase 2 will plug a real
    // D2PIO frame decoder into this callback; for now we just count bytes.
    _impl->xport.subscribe([](const uint8_t* /*data*/, uint16_t /*len*/) {
        // Phase-2 stub.
    });

    // Cache what we already know without doing any protocol traffic.
    const char* pname = _impl->xport.peerName();
    const char* paddr = _impl->xport.peerAddress();
    if (pname && *pname) {
        strncpy(_impl->info.name, pname, sizeof(_impl->info.name) - 1);
    } else if (paddr) {
        strncpy(_impl->info.name, paddr, sizeof(_impl->info.name) - 1);
    }
    _impl->status.rssi = static_cast<int8_t>(_impl->xport.rssi());

    _impl->connected = true;
    _impl->dropped   = 0;
    return true;
}

void GoGoVernier::close() {
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

bool GoGoVernier::enableSensor(uint8_t /*ch*/)       { return false; }
bool GoGoVernier::disableSensor(uint8_t /*ch*/)      { return false; }
uint32_t GoGoVernier::availableChannelMask() const   { return _impl->available; }
uint32_t GoGoVernier::enabledChannelMask() const     { return _impl->enabled; }
uint8_t  GoGoVernier::channelCount() const           { return _impl->channel_count; }

const ChannelInfo* GoGoVernier::channel(uint8_t ch) const {
    if (ch >= kMaxChannels) return nullptr;
    return &_impl->channels[ch];
}

bool GoGoVernier::start(uint32_t /*period_ms*/)      { return false; }
bool GoGoVernier::stop()                              { return false; }
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
bool                GoGoVernier::refreshStatus()      { return false; }

}  // namespace gogo_vernier
