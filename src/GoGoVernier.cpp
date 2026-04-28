// GoGoVernier — Phase 0 stub.
//
// All public methods compile to no-ops or sentinel values so the rest of the
// firmware can link against the new lib while the real protocol + transport
// layers land in subsequent phases (see .claude/plans/gdxlib-rewrite.md).

#include "GoGoVernier.h"

namespace gogo_vernier {

struct GoGoVernier::Impl {
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

bool GoGoVernier::open(const char* /*name*/)        { return false; }
void GoGoVernier::close()                            {}
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
