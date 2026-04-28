// GoGoVernier — Vernier Go Direct (D2PIO) BLE client for GoGo Board 7.
//
// Phase 0 skeleton. Public API only — no implementation yet. The shape mirrors
// what `vernier-firmware/src/vernier-adapter.cpp` consumes today against
// MomePP/GDXLib, with the multi-device + 32-channel goals from
// .claude/plans/gdxlib-rewrite.md baked in from the start.
//
// See NOTICES.md for upstream attribution.

#pragma once

#include <stdint.h>

#include "D2PIOProtocol.h"

namespace gogo_vernier {

// One row of GET_SENSOR_INFO. Field set tracks
// godirect-py/godirect/sensor.py:Sensor verbatim. All fields owned by the
// session; lifetimes match the BLE connection.
struct ChannelInfo {
    uint8_t  number;                       // 0..31
    char     description[32];              // utf-8, null-terminated
    char     units[16];                    // utf-8, null-terminated
    uint32_t sensor_id;                    // Vernier catalogue id
    uint8_t  measurement_type;             // MeasurementType enum value
    uint8_t  sampling_mode;                // periodic / aperiodic
    float    measurement_uncertainty;
    float    min_measurement;
    float    max_measurement;
    uint32_t typ_period_us;
    uint32_t min_period_us;
    uint32_t max_period_us;
    uint32_t period_granularity_us;
    uint32_t mutual_exclusion_mask;        // bits set = sensors that conflict
    bool     enabled;
    float    value;                        // last decoded sample
};

struct DeviceInfo {
    char     name[32];
    char     order_code[16];
    char     serial[16];
    uint16_t vid;
    uint16_t pid;
    uint16_t primary_cpu_version;
    uint16_t secondary_cpu_version;
};

struct DeviceStatus {
    uint8_t      battery_percent;
    ChargerState charger_state;
    int8_t       rssi;
};

class GoGoVernier {
public:
    GoGoVernier();
    ~GoGoVernier();

    // Discovery / connection lifecycle. `name` may be the empty string for
    // proximity (highest-RSSI nearby) or a saved "GDX-XXX 0123ABCD" name.
    bool open(const char* name);
    void close();
    bool isConnected() const;
    bool isScanning() const;
    void abortScan();

    // Channel management. `enableSensor(255)` enables the device-default set
    // (mirrors godirect-py's enable_default_sensors).
    bool enableSensor(uint8_t channel);
    bool disableSensor(uint8_t channel);
    uint32_t availableChannelMask() const;
    uint32_t enabledChannelMask() const;
    uint8_t  channelCount() const;
    const ChannelInfo* channel(uint8_t channel) const;

    // Streaming. period_ms == 0 → use device's typical period.
    bool start(uint32_t period_ms);
    bool stop();
    bool isStreaming() const;

    // Sample retrieval — Phase 0/1 keeps the polling shape; Phase 3 adds an
    // onSample(std::function<...>) callback path.
    bool sampleReady() const;
    bool copySample(float* out, uint8_t& count);
    float measurement(uint8_t channel) const;

    // Monotonic counter — incremented when a new sample frame arrives before
    // the previous one was drained via copySample(). Reset on open().
    uint32_t droppedSamples() const;

    // Identity + status accessors. Cached at connect; refreshed by
    // refreshStatus().
    const DeviceInfo& deviceInfo() const;
    const DeviceStatus& status() const;
    bool refreshStatus();

private:
    // Implementation hidden behind a forward-declared session pimpl so the
    // public header doesn't pull in NimBLE.
    struct Impl;
    Impl* _impl;
};

}  // namespace gogo_vernier
