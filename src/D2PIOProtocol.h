// D2PIO protocol constants — Vernier Go Direct over BLE.
//
// Authoritative source: VernierST/godirect-py (BSD-3-Clause).
// Cross-checked against Vernier-Science-Education/GDXLib (BSD-3-Clause).
// See NOTICES.md.

#pragma once

#include <stdint.h>

namespace gogo_vernier {

// BLE GATT identifiers (128-bit, big-endian string form).
constexpr const char* kGdxServiceUuid       = "d91714ef-28b9-4f91-ba16-f0d9a604f112";
constexpr const char* kGdxCommandCharUuid   = "f4bf14a6-c7d5-4b6d-8aa8-df1a7c83adcb";
constexpr const char* kGdxResponseCharUuid  = "b41e6675-a329-40e0-aa01-44d2f444babe";

// Frame layout on both characteristics:
//   byte 0  : kFrameHeader (constant)
//   byte 1  : total length in bytes (header through checksum, inclusive)
//   byte 2  : rolling counter — request-side decrements 0xFF..0x00, wraps
//   byte 3  : command id  (CMD_ID_*) on writes; response op on reads
//   byte 4..N-2 : payload, command-specific
//   byte N-1: 1's-complement checksum of bytes 0..N-2
constexpr uint8_t kFrameHeader = 0x58;

// Command ids — godirect-py/godirect/device.py
enum CmdId : uint8_t {
    CMD_START_MEASUREMENTS         = 0x18,
    CMD_STOP_MEASUREMENTS          = 0x19,
    CMD_INIT                       = 0x1A,
    CMD_SET_MEASUREMENT_PERIOD     = 0x1B,
    CMD_GET_SENSOR_INFO            = 0x50,
    CMD_GET_SENSOR_AVAILABLE_MASK  = 0x51,
    CMD_DISCONNECT                 = 0x54,
    CMD_GET_DEVICE_INFO            = 0x55,
    CMD_GET_DEFAULT_SENSORS_MASK   = 0x56,
};

// Live-measurement frame TLV tags (see RESPONSE_MEASUREMENT below) — same source.
enum MeasurementType : uint8_t {
    MEAS_NORMAL_REAL32             = 0x06,
    MEAS_WIDE_REAL32               = 0x07,
    MEAS_SINGLE_CHANNEL_REAL32     = 0x08,
    MEAS_SINGLE_CHANNEL_INT32      = 0x09,
    MEAS_APERIODIC_REAL32          = 0x0a,
    MEAS_APERIODIC_INT32           = 0x0b,
    MEAS_START_TIME                = 0x0c,
    MEAS_DROPPED                   = 0x0d,
    MEAS_PERIOD                    = 0x0e,
};

// Top-level response opcodes that can appear in byte 3 of an incoming frame.
constexpr uint8_t kResponseMeasurement = 0x20;

enum ChargerState : uint8_t {
    CHARGER_IDLE     = 0,
    CHARGER_CHARGING = 1,
    CHARGER_COMPLETE = 2,
    CHARGER_ERROR    = 3,
};

// Hard limits dictated by the protocol, not by our implementation.
//
// kMaxChannels: GET_SENSOR_AVAILABLE_MASK returns a 32-bit mask. We size every
// per-channel array to this. The largest GDX device today (GDX-ACC) reports
// 9 channels; godirect-py iterates 0..31 with the same bound.
constexpr uint8_t kMaxChannels = 32;

// 1's-complement checksum used by both godirect-py
// (`Device._GDX_calculate_checksum`) and GDXLib (`D2PIO_CalculateChecksum`).
// Sums bytes 0..len-2, returns ~sum & 0xFF.
inline uint8_t calculateChecksum(const uint8_t* buf, uint8_t len_inclusive) {
    uint16_t s = 0;
    for (uint8_t i = 0; i + 1 < len_inclusive; ++i) s += buf[i];
    return static_cast<uint8_t>(~s & 0xFF);
}

}  // namespace gogo_vernier
