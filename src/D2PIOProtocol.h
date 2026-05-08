// D2PIO protocol constants — Vernier Go Direct over BLE.
//
// Authoritative source: VernierST/godirect-py (BSD-3-Clause).
// Cross-checked against Vernier-Science-Education/GDXLib (BSD-3-Clause).
// See NOTICES.md.

#pragma once

#include <stdint.h>

namespace gogo_vernier {

// BLE GATT identifiers (128-bit, big-endian string form).
constexpr const char* GDX_SERVICE_UUID       = "d91714ef-28b9-4f91-ba16-f0d9a604f112";
constexpr const char* GDX_COMMAND_CHAR_UUID  = "f4bf14a6-c7d5-4b6d-8aa8-df1a7c83adcb";
constexpr const char* GDX_RESPONSE_CHAR_UUID = "b41e6675-a329-40e0-aa01-44d2f444babe";

// Frame layout on both characteristics (verbatim from godirect-py
// `Device._GDX_init` and friends):
//   byte 0  : FRAME_HEADER (constant 0x58)
//   byte 1  : total length in bytes (the entire frame including this byte)
//   byte 2  : rolling counter — request-side decrements 0xFF..0x00, wraps
//   byte 3  : checksum (placeholder until calculateChecksum runs)
//   byte 4  : command id (CMD_ID_*) on writes; response op on reads
//   byte 5..N-1 : payload, command-specific
constexpr uint8_t FRAME_HEADER      = 0x58;
constexpr uint8_t FRAME_HEADER_SIZE = 5;  // [magic][len][rcnt][checksum][op]

// Command ids — godirect-py/godirect/device.py
enum CmdId : uint8_t {
    // Opcode 0x10 isn't in Vernier's published spec; godirect-py
    // hardcodes it as CMD_ID_GET_STATUS. Response carries status +
    // CPU versions + battery percent + charger state.
    CMD_GET_STATUS                 = 0x10,
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
constexpr uint8_t RESPONSE_MEASUREMENT = 0x20;

enum ChargerState : uint8_t {
    CHARGER_IDLE     = 0,
    CHARGER_CHARGING = 1,
    CHARGER_COMPLETE = 2,
    CHARGER_ERROR    = 3,
};

// Hard limits dictated by the protocol, not by our implementation.
//
// MAX_CHANNELS: GET_SENSOR_AVAILABLE_MASK returns a 32-bit mask. We size every
// per-channel array to this. The largest GDX device today (GDX-ACC) reports
// 9 channels; godirect-py iterates 0..31 with the same bound.
constexpr uint8_t MAX_CHANNELS = 32;

// Frame checksum, ported byte-for-byte from godirect-py
// (`Device._GDX_calculate_checksum`):
//
//   length = buff[1]
//   checksum = -buff[3]                # cancels the placeholder
//   for i in range(0, length):
//       checksum = (checksum + buff[i]) & 0xFF
//
// Equivalent to: low 8 bits of the sum of every byte EXCEPT the checksum
// byte itself (byte 3). Plain 8-bit sum — NOT a 1's complement, despite
// what an earlier draft of the spec doc claimed.
//
// Call with `total_len = buf[1]` and an already-populated frame whose
// `buf[3]` is the placeholder checksum (typically 0, doesn't matter —
// the `-buf[3]` cancellation makes this work for any prior value).
inline uint8_t calculateChecksum(const uint8_t* buf, uint8_t total_len) {
    int s = -static_cast<int>(buf[3]);
    for (uint8_t i = 0; i < total_len; ++i) s += buf[i];
    return static_cast<uint8_t>(s & 0xFF);
}

}  // namespace gogo_vernier
