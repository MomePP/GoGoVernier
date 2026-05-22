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

// Offset of the response body (the command-specific payload) inside a
// received frame — the first 5 bytes are FRAME_HEADER_SIZE plus one
// echoed cmd_id at byte 4, so the body starts at byte 6. All decode*
// helpers compute their inner offsets relative to this.
constexpr uint8_t FRAME_BODY_OFFSET = 6;

// Largest request frame we build on the stack. CMD_INIT (25-byte
// payload) is the widest request body; everything else is ≤ 14 bytes
// payload plus the 5-byte header. 64 leaves comfortable headroom and
// keeps the same buffer size across every request site.
constexpr uint8_t MAX_REQUEST_FRAME_SIZE = 64;

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

// --- Request payload selector / framing bytes -----------------------------
//
// Several D2PIO commands take a two-byte prefix inside their payload:
//   byte 0 : sensor-scope flag — always 0xFF in godirect-py for the
//            "applies to the whole device, not a specific channel" form.
//   byte 1 : sub-command selector — disambiguates the operation within
//            the same opcode (period set vs. start vs. stop).
// godirect-py hardcodes these as bare literals; we name them so the
// payload-build sites in GoGoVernier.cpp don't carry magic bytes.
constexpr uint8_t REQ_FLAG_DEVICE_WIDE       = 0xFF;
constexpr uint8_t REQ_SEL_SET_PERIOD         = 0x00;  // inside CMD_SET_MEASUREMENT_PERIOD
constexpr uint8_t REQ_SEL_START_MEAS         = 0x01;  // inside CMD_START_MEASUREMENTS
constexpr uint8_t REQ_SEL_STOP_MEAS          = 0x00;  // inside CMD_STOP_MEASUREMENTS
constexpr uint8_t REQ_PAD_BYTE               = 0xFF;  // CMD_STOP_MEASUREMENTS trailing pad

// CMD_SET_MEASUREMENT_PERIOD payload:
//   [REQ_FLAG_DEVICE_WIDE][REQ_SEL_SET_PERIOD][u32_le period_us][4× 0x00]
constexpr uint8_t REQ_PAYLOAD_SET_PERIOD_SIZE = 10;

// CMD_START_MEASUREMENTS payload:
//   [REQ_FLAG_DEVICE_WIDE][REQ_SEL_START_MEAS][u32_le enabled_mask][8× 0x00]
constexpr uint8_t REQ_PAYLOAD_START_MEAS_SIZE = 14;

// CMD_STOP_MEASUREMENTS payload:
//   [REQ_FLAG_DEVICE_WIDE][REQ_SEL_STOP_MEAS][4× REQ_PAD_BYTE]
constexpr uint8_t REQ_PAYLOAD_STOP_MEAS_SIZE  = 6;

// --- Response body layouts ------------------------------------------------
//
// All offsets below are measured from the start of the body — i.e. they
// assume the caller has already advanced past FRAME_BODY_OFFSET bytes
// from the frame start.

// CMD_GET_DEVICE_INFO response (Vernier struct, after the 6-byte header):
//   off  0..15 : order_code  (16B fixed-width string)
//   off 16..31 : serial      (16B)
//   off 32..63 : name        (32B)
//   off 64..65 : manufacturerId (u16 LE)  — exposed as DeviceInfo::vid
//   off 70     : primary_cpu_major (u8)
//   off 71     : primary_cpu_minor (u8)
//   off 74     : secondary_cpu_major (u8)
//   off 75     : secondary_cpu_minor (u8)
// godirect-py keeps unpacking past offset 78 (BLE addr, NVRAM size,
// description string) — we don't need any of it today.
constexpr uint16_t DEV_INFO_BODY_MIN_SIZE      = 64;   // need at least the 3 strings
constexpr uint16_t DEV_INFO_BODY_EXT_SIZE      = 78;   // through the secondary CPU version
constexpr uint8_t  DEV_INFO_OFF_ORDER_CODE     =  0;
constexpr uint8_t  DEV_INFO_ORDER_CODE_LEN     = 16;
constexpr uint8_t  DEV_INFO_OFF_SERIAL         = 16;
constexpr uint8_t  DEV_INFO_SERIAL_LEN         = 16;
constexpr uint8_t  DEV_INFO_OFF_NAME           = 32;
constexpr uint8_t  DEV_INFO_NAME_LEN           = 32;
constexpr uint8_t  DEV_INFO_OFF_VID            = 64;
constexpr uint8_t  DEV_INFO_OFF_PRIMARY_CPU    = 70;   // major (B), minor (B), build (H)
constexpr uint8_t  DEV_INFO_OFF_SECONDARY_CPU  = 74;

// CMD_GET_SENSOR_AVAILABLE_MASK / CMD_GET_DEFAULT_SENSORS_MASK response:
//   off 0..3 : u32_le mask
constexpr uint16_t SENSOR_MASK_BODY_SIZE       = 4;

// CMD_GET_SENSOR_INFO response — struct `<bBIBB60s32sdddIQIII`:
//   off  0     : sensor_no (i8)
//   off  1     : spare
//   off  2..5  : sensor_id (u32 LE)
//   off  6     : measurement_type (u8)
//   off  7     : sampling_mode (u8)
//   off  8..67 : description (60B fixed-width string)
//   off 68..99 : units       (32B)
//   off 100..107 : measurement_uncertainty (f64 LE)
//   off 108..115 : min_measurement         (f64 LE)
//   off 116..123 : max_measurement         (f64 LE)
//   off 124..127 : min_period_us (u32 LE)
//   off 128..135 : max_period_us (u64 LE — clamped to u32 in ChannelInfo)
//   off 136..139 : typ_period_us (u32 LE)
//   off 140..143 : period_granularity_us (u32 LE)
//   off 144..147 : mutual_exclusion_mask (u32 LE)
constexpr uint8_t  SENSOR_INFO_DESC_LEN        = 60;
constexpr uint8_t  SENSOR_INFO_UNITS_LEN       = 32;
constexpr uint8_t  SENSOR_INFO_DBL_TRIPLET_LEN = 24;   // 3 × f64
constexpr uint8_t  SENSOR_INFO_OFF_DESC        = 8;
constexpr uint8_t  SENSOR_INFO_OFF_UNITS       = SENSOR_INFO_OFF_DESC + SENSOR_INFO_DESC_LEN;
constexpr uint8_t  SENSOR_INFO_OFF_DBLS        = SENSOR_INFO_OFF_UNITS + SENSOR_INFO_UNITS_LEN;
constexpr uint8_t  SENSOR_INFO_OFF_TAIL        = SENSOR_INFO_OFF_DBLS  + SENSOR_INFO_DBL_TRIPLET_LEN;
constexpr uint16_t SENSOR_INFO_BODY_SIZE       = 148;

// CMD_GET_STATUS response (Vernier opcode 0x10, undocumented but stable):
//   off 0     : status (B)
//   off 1     : spare  (B)
//   off 2..3  : primaryCpuMajor (B), primaryCpuMinor (B)
//   off 4..5  : primaryCpuBuild (u16 LE)
//   off 6..7  : secondaryCpuMajor (B), secondaryCpuMinor (B)
//   off 8..9  : secondaryCpuBuild (u16 LE)
//   off 10    : batteryLevelPercent (B)
//   off 11    : chargerState (B; see ChargerState enum above)
constexpr uint8_t  STATUS_OFF_BATTERY          = 10;
constexpr uint8_t  STATUS_OFF_CHARGER          = 11;
constexpr uint16_t STATUS_BODY_MIN_SIZE        = STATUS_OFF_CHARGER + 1;

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
