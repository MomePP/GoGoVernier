# GoGoVernier

Vernier Go Direct (D2PIO) BLE client for the **GoGo Board 7** ESP32-C3
co-processor. Replaces `MomePP/GDXLib` + `MomePP/ArduinoBLE` with a
NimBLE-backed implementation that supports up to 32 channels per device
and multiple devices over a single BLE controller.

**Status:** wired end-to-end against `vernier-firmware`'s 3-slot
multi-device UI. D2PIO handshake, live measurement decode, push-style
sample callback, battery + charger readout via `CMD_GET_STATUS`,
mutex-serialised lifecycle. NimBLE-Arduino transport.

## Capabilities

- **Full D2PIO handshake** — INIT → `CMD_GET_DEVICE_INFO` →
  `CMD_GET_SENSOR_AVAILABLE_MASK` → `CMD_GET_SENSOR_INFO` per enabled
  channel.
- **32-channel ceiling** — matches the protocol's u32 `available_mask`
  width. GDX-ACC's 9-channel layout works without recompilation.
- **Multi-device** — `vernier-firmware` runs a 3-slot pool against a
  single GoGoVernier instance per slot; transport routes notifications
  to the right session via per-instance lambda (no TU-static globals).
- **Push consumer** — `onSample(cb)` is invoked synchronously from the
  NimBLE notify task right after each `RESPONSE_MEASUREMENT` frame is
  decoded. Polling alternative (`sampleReady` / `copySample`) is also
  available; both feed the same notify path.
- **Battery + charger readout** — `refreshStatus()` issues
  `CMD_GET_STATUS` (opcode `0x10`, undocumented in the public spec but
  hardcoded in `godirect-py`) and parses `batteryLevelPercent` +
  `chargerState` from the wide CPU-version reply.
- **Dropped-sample counter** — monotonic, surfaced via
  `droppedSamples()` so the host can flag back-pressure.
- **Lifecycle mutex** — `session_mutex` serialises `open` / `close` /
  `start` / `stop` end-to-end so a host that races
  connect-then-disconnect can't tear into a half-set-up session.

## Public API (`gogo_vernier::GoGoVernier`)

```cpp
// Discovery + handshake
bool open(const char* name);   // "" = proximity, else exact saved name
void close();
bool isConnected() const;      // BLE link up
bool isReady() const;          // handshake done, available_mask populated
bool isScanning() const;
void abortScan();

// Channel mask
bool      enableSensor(uint8_t channel);  // 255 = device defaults
bool      disableSensor(uint8_t channel);
uint32_t  availableChannelMask() const;
uint32_t  enabledChannelMask() const;
uint8_t   channelCount() const;
const ChannelInfo* channel(uint8_t channel) const;

// Streaming
bool start(uint32_t period_ms);  // 0 = device's typical period
bool stop();
bool isStreaming() const;

// Sample retrieval (pick one)
bool   sampleReady() const;
bool   copySample(float* out, uint8_t& count);
float  measurement(uint8_t channel) const;
void   onSample(SampleCallback cb);   // pass {} to clear
uint32_t droppedSamples() const;

// Identity + status (cached at connect, refreshable)
const DeviceInfo&   deviceInfo() const;
const DeviceStatus& status() const;        // battery / charger / RSSI
bool                refreshStatus();
```

`ChannelInfo` mirrors `godirect-py`'s `Sensor` row: description, units,
sensor id, measurement type, min/max range, period bounds, mutex mask.

## Usage sketch

```cpp
#include "GoGoVernier.h"
using gogo_vernier::GoGoVernier;

GoGoVernier dev;
if (!dev.open("")) return;          // proximity scan
dev.enableSensor(255);              // device defaults
dev.onSample([](const auto& s) {
    // s.values[0..s.count-1] are floats in ascending channel-bit order
});
dev.start(0);                       // device's typical period
// ... later ...
dev.stop();
dev.close();
```

The `vernier-firmware` co-processor wraps each `GoGoVernier` instance
behind `VernierAdapter` and exposes it as a slot to the host MCU over
the framed-MsgPack UART protocol. See `vernier-firmware/src/main.cpp`
for the full integration.

## Threading model

- **Lifecycle calls** (`open`, `close`, `start`, `stop`) — caller's
  task. Internally serialised by `session_mutex`; safe to call from
  any context but blocks while another lifecycle op is in flight.
- **Sample callback** (`onSample` cb) — runs on the NimBLE notify
  task. Must not block; back-pressures all other notifications on the
  controller. Push to a queue and bail out fast.
- **Status readers** (`isConnected`, `isReady`, `isStreaming`,
  `deviceInfo`, `status`, `availableChannelMask`, etc.) — lock-free.
  Safe from any task; values are eventually-consistent across the
  notify hand-off.

## Layout

```
lib/GoGoVernier/
├── library.json          # PlatformIO manifest (NimBLE-Arduino ^2.0.0)
├── LICENSE               # BSD-3-Clause (this fork) + Vernier portions
├── NOTICES.md            # Upstream attribution (godirect-py, GDXLib)
├── README.md             # This file
└── src/
    ├── D2PIOProtocol.h   # Opcodes, UUIDs, checksum, MAX_CHANNELS, ChargerState
    ├── GoGoVernier.h     # Public API (Sample, ChannelInfo, DeviceInfo, DeviceStatus)
    ├── GoGoVernier.cpp   # Handshake, decode loop, status refresh
    └── transport/
        └── NimBleXport.* # Per-instance NimBLE wrapper, notify routing
```

## Build / dependencies

- Framework: Arduino. Platform: `espressif32` (pioarduino fork pinned
  by the consuming `vernier-firmware`).
- One declared dependency: `h2zero/NimBLE-Arduino ^2.0.0`.
- Consumed via PlatformIO `lib_deps` from a parent project; the
  `vernier-firmware` pin is currently `aee108c` on `main`.

## Why a rewrite

- Old GDXLib caps at 7 channels per device; protocol allows 32.
- Old GDXLib is single-instance only (TU-static globals) — no
  two-sensor setups. This rewrite holds session state per `GoGoVernier`
  instance and routes notifications via an `attHandle → lambda` map.
- Old ArduinoBLE host crashes inside `esp_bt_controller_init` /
  `r_lld_env_init` on arduino-esp32 ≥ 3.3.7 / IDF 5.5.4. NimBLE-Arduino
  is the actively-maintained alternative.
- Vernier's official Python lib (`VernierST/godirect-py`) ships the
  full opcode set + frame layout. The rewrite is a port, not reverse
  engineering.

## Wire protocol

See `.claude/specs/d2pio-protocol.md` (in the parent `vernier-firmware`
repo) for the full opcode table. UUIDs and constants are duplicated as
compile-time values in `src/D2PIOProtocol.h`.

`CMD_GET_STATUS` (opcode `0x10`) is not in Vernier's published spec —
hardcoded in `godirect-py` and ported here. See
`.claude/knowledges/vernier-mcu-internals.md` (parent repo) for the
response byte layout used by `refreshStatus()`.

## License

BSD 3-Clause. Portions derived from `VernierST/godirect-py` and
`Vernier-Science-Education/GDXLib`, both BSD-3, Copyright (c) 2024
Vernier Science Education. See `LICENSE` and `NOTICES.md`.

Not endorsed by, affiliated with, or supported by Vernier.
