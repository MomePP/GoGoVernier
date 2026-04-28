# GoGoVernier

Vernier Go Direct (D2PIO) BLE client for the **GoGo Board 7** ESP32-C3
co-processor. Replaces `MomePP/GDXLib` + `MomePP/ArduinoBLE` with a clean
NimBLE-backed implementation that supports up to 32 channels per device and
multiple devices connected concurrently.

Status: **Phase 0 / skeleton** — public API only, no behaviour yet. See
`.claude/plans/gdxlib-rewrite.md` in the parent repo for the phase plan.

## Why a rewrite

- Old GDXLib caps at 7 channels per device; protocol allows 32. GDX-ACC
  already exposes 9.
- Old GDXLib is single-instance only (TU-static globals) — no two-sensor
  setups.
- Old ArduinoBLE host crashes inside `esp_bt_controller_init` /
  `r_lld_env_init` on arduino-esp32 ≥ 3.3.7 / IDF 5.5.4.
- Vernier's official Python lib (`VernierST/godirect-py`) ships the full
  opcode set + frame layout. The rewrite is a port, not reverse engineering.

## Wire protocol

See `.claude/specs/d2pio-protocol.md` (in parent repo) for the full opcode
table. UUIDs and constants are duplicated as compile-time values in
`src/D2PIOProtocol.h`.

## Layout

```
lib/GoGoVernier/
├── library.json          # PlatformIO manifest
├── LICENSE               # BSD-3-Clause (this fork) + Vernier portions
├── NOTICES.md            # Upstream attribution (godirect-py, GDXLib)
├── README.md             # This file
└── src/
    ├── D2PIOProtocol.h   # Opcodes, UUIDs, checksum
    ├── GoGoVernier.h     # Public API
    └── GoGoVernier.cpp   # Phase-0 stubs
```

## License

BSD 3-Clause. Portions derived from `VernierST/godirect-py` and
`Vernier-Science-Education/GDXLib`, both BSD-3, Copyright (c) 2024 Vernier
Science Education. See `LICENSE` and `NOTICES.md`.

Not endorsed by, affiliated with, or supported by Vernier.
