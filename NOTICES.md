# Third-party notices

`GoGoVernier` is a clean-room rewrite for the GoGo Board 7 platform of the
Vernier Go Direct Bluetooth protocol (D2PIO). It is **derived from** two
upstream projects, both BSD-3-Clause licensed and copyright Vernier Science
Education. Per BSD-3 §1, the upstream copyright notices and disclaimer are
preserved here.

## Upstream: `VernierST/godirect-py`

- Repository: <https://github.com/VernierST/godirect-py>
- Copyright (c) 2024, Vernier Science Education
- License: BSD 3-Clause (see <https://github.com/VernierST/godirect-py/blob/main/LICENSE>)

Used as the authoritative reference for D2PIO command opcodes
(`CMD_ID_*`), measurement-frame TLV tags (`MEASUREMENT_TYPE_*`), the
sensor-info record layout, the rolling-counter behaviour, and the
read/reassembly state machine. Field names and constant values in this
library track that source.

## Upstream: `Vernier-Science-Education/GDXLib`

- Repository: <https://github.com/Vernier-Science-Education/GDXLib>
- Fork in use during bring-up: <https://github.com/MomePP/GDXLib>
- Copyright (c) 2024, Vernier Science Education
- License: BSD 3-Clause (see <https://github.com/Vernier-Science-Education/GDXLib/blob/main/LICENSE>)

Used as the reference for the Arduino-flavoured BLE characteristic discovery
sequence, observed MTU and chunking behaviour on ESP32, the 1's-complement
checksum routine, and the GDX service / command / response UUIDs.

## What is *not* derived

- The NimBLE-Arduino backend transport. Written against
  [`h2zero/NimBLE-Arduino`](https://github.com/h2zero/NimBLE-Arduino),
  Apache-2.0.
- The 32-channel instance-state architecture, multi-device session model,
  callback-based sample delivery, and host-side multi-slot UART protocol.
  Originally written for this project.
- The conformance test harness using godirect-py as a device-side fake
  GATT server. The harness is original; the bytes it emits are godirect-py's.

## Trademarks

"Vernier", "Go Direct" and the GDX device names (e.g. GDX-LC, GDX-ACC) are
trademarks of Vernier Science Education. This library names them only for
the purpose of describing interoperability with their devices, per BSD-3 §3
which prohibits using the copyright holder's name to endorse this work.
This library is not endorsed by, affiliated with, or supported by Vernier.
