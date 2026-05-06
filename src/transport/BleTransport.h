// BleTransport — abstract seam between the D2PIO protocol layer and whatever
// BLE stack we're sitting on. Lets us swap the bundled arduino-esp32 BLE
// library for a host-side fake transport in tests without touching the
// session / encoder / decoder code.

#pragma once

#include <stdint.h>
#include <functional>

namespace gogo_vernier {

class BleTransport {
public:
    // Called from the BLE stack's notify callback on the response
    // characteristic. `data` is the raw notify payload exactly as it left
    // the device — assembly into D2PIO frames is the session's job.
    using NotifyCb = std::function<void(const uint8_t* data, uint16_t len)>;

    virtual ~BleTransport() = default;

    // Scan + connect. `name` semantics:
    //   - empty / "proximity"  → highest-RSSI device advertising the GDX
    //                            service UUID, RSSI > PROXIMITY_RSSI_FLOOR
    //   - "GDX-XXX 0123ABCD"   → exact advertised local-name match
    // `scan_timeout_ms` bounds the scan; 0 = library default.
    virtual bool connect(const char* name, uint32_t scan_timeout_ms) = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() const = 0;
    virtual int  rssi() const = 0;
    virtual const char* peerName() const = 0;
    virtual const char* peerAddress() const = 0;

    // Synchronous write to the GDX command characteristic.
    virtual bool write(const uint8_t* data, uint16_t len) = 0;

    // Subscribe / unsubscribe to the GDX response characteristic.
    virtual bool subscribe(NotifyCb cb) = 0;
    virtual void unsubscribe() = 0;
};

// "Proximity" mode RSSI floor (dBm). Devices below this are ignored even if
// they're the closest match — keeps stray neighbouring devices out.
constexpr int8_t PROXIMITY_RSSI_FLOOR = -75;

}  // namespace gogo_vernier
