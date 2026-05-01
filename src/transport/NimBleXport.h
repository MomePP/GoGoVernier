// NimBleXport — BleTransport implementation backed by h2zero/NimBLE-Arduino.
//
// On arduino-esp32 3.3.x the NimBLE-Arduino library compiles against the
// bundled NimBLE host (no vendored copy → no version skew with the BT
// controller blob), and unlike the arduino-esp32-bundled `BLE` library it
// doesn't force an MTU exchange on connect that races with peripherals
// which initiate it themselves (Vernier GDX devices do).

#pragma once

#include "BleTransport.h"

class NimBLEAdvertisedDevice;
class NimBLEClient;
class NimBLERemoteCharacteristic;

namespace gogo_vernier {

class NimBleXport : public BleTransport {
public:
    NimBleXport();
    ~NimBleXport() override;

    bool connect(const char* name, uint32_t scan_timeout_ms) override;
    void disconnect() override;
    bool isConnected() const override;
    int  rssi() const override;
    const char* peerName() const override;
    const char* peerAddress() const override;

    bool write(const uint8_t* data, uint16_t len) override;
    bool subscribe(NotifyCb cb) override;
    void unsubscribe() override;

    // Public so the .cpp's ClientCallbacks helper class can touch the
    // connected flag from onDisconnect(). Treat it as a private detail
    // of the translation unit. (The pre-Phase-4 file-static notify
    // trampoline that also needed this is gone — notifications now
    // route through a per-instance lambda capturing Impl* directly.)
    struct Impl;

private:
    Impl* _impl;
};

}  // namespace gogo_vernier
