// BundledBleXport — BleTransport implementation backed by the BLE library
// shipped inside arduino-esp32 (libraries/BLE). On 3.3.x that library is
// itself a NimBLE wrapper, which dodges the version-skew failure mode we
// hit with MomePP/ArduinoBLE.

#pragma once

#include "BleTransport.h"

class BLEAdvertisedDevice;
class BLEClient;
class BLERemoteCharacteristic;

namespace gogo_vernier {

class BundledBleXport : public BleTransport {
public:
    BundledBleXport();
    ~BundledBleXport() override;

    bool connect(const char* name, uint32_t scan_timeout_ms) override;
    void disconnect() override;
    bool isConnected() const override;
    int  rssi() const override;
    const char* peerName() const override;
    const char* peerAddress() const override;

    bool write(const uint8_t* data, uint16_t len) override;
    bool subscribe(NotifyCb cb) override;
    void unsubscribe() override;

    // Public so the .cpp's static notify trampoline + scan callbacks can
    // touch it. Treat it as a private detail of the translation unit.
    struct Impl;

private:
    Impl* _impl;
};

}  // namespace gogo_vernier
