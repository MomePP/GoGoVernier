// BundledBleXport — implementation against arduino-esp32's bundled BLE library.
//
// API reference:
//   https://github.com/espressif/arduino-esp32/tree/master/libraries/BLE
//
// Connection model (one connection per BundledBleXport instance):
//   1. BLEDevice::init() — process-wide, idempotent.
//   2. BLEScan finds the target device. We use an advertised-device callback
//      that aborts the scan once we have a match, so we don't pay the full
//      timeout when the device shows up early.
//   3. BLEClient::connect(BLEAdvertisedDevice*) — opens the link.
//   4. getService(kGdxServiceUuid) → getCharacteristic(cmd/response).
//   5. response->registerForNotify() forwards bytes into the user callback.

#include "BundledBleXport.h"

#include <Arduino.h>
#include <BLEAddress.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLEDevice.h>
#include <BLERemoteCharacteristic.h>
#include <BLERemoteService.h>
#include <BLEScan.h>
#include <BLEUUID.h>
#include <esp_log.h>

#include <string>

#include "../D2PIOProtocol.h"

namespace gogo_vernier {

namespace {

// Process-wide BLEDevice init. Calling BLEDevice::init() more than once on
// arduino-esp32 is safe but logs a warning — guard it ourselves.
bool g_ble_inited = false;
void ensureBleInited() {
    if (g_ble_inited) return;
    // Quiet the bundled BLE library's debug logs. With CORE_DEBUG_LEVEL=4
    // every advertisement byte is dumped at log_d level, which on ESP32-C3
    // USB-CDC overruns the host TX buffer and we lose unrelated logs to
    // truncation. Override per-tag so the rest of the firmware keeps its
    // verbose logging.
    esp_log_level_set("BLEDevice",            ESP_LOG_INFO);
    esp_log_level_set("BLEScan",              ESP_LOG_INFO);
    esp_log_level_set("BLEAdvertisedDevice",  ESP_LOG_WARN);
    esp_log_level_set("BLEClient",            ESP_LOG_INFO);
    esp_log_level_set("BLERemoteCharacteristic", ESP_LOG_INFO);
    esp_log_level_set("BLERemoteService",     ESP_LOG_INFO);
    esp_log_level_set("NimBLE",               ESP_LOG_WARN);
    BLEDevice::init("GoGoVernier");
    g_ble_inited = true;
}

}  // namespace

struct BundledBleXport::Impl {
    BLEClient*               client    = nullptr;
    BLEAdvertisedDevice*     target    = nullptr;        // owned, freed on disconnect
    BLERemoteCharacteristic* cmd_char  = nullptr;
    BLERemoteCharacteristic* rsp_char  = nullptr;
    NotifyCb                 on_notify;
    std::string              peer_name;
    std::string              peer_addr;
    int                      cached_rssi = 0;
    bool                     connected   = false;
};

// Free function shim so BLERemoteCharacteristic::registerForNotify can call
// us. The xport pointer is captured via a small lookup — we only ever have
// one notify subscription active per Impl, so we stash a back-pointer in
// BLEClient's app data slot when available; arduino-esp32 doesn't expose
// that cleanly, so we use a single static for now (single-conn assumption,
// matches Phase 1 — multi-conn lands in Phase 4).
static BundledBleXport::Impl* g_active_impl = nullptr;
static void notifyTrampoline(BLERemoteCharacteristic* /*chr*/,
                             uint8_t* data, size_t len, bool /*isNotify*/) {
    if (g_active_impl && g_active_impl->on_notify) {
        g_active_impl->on_notify(data, static_cast<uint16_t>(len));
    }
}

// Scan callback. Aborts the scan as soon as we see a matching device so we
// don't pay full scan_timeout when the target is already advertising.
class TargetFinder : public BLEAdvertisedDeviceCallbacks {
public:
    TargetFinder(const char* wanted_name, BLEUUID service_uuid)
        : _wanted(wanted_name ? wanted_name : ""),
          _proximity(_wanted.empty() || _wanted == "proximity"),
          _service(service_uuid) {}

    void onResult(BLEAdvertisedDevice advertised) override {
        // Named match takes precedence and bypasses the service-UUID gate:
        // Vernier devices put the local name in the primary advertisement
        // packet but only echo the GDX service UUID in the scan response.
        // Active-scan reassembly is best-effort, so a saved-name reconnect
        // must work even when isAdvertisingService() returns false.
        if (!_proximity) {
            if (advertised.haveName() &&
                strcmp(advertised.getName().c_str(), _wanted.c_str()) == 0) {
                if (_best) delete _best;
                _best = new BLEAdvertisedDevice(advertised);
                advertised.getScan()->stop();
            }
            return;
        }

        // Proximity mode: filter by service UUID, then track best RSSI.
        if (!advertised.haveServiceUUID() ||
            !advertised.isAdvertisingService(_service)) {
            return;
        }
        int rssi = advertised.getRSSI();
        if (rssi < kProximityRssiFloor) return;
        if (!_best || rssi > _best->getRSSI()) {
            if (_best) delete _best;
            _best = new BLEAdvertisedDevice(advertised);
        }
        // Keep scanning until timeout to find the strongest signal.
    }

    // Caller takes ownership.
    BLEAdvertisedDevice* take() { auto* p = _best; _best = nullptr; return p; }

private:
    std::string _wanted;
    bool _proximity;
    BLEUUID _service;
    BLEAdvertisedDevice* _best = nullptr;
};

class ClientCallbacks : public BLEClientCallbacks {
public:
    explicit ClientCallbacks(BundledBleXport::Impl* impl) : _impl(impl) {}
    void onConnect(BLEClient* /*c*/) override {}
    void onDisconnect(BLEClient* /*c*/) override {
        if (_impl) _impl->connected = false;
    }
private:
    BundledBleXport::Impl* _impl;
};

BundledBleXport::BundledBleXport() : _impl(new Impl()) {}

BundledBleXport::~BundledBleXport() {
    disconnect();
    delete _impl;
}

bool BundledBleXport::connect(const char* name, uint32_t scan_timeout_ms) {
    ensureBleInited();

    BLEScan* scan = BLEDevice::getScan();
    BLEUUID  svc(kGdxServiceUuid);
    TargetFinder finder(name, svc);

    scan->setAdvertisedDeviceCallbacks(&finder, /*wantDuplicates=*/false);
    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(99);

    uint32_t timeout_s = scan_timeout_ms ? (scan_timeout_ms + 999) / 1000 : 5;
    log_i("scan start name=\"%s\" timeout=%us", name ? name : "", timeout_s);
    scan->start(timeout_s, /*is_continue=*/false);
    scan->stop();
    scan->clearResults();

    _impl->target = finder.take();
    if (!_impl->target) {
        log_e("scan complete, no match");
        return false;
    }

    _impl->peer_addr = _impl->target->getAddress().toString().c_str();
    _impl->peer_name = _impl->target->haveName()
                           ? _impl->target->getName().c_str()
                           : "";
    _impl->cached_rssi = _impl->target->getRSSI();
    log_i("found peer name=\"%s\" addr=%s rssi=%d",
          _impl->peer_name.c_str(), _impl->peer_addr.c_str(),
          _impl->cached_rssi);

    _impl->client = BLEDevice::createClient();
    _impl->client->setClientCallbacks(new ClientCallbacks(_impl));

    if (!_impl->client->connect(_impl->target)) {
        log_e("BLEClient::connect failed addr=%s", _impl->peer_addr.c_str());
        delete _impl->target; _impl->target = nullptr;
        delete _impl->client; _impl->client = nullptr;
        return false;
    }
    log_d("BLEClient connected");

    BLERemoteService* service = _impl->client->getService(svc);
    if (!service) {
        log_e("GDX service %s not found", kGdxServiceUuid);
        _impl->client->disconnect();
        return false;
    }
    log_d("GDX service discovered");

    _impl->cmd_char = service->getCharacteristic(BLEUUID(kGdxCommandCharUuid));
    _impl->rsp_char = service->getCharacteristic(BLEUUID(kGdxResponseCharUuid));
    if (!_impl->cmd_char || !_impl->rsp_char) {
        log_e("missing characteristic cmd=%p rsp=%p",
              (void*)_impl->cmd_char, (void*)_impl->rsp_char);
        _impl->client->disconnect();
        return false;
    }
    log_d("chars discovered cmd=%s rsp=%s",
          kGdxCommandCharUuid, kGdxResponseCharUuid);

    _impl->connected = true;
    g_active_impl = _impl;
    return true;
}

void BundledBleXport::disconnect() {
    if (g_active_impl == _impl) g_active_impl = nullptr;
    if (_impl->client) {
        if (_impl->client->isConnected()) {
            log_i("disconnect addr=%s", _impl->peer_addr.c_str());
            _impl->client->disconnect();
        }
        delete _impl->client;
        _impl->client = nullptr;
    }
    if (_impl->target) {
        delete _impl->target;
        _impl->target = nullptr;
    }
    _impl->cmd_char = nullptr;
    _impl->rsp_char = nullptr;
    _impl->connected = false;
    _impl->on_notify = nullptr;
}

bool BundledBleXport::isConnected() const {
    return _impl->connected && _impl->client && _impl->client->isConnected();
}

int BundledBleXport::rssi() const {
    if (_impl->client && _impl->client->isConnected()) {
        // arduino-esp32 BLEClient doesn't currently expose live RSSI on
        // every release; fall back to the cached scan-time value.
        return _impl->cached_rssi;
    }
    return _impl->cached_rssi;
}

const char* BundledBleXport::peerName() const    { return _impl->peer_name.c_str(); }
const char* BundledBleXport::peerAddress() const { return _impl->peer_addr.c_str(); }

bool BundledBleXport::write(const uint8_t* data, uint16_t len) {
    if (!isConnected() || !_impl->cmd_char) return false;
    _impl->cmd_char->writeValue(const_cast<uint8_t*>(data), len, /*response=*/false);
    return true;
}

bool BundledBleXport::subscribe(NotifyCb cb) {
    if (!isConnected() || !_impl->rsp_char) {
        log_e("subscribe with no connection");
        return false;
    }
    _impl->on_notify = std::move(cb);
    g_active_impl = _impl;
    _impl->rsp_char->registerForNotify(notifyTrampoline);
    log_d("subscribed to response char");
    return true;
}

void BundledBleXport::unsubscribe() {
    if (_impl->rsp_char) _impl->rsp_char->registerForNotify(nullptr);
    _impl->on_notify = nullptr;
}

}  // namespace gogo_vernier
