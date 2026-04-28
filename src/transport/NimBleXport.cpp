// NimBleXport — h2zero/NimBLE-Arduino backend.
//
// API reference:
//   https://h2zero.github.io/esp-nimble-cpp/
//
// Why this transport over the arduino-esp32 bundled BLE library:
//   The bundled BLEClient::connect unconditionally calls
//   ble_gattc_exchange_mtu after the link comes up. Vernier GDX peripherals
//   send their own MTU REQUEST as soon as the link is up; the central's
//   second exchange returns BLE_HS_EALREADY and BLEClient bails despite
//   the controller-level link being healthy (sensor LED red→green).
//   NimBLE-Arduino exposes exchangeMTU as an opt-in flag — passing
//   exchangeMTU=false to connect() avoids the race entirely.
//
// On arduino-esp32 3.3.x NimBLE-Arduino compiles against the bundled
// NimBLE host (no vendored stack) — same blob version as the BT
// controller, no version-skew failure mode.
//
// Connection model (one connection per NimBleXport instance for now —
// multi-conn lands in Phase 4):
//   1. NimBLEDevice::init() — process-wide, idempotent.
//   2. NimBLEScan finds the target device.
//   3. NimBLEClient::connect(device, deleteAttrs=true, async=false,
//                            exchangeMTU=false).
//   4. getService(kGdxServiceUuid) → getCharacteristic(cmd/response).
//   5. response->subscribe(true, cb) wires notifications.

#include "NimBleXport.h"

#include <NimBLEDevice.h>
#include <NimBLEAddress.h>
#include <NimBLEAdvertisedDevice.h>
#include <NimBLEClient.h>
#include <NimBLERemoteCharacteristic.h>
#include <NimBLERemoteService.h>
#include <NimBLEScan.h>
#include <NimBLEUUID.h>

#include <Arduino.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <mutex>
#include <string>

#include "../D2PIOProtocol.h"

namespace gogo_vernier {

namespace {

// One-time init guarded by std::call_once so two tasks racing into
// connect() can't both call NimBLEDevice::init().
std::once_flag g_ble_init_flag;

// Serialises the entire connect/disconnect path. NimBLEScan and
// NimBLEDevice::createClient are non-reentrant for our usage pattern.
SemaphoreHandle_t g_ble_mutex = nullptr;

void initBleOnce() {
    // Quiet NimBLE-Arduino's per-event log spam. Tags include the ".cpp"
    // suffix because arduino-esp32's log_* macros derive the tag from
    // pathToFileName(__FILE__). USE_ESP_IDF_LOG must be defined globally
    // (see platformio.ini debug env) for these settings to take effect.
    esp_log_level_set("NimBLEDevice.cpp",                ESP_LOG_INFO);
    esp_log_level_set("NimBLEScan.cpp",                  ESP_LOG_INFO);
    esp_log_level_set("NimBLEAdvertisedDevice.cpp",      ESP_LOG_WARN);
    esp_log_level_set("NimBLEClient.cpp",                ESP_LOG_INFO);
    esp_log_level_set("NimBLERemoteCharacteristic.cpp",  ESP_LOG_INFO);
    esp_log_level_set("NimBLERemoteService.cpp",         ESP_LOG_INFO);
    esp_log_level_set("NimBLEUtils.cpp",                 ESP_LOG_WARN);
    esp_log_level_set("NimBLEAddress.cpp",               ESP_LOG_WARN);
    esp_log_level_set("NimBLEUUID.cpp",                  ESP_LOG_WARN);
    esp_log_level_set("NimBLE",                          ESP_LOG_WARN);

    g_ble_mutex = xSemaphoreCreateMutex();
    NimBLEDevice::init("GoGoVernier");
}

void ensureBleInited() {
    std::call_once(g_ble_init_flag, initBleOnce);
}

}  // namespace

struct NimBleXport::Impl {
    NimBLEClient*               client      = nullptr;
    NimBLEAdvertisedDevice*     target      = nullptr;        // owned
    NimBLERemoteCharacteristic* cmd_char    = nullptr;
    NimBLERemoteCharacteristic* rsp_char    = nullptr;
    NotifyCb                    on_notify;
    std::string                 peer_name;
    std::string                 peer_addr;
    int                         cached_rssi = 0;
    bool                        connected   = false;
};

// Single-conn assumption (Phase 1). Multi-conn (Phase 4) replaces this with
// a conn_handle → Impl* map.
static NimBleXport::Impl* g_active_impl = nullptr;
static void notifyTrampoline(NimBLERemoteCharacteristic* /*chr*/,
                             uint8_t* data, size_t len, bool /*isNotify*/) {
    if (g_active_impl && g_active_impl->on_notify) {
        g_active_impl->on_notify(data, static_cast<uint16_t>(len));
    }
}

class TargetFinder : public NimBLEScanCallbacks {
public:
    TargetFinder(const char* wanted_name, NimBLEUUID service_uuid)
        : _wanted(wanted_name ? wanted_name : ""),
          _proximity(_wanted.empty() || _wanted == "proximity"),
          _service(service_uuid) {}

    void onResult(const NimBLEAdvertisedDevice* advertised) override {
        // Named match — Vernier puts the local name in the primary adv
        // packet, service UUID only in scan response. Match name even
        // when isAdvertisingService() is false.
        if (!_proximity) {
            if (advertised->haveName() &&
                advertised->getName() == _wanted) {
                if (_best) delete _best;
                _best = new NimBLEAdvertisedDevice(*advertised);
                NimBLEDevice::getScan()->stop();
            }
            return;
        }

        // Proximity: service UUID OR "GDX-" name prefix qualifies.
        bool match = false;
        if (advertised->haveServiceUUID() &&
            advertised->isAdvertisingService(_service)) {
            match = true;
        } else if (advertised->haveName()) {
            const std::string& n = advertised->getName();
            if (n.compare(0, 4, "GDX-") == 0) {
                match = true;
            }
        }
        if (!match) return;

        int rssi = advertised->getRSSI();
        if (rssi < kProximityRssiFloor) return;
        if (!_best || rssi > _best->getRSSI()) {
            if (_best) delete _best;
            _best = new NimBLEAdvertisedDevice(*advertised);
        }
        // Keep scanning until timeout to find the strongest signal.
    }

    NimBLEAdvertisedDevice* take() { auto* p = _best; _best = nullptr; return p; }

private:
    std::string _wanted;
    bool _proximity;
    NimBLEUUID _service;
    NimBLEAdvertisedDevice* _best = nullptr;
};

class ClientCallbacks : public NimBLEClientCallbacks {
public:
    explicit ClientCallbacks(NimBleXport::Impl* impl) : _impl(impl) {}
    void onConnect(NimBLEClient* /*c*/) override {}
    void onDisconnect(NimBLEClient* /*c*/, int /*reason*/) override {
        if (_impl) _impl->connected = false;
    }
private:
    NimBleXport::Impl* _impl;
};

NimBleXport::NimBleXport() : _impl(new Impl()) {}

NimBleXport::~NimBleXport() {
    disconnect();
    delete _impl;
}

bool NimBleXport::connect(const char* name, uint32_t scan_timeout_ms) {
    ensureBleInited();

    xSemaphoreTake(g_ble_mutex, portMAX_DELAY);
    struct GiveOnExit {
        ~GiveOnExit() { xSemaphoreGive(g_ble_mutex); }
    } _scope;

    NimBLEScan* scan = NimBLEDevice::getScan();
    if (!scan) {
        log_e("NimBLEDevice::getScan() returned NULL");
        return false;
    }
    NimBLEUUID svc(kGdxServiceUuid);
    TargetFinder finder(name, svc);

    scan->setScanCallbacks(&finder, /*wantDuplicates=*/false);
    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(99);

    uint32_t timeout_ms = scan_timeout_ms ? scan_timeout_ms : 5000;
    log_i("scan start name=\"%s\" timeout=%ums", name ? name : "", timeout_ms);
    scan->getResults(timeout_ms, /*is_continue=*/false);
    scan->stop();
    scan->clearResults();

    _impl->target = finder.take();
    if (!_impl->target) {
        log_e("scan complete, no match");
        return false;
    }

    _impl->peer_addr = _impl->target->getAddress().toString();
    _impl->peer_name = _impl->target->haveName() ? _impl->target->getName() : "";
    _impl->cached_rssi = _impl->target->getRSSI();
    log_i("found peer name=\"%s\" addr=%s rssi=%d",
          _impl->peer_name.c_str(), _impl->peer_addr.c_str(),
          _impl->cached_rssi);

    // Settle delay — NimBLE host clears scan-internal flags a few HCI
    // round-trips after stop() returns.
    vTaskDelay(pdMS_TO_TICKS(200));

    _impl->client = NimBLEDevice::createClient();
    _impl->client->setClientCallbacks(new ClientCallbacks(_impl), /*deleteCallbacks=*/true);

    if (!_impl->client->connect(_impl->target,
                                /*deleteAttributes=*/true,
                                /*asyncConnect=*/false,
                                /*exchangeMTU=*/false)) {
        log_e("NimBLEClient::connect failed addr=%s", _impl->peer_addr.c_str());
        delete _impl->target; _impl->target = nullptr;
        NimBLEDevice::deleteClient(_impl->client); _impl->client = nullptr;
        return false;
    }
    log_d("NimBLEClient connected");

    NimBLERemoteService* service = _impl->client->getService(svc);
    if (!service) {
        log_e("GDX service %s not found", kGdxServiceUuid);
        _impl->client->disconnect();
        return false;
    }
    log_d("GDX service discovered");

    _impl->cmd_char = service->getCharacteristic(NimBLEUUID(kGdxCommandCharUuid));
    _impl->rsp_char = service->getCharacteristic(NimBLEUUID(kGdxResponseCharUuid));
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

void NimBleXport::disconnect() {
    if (g_ble_mutex) xSemaphoreTake(g_ble_mutex, portMAX_DELAY);
    struct GiveOnExit {
        ~GiveOnExit() { if (g_ble_mutex) xSemaphoreGive(g_ble_mutex); }
    } _scope;

    if (g_active_impl == _impl) g_active_impl = nullptr;
    if (_impl->client) {
        if (_impl->client->isConnected()) {
            log_i("disconnect addr=%s", _impl->peer_addr.c_str());
            _impl->client->disconnect();
            const TickType_t kStep = pdMS_TO_TICKS(20);
            const TickType_t kMax  = pdMS_TO_TICKS(500);
            TickType_t waited = 0;
            while (_impl->client->isConnected() && waited < kMax) {
                vTaskDelay(kStep);
                waited += kStep;
            }
        }
        NimBLEDevice::deleteClient(_impl->client);
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

bool NimBleXport::isConnected() const {
    return _impl->connected && _impl->client && _impl->client->isConnected();
}

int NimBleXport::rssi() const {
    if (_impl->client && _impl->client->isConnected()) {
        int live = _impl->client->getRssi();
        if (live != 0) return live;
    }
    return _impl->cached_rssi;
}

const char* NimBleXport::peerName() const    { return _impl->peer_name.c_str(); }
const char* NimBleXport::peerAddress() const { return _impl->peer_addr.c_str(); }

bool NimBleXport::write(const uint8_t* data, uint16_t len) {
    if (!isConnected() || !_impl->cmd_char) return false;
    return _impl->cmd_char->writeValue(data, len, /*response=*/false);
}

bool NimBleXport::subscribe(NotifyCb cb) {
    if (!isConnected() || !_impl->rsp_char) {
        log_e("subscribe with no connection");
        return false;
    }
    _impl->on_notify = std::move(cb);
    g_active_impl = _impl;
    if (!_impl->rsp_char->subscribe(/*notifications=*/true, notifyTrampoline)) {
        log_e("subscribe failed");
        return false;
    }
    log_d("subscribed to response char");
    return true;
}

void NimBleXport::unsubscribe() {
    if (_impl->rsp_char) _impl->rsp_char->unsubscribe();
    _impl->on_notify = nullptr;
}

}  // namespace gogo_vernier
