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
    // Per-tag log gating. With USE_ESP_IDF_LOG defined globally (see
    // platformio.ini debug env), arduino-esp32's log_*() macros route
    // through esp_log_write with tag = ARDUHAL_ESP_LOG_TAG ("ARDUINO").
    // Default tag level on arduino-esp32 is WARN, so log_i / log_d are
    // dropped unless we explicitly raise the ARDUINO tag here. Quiet
    // NimBLE-Arduino's component tags (their NIMBLE_LOG_* path uses
    // its own LOG_TAG strings — no ".cpp" suffix).
    esp_log_level_set("*",            ESP_LOG_WARN);
    esp_log_level_set("ARDUINO",      ESP_LOG_DEBUG);
    esp_log_level_set("NimBLEDevice", ESP_LOG_INFO);
    esp_log_level_set("NimBLEClient", ESP_LOG_INFO);
    esp_log_level_set("NimBLEScan",   ESP_LOG_INFO);

    g_ble_mutex = xSemaphoreCreateMutex();

    // setMTU must run AFTER init: ble_att_set_preferred_mtu only takes
    // effect once the NimBLE host stack is registered. Calling it before
    // init() silently fails and leaves the local preferred MTU at the
    // 23-byte default, which caps single ATT writes at 20 bytes — the
    // 25-byte CMD_INIT then gets truncated by NimBLERemoteValueAttribute::
    // writeValue's long-write fallback and the peripheral drops it.
    NimBLEDevice::init("GoGoVernier");
    bool mtuOk = NimBLEDevice::setMTU(247);
    log_i("BLE init done, setMTU(247) -> %s, getMTU()=%u",
          mtuOk ? "ok" : "FAIL",
          (unsigned)NimBLEDevice::getMTU());
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

    // exchangeMTU=true: let h2zero kick off the MTU exchange right after
    // BLE connect. Unlike the arduino-esp32 bundled BLE library (whose
    // BLEClient::connect bailed on BLE_HS_EALREADY when the Vernier
    // peripheral beat us to MTU REQUEST), h2zero's exchangeMTU() treats
    // EALREADY as success — the peer-initiated case is fine. Default MTU
    // 23 caps single ATT writes at 20 bytes; CMD_INIT is 25, so this
    // exchange is mandatory before any handshake frame goes out.
    if (!_impl->client->connect(_impl->target,
                                /*deleteAttributes=*/true,
                                /*asyncConnect=*/false,
                                /*exchangeMTU=*/true)) {
        log_e("NimBLEClient::connect failed addr=%s", _impl->peer_addr.c_str());
        delete _impl->target; _impl->target = nullptr;
        NimBLEDevice::deleteClient(_impl->client); _impl->client = nullptr;
        return false;
    }
    log_d("NimBLEClient connected");

    // h2zero's exchangeMTU is async — connect() returns before the GATT
    // exchange completes. Poll getMTU() until it climbs above the default
    // 23, or fall back to a manual exchange + poll if the auto-exchange
    // never landed. Without MTU >= 28, NimBLERemoteValueAttribute::
    // writeValue truncates the 25-byte CMD_INIT to 20 bytes and the
    // peripheral drops it.
    {
        const TickType_t kStep = pdMS_TO_TICKS(50);
        const TickType_t kMax  = pdMS_TO_TICKS(2000);
        TickType_t waited = 0;
        while (_impl->client->getMTU() <= 23 && waited < kMax) {
            vTaskDelay(kStep);
            waited += kStep;
        }
        uint16_t mtu = _impl->client->getMTU();
        if (mtu <= 23) {
            log_w("MTU still %u after %ums — issuing manual exchangeMTU",
                  (unsigned)mtu, (unsigned)pdTICKS_TO_MS(waited));
            _impl->client->exchangeMTU();
            // Poll again for up to another 1.5s.
            waited = 0;
            const TickType_t kMax2 = pdMS_TO_TICKS(1500);
            while (_impl->client->getMTU() <= 23 && waited < kMax2) {
                vTaskDelay(kStep);
                waited += kStep;
            }
            mtu = _impl->client->getMTU();
        }
        log_i("MTU = %u (settle %ums)", (unsigned)mtu,
              (unsigned)pdTICKS_TO_MS(waited));
        if (mtu < 28) {
            log_e("MTU %u too small for 25-byte CMD_INIT — handshake will fail",
                  (unsigned)mtu);
        }
    }

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
    log_i("char props: cmd[w=%d wnr=%d] rsp[notify=%d indicate=%d] MTU=%u",
          (int)_impl->cmd_char->canWrite(),
          (int)_impl->cmd_char->canWriteNoResponse(),
          (int)_impl->rsp_char->canNotify(),
          (int)_impl->rsp_char->canIndicate(),
          (unsigned)_impl->client->getMTU());

    _impl->connected = true;
    return true;
}

void NimBleXport::disconnect() {
    if (g_ble_mutex) xSemaphoreTake(g_ble_mutex, portMAX_DELAY);
    struct GiveOnExit {
        ~GiveOnExit() { if (g_ble_mutex) xSemaphoreGive(g_ble_mutex); }
    } _scope;

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
    // Pick the write mode based on what the peer actually advertises.
    // GDXLib uses ArduinoBLE's writeValue() default which auto-selects;
    // we replicate that logic explicitly. Prefer write-with-response
    // when the char supports it; h2zero's writeValue with response=false
    // truncates length > MTU-3 (long-write requires response).
    bool use_response = _impl->cmd_char->canWrite();
    bool ok = _impl->cmd_char->writeValue(data, len, use_response);
    if (!ok) {
        log_e("xport write FAILED len=%u resp=%d cmd=0x%02X",
              (unsigned)len, (int)use_response, (unsigned)data[4]);
    } else {
        log_d("xport write ok len=%u resp=%d cmd=0x%02X rcnt=0x%02X",
              (unsigned)len, (int)use_response,
              (unsigned)data[4], (unsigned)data[2]);
    }
    return ok;
}

bool NimBleXport::subscribe(NotifyCb cb) {
    if (!isConnected() || !_impl->rsp_char) {
        log_e("subscribe with no connection");
        return false;
    }
    _impl->on_notify = std::move(cb);

    // Per-instance routing via lambda capture, replacing the old file-
    // static `g_active_impl` + free-function trampoline. Two reasons:
    //   1. Multi-device (Phase 4): the global trampoline routed every
    //      connection's notifications to whichever Impl was bound last,
    //      silently stealing notifications between instances.
    //   2. Reduced UAF surface: the lambda's captured `Impl*` becomes
    //      unreachable when h2zero clears subscriptions inside
    //      NimBLEDevice::deleteClient (called from disconnect() before
    //      `delete _impl`), so a notify in flight at teardown can't
    //      land on a freed pointer the way the global could.
    // The captured `impl` indirects through `on_notify`, which
    // unsubscribe() nulls out — so even if h2zero dispatches a
    // late-queued notification before the subscription is fully torn
    // down, the lambda no-ops cleanly.
    Impl* impl = _impl;
    bool ok = _impl->rsp_char->subscribe(/*notifications=*/true,
        [impl](NimBLERemoteCharacteristic* /*chr*/,
               uint8_t* data, size_t len, bool /*isNotify*/) {
            if (impl && impl->on_notify) {
                impl->on_notify(data, static_cast<uint16_t>(len));
            }
        });
    if (!ok) {
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
