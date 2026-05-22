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
// Connection model — one BLE connection per NimBleXport instance,
// multiple instances coexist on a single controller (notifications
// route per-session via lambda capture, see subscribe() below):
//   1. NimBLEDevice::init() — process-wide, idempotent.
//   2. NimBLEScan finds the target device.
//   3. NimBLEClient::connect(device, deleteAttrs=true, async=false,
//                            exchangeMTU=false).
//   4. getService(GDX_SERVICE_UUID) → getCharacteristic(cmd/response).
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

// ---- BLE / GATT tunables --------------------------------------------------
//
// All timing knobs live here so a future maintainer can tune them in one
// place. Values were chosen empirically during the Phase-1/2 GDX-LC
// bring-up; see .claude/knowledges/d2pio-debug-findings.md for the
// reasoning behind each.

// MTU. Default ATT MTU is 23 bytes → 20-byte single-write payload, too
// small for the 25-byte CMD_INIT frame. We request 247 (NimBLE max for
// the bundled host); CMD_INIT only needs ≥ 28 to fit in one ATT write.
constexpr uint16_t PREFERRED_MTU       = 247;
constexpr uint16_t DEFAULT_ATT_MTU      =  23;
constexpr uint16_t MIN_USABLE_MTU       =  28;

// MTU exchange settle. h2zero's exchangeMTU is async — connect() returns
// before the GATT exchange completes. Poll getMTU() in MTU_POLL_STEP_MS
// increments up to MTU_FIRST_WAIT_MS; if still at default, fire a manual
// exchangeMTU and poll up to MTU_RETRY_WAIT_MS more.
constexpr uint32_t MTU_POLL_STEP_MS      =   50;
constexpr uint32_t MTU_FIRST_WAIT_MS     = 2000;
constexpr uint32_t MTU_RETRY_WAIT_MS     = 1500;

// Scan defaults. setInterval / setWindow are in 0.625 ms units per the
// BLE spec; 100 / 99 means "scan ~62 ms out of every ~62.5 ms" — near
// 100 % duty cycle, picks up advertisers fast.
constexpr uint16_t SCAN_INTERVAL_UNITS  =  100;
constexpr uint16_t SCAN_WINDOW_UNITS    =   99;
constexpr uint32_t DEFAULT_SCAN_MS      = 5000;
// Settle delay between scan->stop() and createClient — NimBLE host
// clears scan-internal flags a few HCI round-trips after stop returns.
constexpr uint32_t POST_SCAN_SETTLE_MS   =  200;

// Disconnect wait. NimBLEClient::disconnect is async; poll
// isConnected() in DISCO_POLL_STEP_MS increments up to DISCO_MAX_WAIT_MS.
constexpr uint32_t DISCO_POLL_STEP_MS    =   20;
constexpr uint32_t DISCO_MAX_WAIT_MS     =  500;

// ---------------------------------------------------------------------------

// One-time init guarded by std::call_once so two tasks racing into
// connect() can't both call NimBLEDevice::init().
std::once_flag g_ble_init_flag;

// Serialises the entire connect/disconnect path. NimBLEScan and
// NimBLEDevice::createClient are non-reentrant for our usage pattern.
// Recursive so disconnect() can call into unsubscribe() (which now
// takes the mutex itself for the standalone-unsubscribe case) without
// deadlocking — see disconnect()'s teardown ordering.
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

    g_ble_mutex = xSemaphoreCreateRecursiveMutex();

    // setMTU must run AFTER init: ble_att_set_preferred_mtu only takes
    // effect once the NimBLE host stack is registered. Calling it before
    // init() silently fails and leaves the local preferred MTU at the
    // 23-byte default, which caps single ATT writes at 20 bytes — the
    // 25-byte CMD_INIT then gets truncated by NimBLERemoteValueAttribute::
    // writeValue's long-write fallback and the peripheral drops it.
    NimBLEDevice::init("GoGoVernier");
    bool mtuOk = NimBLEDevice::setMTU(PREFERRED_MTU);
    log_i("BLE init done, setMTU(%u) -> %s, getMTU()=%u",
          (unsigned)PREFERRED_MTU,
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
        if (rssi < PROXIMITY_RSSI_FLOOR) return;
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

    xSemaphoreTakeRecursive(g_ble_mutex, portMAX_DELAY);
    struct GiveOnExit {
        ~GiveOnExit() { xSemaphoreGiveRecursive(g_ble_mutex); }
    } _scope;

    NimBLEScan* scan = NimBLEDevice::getScan();
    if (!scan) {
        log_e("NimBLEDevice::getScan() returned NULL");
        return false;
    }
    NimBLEUUID svc(GDX_SERVICE_UUID);
    TargetFinder finder(name, svc);

    scan->setScanCallbacks(&finder, /*wantDuplicates=*/false);
    scan->setActiveScan(true);
    scan->setInterval(SCAN_INTERVAL_UNITS);
    scan->setWindow(SCAN_WINDOW_UNITS);

    uint32_t timeout_ms = scan_timeout_ms ? scan_timeout_ms : DEFAULT_SCAN_MS;
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
    vTaskDelay(pdMS_TO_TICKS(POST_SCAN_SETTLE_MS));

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
        const TickType_t kStep = pdMS_TO_TICKS(MTU_POLL_STEP_MS);
        const TickType_t kMax  = pdMS_TO_TICKS(MTU_FIRST_WAIT_MS);
        TickType_t waited = 0;
        while (_impl->client->getMTU() <= DEFAULT_ATT_MTU && waited < kMax) {
            vTaskDelay(kStep);
            waited += kStep;
        }
        uint16_t mtu = _impl->client->getMTU();
        if (mtu <= DEFAULT_ATT_MTU) {
            log_w("MTU still %u after %ums — issuing manual exchangeMTU",
                  (unsigned)mtu, (unsigned)pdTICKS_TO_MS(waited));
            _impl->client->exchangeMTU();
            waited = 0;
            const TickType_t kMax2 = pdMS_TO_TICKS(MTU_RETRY_WAIT_MS);
            while (_impl->client->getMTU() <= DEFAULT_ATT_MTU && waited < kMax2) {
                vTaskDelay(kStep);
                waited += kStep;
            }
            mtu = _impl->client->getMTU();
        }
        log_i("MTU = %u (settle %ums)", (unsigned)mtu,
              (unsigned)pdTICKS_TO_MS(waited));
        if (mtu < MIN_USABLE_MTU) {
            log_e("MTU %u below MIN_USABLE_MTU=%u — handshake will fail",
                  (unsigned)mtu, (unsigned)MIN_USABLE_MTU);
        }
    }

    NimBLERemoteService* service = _impl->client->getService(svc);
    if (!service) {
        log_e("GDX service %s not found", GDX_SERVICE_UUID);
        _impl->client->disconnect();
        return false;
    }
    log_d("GDX service discovered");

    _impl->cmd_char = service->getCharacteristic(NimBLEUUID(GDX_COMMAND_CHAR_UUID));
    _impl->rsp_char = service->getCharacteristic(NimBLEUUID(GDX_RESPONSE_CHAR_UUID));
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
    if (g_ble_mutex) xSemaphoreTakeRecursive(g_ble_mutex, portMAX_DELAY);
    struct GiveOnExit {
        ~GiveOnExit() { if (g_ble_mutex) xSemaphoreGiveRecursive(g_ble_mutex); }
    } _scope;

    // Tear down in reverse order of connect():
    //   1. unsubscribe() — nulls on_notify (lambda no-ops on any
    //      in-flight notify) AND writes CCCD with response so the peer
    //      stops sending notifications before this call returns.
    //      h2zero's notify dispatch queue is drained synchronously
    //      because no new notifications can land after the CCCD ACK.
    //   2. client->disconnect() + wait — kills the BLE link.
    //   3. deleteClient — destroys the NimBLEClient and, with it, the
    //      subscribe-lambda that captured `_impl`. Safe now: the
    //      lambda is destroyed before `~NimBleXport` later runs
    //      `delete _impl`, so the captured raw pointer can't outlive
    //      its target.
    // The explicit unsubscribe at the top keeps the ordering load-
    // bearing on a contract h2zero documents (CCCD-write-with-response)
    // rather than the undocumented "deleteClient is notify-quiescent"
    // side-effect.
    unsubscribe();

    if (_impl->client) {
        if (_impl->client->isConnected()) {
            log_i("disconnect addr=%s", _impl->peer_addr.c_str());
            _impl->client->disconnect();
            // Release g_ble_mutex during the async-disconnect wait.
            // A peer disconnecting takes up to DISCO_MAX_WAIT_MS for
            // the controller to acknowledge. Holding the BLE-wide
            // mutex across that window serialises every other
            // instance's connect/disconnect path through this one
            // wait, killing throughput when N peers churn at the
            // same time. The wait only polls the per-client
            // isConnected() flag — h2zero handles that lock-free, and
            // the deleteClient call below retakes the mutex.
            xSemaphoreGiveRecursive(g_ble_mutex);
            const TickType_t kStep = pdMS_TO_TICKS(DISCO_POLL_STEP_MS);
            const TickType_t kMax  = pdMS_TO_TICKS(DISCO_MAX_WAIT_MS);
            TickType_t waited = 0;
            while (_impl->client->isConnected() && waited < kMax) {
                vTaskDelay(kStep);
                waited += kStep;
            }
            xSemaphoreTakeRecursive(g_ble_mutex, portMAX_DELAY);
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

    // Per-instance routing via lambda capture. Two properties matter:
    //   1. Multi-device safety: each session's notifications dispatch
    //      to its own Impl, not to a process-wide last-bound global.
    //   2. Reduced UAF surface: the lambda's captured `Impl*` becomes
    //      unreachable when h2zero clears subscriptions inside
    //      NimBLEDevice::deleteClient (called from disconnect() before
    //      `delete _impl`), so a notify in flight at teardown can't
    //      land on a freed pointer.
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
    // Take g_ble_mutex (recursive) so a standalone unsubscribe() from
    // a caller task can't race with connect() / disconnect() touching
    // the same characteristic / client. disconnect() also calls this
    // while already holding the mutex — recursive take is the reason
    // g_ble_mutex was promoted from non-recursive.
    if (g_ble_mutex) xSemaphoreTakeRecursive(g_ble_mutex, portMAX_DELAY);
    struct GiveOnExit {
        ~GiveOnExit() { if (g_ble_mutex) xSemaphoreGiveRecursive(g_ble_mutex); }
    } _scope;

    // Clear the indirect-through pointer first so any notification
    // already in h2zero's dispatch queue at the moment we take the
    // mutex no-ops in the lambda instead of firing a stale user
    // callback. The CCCD-write below stops the peer from generating
    // any further notifications — by the time it returns the queue
    // is drained.
    _impl->on_notify = nullptr;
    if (_impl->rsp_char) _impl->rsp_char->unsubscribe();
}

}  // namespace gogo_vernier
