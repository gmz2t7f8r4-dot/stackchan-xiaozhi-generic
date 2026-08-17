#include "stackchan_ble_compat.h"

#include <cstring>

#include "esp_log.h"
#include "esp_mac.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

namespace {

constexpr char kTag[] = "StackChanBle";
constexpr size_t kMaxFrameLength = 768;

// e2e5e5e0-1234-5678-1234-56789abcdef0
const ble_uuid128_t kDanceServiceUuid = BLE_UUID128_INIT(
    0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0xe0, 0xe5, 0xe5, 0xe2);
// e2e5e5e1-1234-5678-1234-56789abcdef0
const ble_uuid128_t kMotionUuid = BLE_UUID128_INIT(
    0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0xe1, 0xe5, 0xe5, 0xe2);
// e2e5e5e2-1234-5678-1234-56789abcdef0
const ble_uuid128_t kAvatarUuid = BLE_UUID128_INIT(
    0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0xe2, 0xe5, 0xe5, 0xe2);
// e2e5e5e3-1234-5678-1234-56789abcdef0
const ble_uuid128_t kConfigUuid = BLE_UUID128_INIT(
    0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0xe3, 0xe5, 0xe5, 0xe2);
// e2e5e5e4-1234-5678-1234-56789abcdef0
const ble_uuid128_t kRgbUuid = BLE_UUID128_INIT(
    0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0xe4, 0xe5, 0xe5, 0xe2);

enum class CharacteristicKind : uintptr_t { Motion = 1, Avatar = 2, Config = 3, Rgb = 4 };

uint8_t g_own_addr_type = BLE_OWN_ADDR_PUBLIC;

int AccessCharacteristic(uint16_t, uint16_t, ble_gatt_access_ctxt* context,
                         void* arg) {
    const auto kind = static_cast<CharacteristicKind>(
        reinterpret_cast<uintptr_t>(arg));
    if (context->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        static constexpr char kEmptyJson[] = "{}";
        return os_mbuf_append(context->om, kEmptyJson, sizeof(kEmptyJson) - 1) == 0
                   ? 0
                   : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (context->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    const uint16_t length = OS_MBUF_PKTLEN(context->om);
    if (length == 0 || length > kMaxFrameLength) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    char payload[kMaxFrameLength + 1];
    uint16_t copied = 0;
    const int result = ble_hs_mbuf_to_flat(context->om, payload,
                                            kMaxFrameLength, &copied);
    if (result != 0) return BLE_ATT_ERR_UNLIKELY;
    payload[copied] = '\0';

    auto& ble = StackChanBleCompat::Instance();
    if (kind == CharacteristicKind::Motion) ble.DispatchMotion(payload, copied);
    else if (kind == CharacteristicKind::Avatar) ble.DispatchAvatar(payload, copied);
    else if (kind == CharacteristicKind::Rgb) ble.DispatchRgb(payload, copied);
    return 0;
}

ble_gatt_chr_def g_characteristics[] = {
    {.uuid = &kMotionUuid.u, .access_cb = AccessCharacteristic,
     .arg = reinterpret_cast<void*>(static_cast<uintptr_t>(CharacteristicKind::Motion)),
     .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE},
    {.uuid = &kAvatarUuid.u, .access_cb = AccessCharacteristic,
     .arg = reinterpret_cast<void*>(static_cast<uintptr_t>(CharacteristicKind::Avatar)),
     .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE},
    {.uuid = &kConfigUuid.u, .access_cb = AccessCharacteristic,
     .arg = reinterpret_cast<void*>(static_cast<uintptr_t>(CharacteristicKind::Config)),
     .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE},
    {.uuid = &kRgbUuid.u, .access_cb = AccessCharacteristic,
     .arg = reinterpret_cast<void*>(static_cast<uintptr_t>(CharacteristicKind::Rgb)),
     .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE},
    {0},
};

ble_gatt_svc_def g_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &kDanceServiceUuid.u,
        .characteristics = g_characteristics,
    },
    {0},
};

void StartAdvertising();

int GapEvent(ble_gap_event* event, void*) {
    auto& ble = StackChanBleCompat::Instance();
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ble.SetConnected(true);
            } else {
                StartAdvertising();
            }
            return 0;
        case BLE_GAP_EVENT_DISCONNECT:
            ble.SetConnected(false);
            if (ble.IsActive()) StartAdvertising();
            return 0;
        case BLE_GAP_EVENT_ADV_COMPLETE:
            if (ble.IsActive()) StartAdvertising();
            return 0;
        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(kTag, "MTU updated: %u", event->mtu.value);
            return 0;
        default:
            return 0;
    }
}

void StartAdvertising() {
    if (!StackChanBleCompat::Instance().IsActive()) return;
    ble_hs_adv_fields fields{};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = const_cast<ble_uuid128_t*>(&kDanceServiceUuid);
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(kTag, "Failed to set advertising fields: %d", rc);
        return;
    }

    uint8_t manufacturer[8] = {0xE5, 0x02};
    esp_read_mac(manufacturer + 2, ESP_MAC_EFUSE_FACTORY);
    ble_hs_adv_fields response{};
    const char* name = ble_svc_gap_device_name();
    response.name = reinterpret_cast<const uint8_t*>(name);
    response.name_len = std::strlen(name);
    response.name_is_complete = 1;
    response.mfg_data = manufacturer;
    response.mfg_data_len = sizeof(manufacturer);
    rc = ble_gap_adv_rsp_set_fields(&response);
    if (rc != 0) {
        ESP_LOGE(kTag, "Failed to set scan response: %d", rc);
        return;
    }

    ble_gap_adv_params parameters{};
    parameters.conn_mode = BLE_GAP_CONN_MODE_UND;
    parameters.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(g_own_addr_type, nullptr, BLE_HS_FOREVER,
                           &parameters, GapEvent, nullptr);
    if (rc == 0) {
        ESP_LOGI(kTag, "Advertising started successfully");
        StackChanBleCompat::Instance().ReportAdvertisingResult(true);
    } else {
        ESP_LOGE(kTag, "Advertising failed: rc=%d", rc);
        StackChanBleCompat::Instance().ReportAdvertisingResult(false);
    }
}

void OnSync() {
    int rc = ble_hs_util_ensure_addr(0);
    if (rc == 0) rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(kTag, "Unable to select BLE identity address: %d", rc);
        return;
    }
    StartAdvertising();
}

void HostTask(void*) {
    auto& ble = StackChanBleCompat::Instance();
    ESP_LOGI(kTag, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
    ble.SetConnected(false);
    ESP_LOGI(kTag, "NimBLE host task stopped");
}

}  // namespace

StackChanBleCompat& StackChanBleCompat::Instance() {
    static StackChanBleCompat instance;
    return instance;
}

bool StackChanBleCompat::Start(DataCallback motion, DataCallback avatar,
                               DataCallback rgb, StatusCallback status) {
    if (active_.exchange(true)) return true;
    motion_callback_ = std::move(motion);
    avatar_callback_ = std::move(avatar);
    rgb_callback_ = std::move(rgb);
    status_callback_ = std::move(status);

    const esp_err_t init_result = nimble_port_init();
    if (init_result != ESP_OK) {
        ESP_LOGE(kTag, "nimble_port_init failed: %s", esp_err_to_name(init_result));
        active_.store(false);
        return false;
    }

    ble_svc_gap_init();
    ble_svc_gatt_init();
    int rc = ble_gatts_count_cfg(g_services);
    if (rc == 0) rc = ble_gatts_add_svcs(g_services);
    if (rc == 0) rc = ble_svc_gap_device_name_set("StackChan");
    if (rc != 0) {
        ESP_LOGE(kTag, "Failed to register official dance service: %d", rc);
        nimble_port_deinit();
        active_.store(false);
        return false;
    }

    ble_hs_cfg.sync_cb = OnSync;
    host_running_.store(true);
    nimble_port_freertos_init(HostTask);
    ESP_LOGI(kTag, "Official StackChan dance BLE mode ready");
    return true;
}

void StackChanBleCompat::Stop() {
    if (!active_.exchange(false)) return;
    ble_gap_adv_stop();
    if (connected_.load()) {
        // One connection is supported; querying is unnecessary because NimBLE
        // will terminate it as part of host shutdown.
        connected_.store(false);
    }
    nimble_port_stop();
    vTaskDelay(pdMS_TO_TICKS(120));
    nimble_port_deinit();
    host_running_.store(false);
    motion_callback_ = nullptr;
    avatar_callback_ = nullptr;
    rgb_callback_ = nullptr;
    status_callback_ = nullptr;
    ESP_LOGI(kTag, "Official StackChan dance BLE mode stopped");
}

void StackChanBleCompat::DispatchMotion(const char* data, size_t len) {
    if (active_.load() && motion_callback_) motion_callback_(std::string(data, len));
}

void StackChanBleCompat::DispatchAvatar(const char* data, size_t len) {
    if (active_.load() && avatar_callback_) avatar_callback_(std::string(data, len));
}

void StackChanBleCompat::DispatchRgb(const char* data, size_t len) {
    if (active_.load() && rgb_callback_) rgb_callback_(std::string(data, len));
}

void StackChanBleCompat::SetConnected(bool connected) {
    connected_.store(connected);
    if (status_callback_) {
        status_callback_(connected ? Status::Connected : Status::Advertising);
    }
    ESP_LOGI(kTag, "Official app %s", connected ? "connected" : "disconnected");
}

void StackChanBleCompat::ReportAdvertisingResult(bool ready) {
    if (status_callback_) {
        status_callback_(ready ? Status::Advertising : Status::Error);
    }
}
