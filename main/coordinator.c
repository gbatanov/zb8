
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "ezbee/secur.h"
#include "ezbee/bdb.h"
#include "ezbee/af.h"
#include "ezbee/zcl.h"
#include "ezbee/nwk.h"
#include "ezbee/zdo/zdo_dev_srv_disc.h"
#include "ezbee/zdo/zdo_bind_mgmt.h"

#include "coordinator.h"

const char *TAG = "GSB_ZB8";

// Контекст последнего подключаемого устройства

joining_device_ctx_t s_joining_dev = {0};

/// @brief /
/// @param status_string
static void log_nwk_info(const char *status_string)
{
    ezb_extaddr_t extpanid;
    ezb_get_extended_panid(&extpanid);
    ESP_LOGI(TAG, "%s (Extended PAN ID: 0x%016llx, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
             status_string, extpanid.u64, ezb_get_panid(), ezb_get_current_channel(), ezb_get_short_address());
}

static void zb_action_handler(ezb_zcl_core_action_callback_id_t callback_id, void *message)
{
    ESP_LOGI(TAG, ">>> ZCL ACTION: cb_id=0x%x", callback_id);

    switch (callback_id)
    {
    case EZB_ZCL_CORE_READ_ATTR_RSP_CB_ID:
        ESP_LOGI(TAG, ">>> ПОЛУЧЕН ОТВЕТ ZCL Read Attributes от устройства!");
        break;

    case EZB_ZCL_CORE_REPORT_ATTR_CB_ID:
        ESP_LOGI(TAG, ">>> ПОЛУЧЕН ОТЧЕТ ZCL Report Attributes!");
        break;

    case EZB_ZCL_CORE_DEFAULT_RSP_CB_ID:
        ESP_LOGI(TAG, ">>> ПОЛУЧЕН Default Response от устройства!");
        break;

    default:
        break;
    }
}

static bool zb_app_signal_handler(const ezb_app_signal_t *app_signal)
{
    ESP_LOGI(TAG, "zb_app_signal_handler");

    ezb_app_signal_type_t signal_type = ezb_app_signal_get_type(app_signal);

    union
    {
        const ezb_bdb_signal_simple_params_t *bdb;
        const ezb_zdo_signal_leave_indication_params_t *leave_ind;
        const ezb_zdo_signal_device_annce_params_t *dev_annce;
        const ezb_zdo_signal_device_update_params_t *dev_upd;
        const ezb_zdo_signal_device_authorized_params_t *dev_auth;
        const ezb_zdo_signal_device_unavailable_params_t *dev_unavail;
        const ezb_nwk_signal_permit_join_status_params_t *permit_join;
        const ezb_nwk_signal_network_status_params_t *nwk_status;
        const void *param;
    } signal_params = {.param = ezb_app_signal_get_params(app_signal)};

    switch (signal_type)
    {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack:: bdb_comm start init");
        setup_coord_trust_center_addr();

        break;
    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:

        if (signal_params.bdb->status == EZB_ERR_NONE)
        {
            ESP_LOGI(TAG, "Первый старт (Flash чистый), формируем новую сеть...:: bdb_comm start form");
            ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_FORMATION);
        }
        break;

        ESP_LOGI(TAG, "Device first start");
        /* fall through */

    case EZB_BDB_SIGNAL_DEVICE_REBOOT:

        if (signal_params.bdb->status == EZB_ERR_NONE)
        {
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", ezb_bdb_is_factory_new() ? "" : " non");
            ESP_LOGI(TAG, "Сеть восстановлена из памяти (Канал: %d), открываем на 120с", ezb_nwk_get_current_channel());
            ezb_bdb_open_network(120);
        }
        else
        {
            ESP_LOGE(TAG, "%s failed with status: %d, please retry", ezb_app_signal_to_string(signal_type),
                     signal_params.bdb->status);
        }

        break;
    case EZB_BDB_SIGNAL_FORMATION:
        if (signal_params.bdb->status == EZB_BDB_STATUS_SUCCESS)
        {
            log_nwk_info("Formed network successfully");
            setup_coord_trust_center_addr();
        }
        else
        {
            ESP_LOGI(TAG, "Failed to form network (status: %d)", signal_params.bdb->status);
        }
        break;
    case EZB_BDB_SIGNAL_STEERING:
        if (signal_params.bdb->status == EZB_BDB_STATUS_SUCCESS)
        {
            log_nwk_info("Joined network successfully");
        }
        else
        {
            ESP_LOGI(TAG, "Failed to join network (status: %d)", signal_params.bdb->status);
        }
        break;
    case EZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGI(TAG, "Left network successfully");
        break;
    case EZB_ZDO_SIGNAL_LEAVE_INDICATION:
        ESP_LOGI(TAG, "Zigbee Node (0x%04hx) is leaving network", signal_params.leave_ind->short_addr);
        break;
    case EZB_ZDO_SIGNAL_DEVICE_ANNCE:
        if (s_joining_dev.short_addr != 0 && s_joining_dev.short_addr != 0xFFFF)
        {
            ESP_LOGI(TAG, "Device annce: short=0x%04x", s_joining_dev.short_addr);

            ezb_zdo_active_ep_req_t req = {
                .dst_nwk_addr = s_joining_dev.short_addr,
                .field = {
                    .nwk_addr_of_interest = s_joining_dev.short_addr,
                },
                .cb = zdo_active_ep_cb,
                .user_ctx = NULL,
            };
            ezb_err_t err = ezb_zdo_active_ep_req(&req);
            ESP_LOGI(TAG, "Sent Active EP Req to 0x%04x, err=0x%x", s_joining_dev.short_addr, err);
        }
        break;
    case EZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (signal_params.permit_join->duration)
        {
            ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", ezb_get_panid(), signal_params.permit_join->duration);
        }
        else
        {
            ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", ezb_get_panid());
        }
        break;
    case EZB_BDB_SIGNAL_TOUCHLINK_TARGET_FINISHED:
        if (signal_params.bdb->status == EZB_BDB_STATUS_SUCCESS)
        {
            log_nwk_info("Touchlink commissioning successfully");
        }
        else
        {
            ESP_LOGW(TAG, "Touchlink commissioning failed (status: %d)", signal_params.bdb->status);
        }
        ESP_LOGI(TAG, "Touchlink target finished (status: %d)", signal_params.bdb->status);
        break;
    case EZB_BDB_SIGNAL_TOUCHLINK_INITIATOR_FINISHED:
        if (signal_params.bdb->status == EZB_BDB_STATUS_SUCCESS)
        {
            log_nwk_info("Touchlink commissioning successfully");
        }
        else
        {
            ESP_LOGW(TAG, "No Touchlink target devices found");
        }
        ESP_LOGI(TAG, "Touchlink initiator finished (status: %d)", signal_params.bdb->status);
        break;
    case EZB_NWK_SIGNAL_NETWORK_STATUS:
        ESP_LOGW(TAG, "NLME-NWK-STATUS: %s(%d), address: 0x%04" PRIx16,
                 ezb_nwk_network_status_to_string(signal_params.nwk_status->status),
                 signal_params.nwk_status->status, signal_params.nwk_status->network_addr);
        break;
    case EZB_ZDO_SIGNAL_DEVICE_UPDATE:
        ESP_LOGI(TAG, "Device(0x%04x:0x%016llx) update: parent(0x%04x) status(%d) tc_action(%d)",
                 signal_params.dev_upd->short_addr, signal_params.dev_upd->device_addr.u64,
                 signal_params.dev_upd->parent_short, signal_params.dev_upd->status, signal_params.dev_upd->tc_action);
        s_joining_dev.short_addr = signal_params.dev_upd->short_addr;                                     // переменная, из которой печатается 0xd0b4
        memcpy(&s_joining_dev.ieee_addr, &signal_params.dev_upd->device_addr.u64, sizeof(ezb_extaddr_t)); // откуда печатается 0x54ef4410005b2639

        break;
    case EZB_ZDO_SIGNAL_DEVICE_AUTHORIZED:
        ESP_LOGI(TAG, "Device(0x%04x:0x%016llx) authorized: status(%d) type (%d)",
                 signal_params.dev_auth->short_addr, signal_params.dev_auth->device_addr.u64,
                 signal_params.dev_auth->status, signal_params.dev_auth->type);
        break;
    case EZB_ZDO_SIGNAL_DEVICE_UNAVAILABLE:
        ESP_LOGI(TAG, "Device(0x%04x:0x%016llx) is unavailable",
                 signal_params.dev_unavail->short_addr, signal_params.dev_unavail->device_addr.u64);
        break;
    case EZB_BDB_SIGNAL_FINDING_AND_BINDING_TARGET_FINISHED:
    case EZB_BDB_SIGNAL_FINDING_AND_BINDING_INITIATOR_FINISHED:
    case EZB_ZDO_SIGNAL_DEFAULT_START:
    case EZB_ZDO_SIGNAL_ERROR:
    case EZB_NWK_SIGNAL_DEVICE_ASSOCIATED:
    case EZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT:
    case EZB_NWK_SIGNAL_PANID_CONFLICT_DETECTED:
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x)", ezb_app_signal_to_string(signal_type), signal_type);
        break;
    }

    return false;
}


void zb_stack_init(void)
{

    ESP_LOGI(TAG, "1. Старт задачи Zigbee...");

    esp_zigbee_config_t zb_cfg = {
        .device_config = {
            .device_type = EZB_NWK_DEVICE_TYPE_COORDINATOR, // <-- Обязательно Coordinator!
            .install_code_policy = false,
            .zczr_config = {
                .max_children = 32,
            },
        },
        .platform_config = {
            .radio_config = {
                .radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE,
            },
        },
    };
    ESP_ERROR_CHECK(esp_zigbee_init(&zb_cfg));
    ESP_LOGI(TAG, "2. Стек инициализирован");

    // Set default allowed network channels
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(ESP_ZIGBEE_ALL_CHANNELS_MASK));
    ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(0));
    /* Set default allowed network channels */
    ESP_ERROR_CHECK(ezb_set_channel_mask(ESP_ZIGBEE_ALL_CHANNELS_MASK));

    static const uint8_t zigbee_alliance09_key[16] = {
        0x5A, 0x69, 0x67, 0x42, 0x65, 0x65, 0x41, 0x6C,
        0x6C, 0x69, 0x61, 0x6E, 0x63, 0x65, 0x30, 0x39};
    ezb_secur_set_global_link_key(zigbee_alliance09_key);

    ezb_af_node_desc_set_manuf_code(0x0115F); // Код производителя Lumi/Aqara 0x115
    ezb_bdb_set_router_rejoin_required(true);
    ezb_secur_set_kn_method_enabled(EZB_SECUR_KN_METHOD_STATIC_REQUEST_KEY, true);
    ezb_secur_set_tclk_exchange_required(true);
    ezb_secur_tcpol_set_allow_rejoins_with_well_known_key(true);
    ezb_secur_set_kn_method_enabled(EZB_SECUR_KN_METHOD_ECSPEKE_CURVE25519_AESMMO, true);
    ezb_secur_set_kn_method_enabled(EZB_SECUR_KN_METHOD_ECSPEKE_CURVE25519_SHA256, true);

    // Set default network type
    ezb_aps_secur_enable_distributed_security(true);
    // Register application signal handler
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(zb_app_signal_handler));

    ezb_af_node_desc_set_manuf_code(0x115F); // TODO: делать только для Aqara

    /// Регистрация обработчика всех входящих ZCL-событий и ответов
    setup_coord_trust_center_addr();
    ezb_zcl_core_action_handler_register(zb_action_handler);
    // Здесь вызывается ezb_config_memory(&mem_cfg) и регистрация эндпоинта
    coordinator_register_endpoint();
    ESP_ERROR_CHECK(ezb_dev_start(true));
    ESP_LOGI(TAG, "start");
}

static void zb_stack_main_task(void *pvParameters)
{
    zb_stack_init();

    esp_zigbee_start(false);

    ESP_ERROR_CHECK(ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION));
   ///  ESP_ERROR_CHECK(ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_FORMATION));

    ESP_ERROR_CHECK(esp_zigbee_launch_mainloop());

    esp_zigbee_deinit();
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    xTaskCreate(zb_stack_main_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
