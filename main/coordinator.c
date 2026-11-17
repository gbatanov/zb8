
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "ezbee/secur.h"
#include "ezbee/bdb.h"
#include "ezbee/af.h"
#include "coordinator.h"

const char *TAG = "GSB_ZB8";

static void log_nwk_info(const char *status_string)
{
    ezb_extaddr_t extpanid;
    ezb_get_extended_panid(&extpanid);
    ESP_LOGI(TAG, "%s (Extended PAN ID: 0x%016llx, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
             status_string, extpanid.u64, ezb_get_panid(), ezb_get_current_channel(), ezb_get_short_address());
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
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
        break;
    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
        if (signal_params.bdb->status == EZB_ERR_NONE)
        {
            ESP_LOGI(TAG, "Первый старт (Flash чистый), формируем новую сеть...");
            ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_FORMATION);
        }
        break;
    case EZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (signal_params.bdb->status == EZB_ERR_NONE)
        {
            ESP_LOGI(TAG, "Сеть восстановлена из памяти (Канал: %d), открываем на 120с",
                     ezb_nwk_get_current_channel());
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
            ezb_bdb_open_network(120);
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
        ESP_LOGI(TAG, "New device commissioned or rejoined (short: 0x%04hx)", signal_params.dev_annce->short_addr);
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
        if (signal_params.dev_upd->status == 1 && signal_params.dev_upd->tc_action == 0)
            test_send_active_ep(signal_params.dev_upd->short_addr);
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
    // Initialize Zigbee stack with default configuration
    ///    esp_zigbee_config_t zb_config = ESP_ZIGBEE_DEFAULT_CONFIG();
    ///  esp_zigbee_init(&zb_config);

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

    ezb_secur_tcpol_set_allow_rejoins_with_well_known_key(true);
    ezb_secur_set_tclk_exchange_required(false);
 
    // Set default allowed network channels
    ///    ESP_ERROR_CHECK(ezb_set_channel_mask(ESP_ZIGBEE_ALL_CHANNELS_MASK));
    // Also set the channels for BDB commissioning
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(ESP_ZIGBEE_ALL_CHANNELS_MASK));
    ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(0));

    // Set default network type
    ezb_aps_secur_enable_distributed_security(true);

    // Здесь вызывается ezb_config_memory(&mem_cfg) и регистрация эндпоинта
    coordinator_register_endpoint();

    // Register application signal handler
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(zb_app_signal_handler));

    ezb_af_node_desc_set_manuf_code(0x115F);
    // Enable CLI managed ep_list
    // esp_zb_console_manage_ep_list(NULL);
}

static void zb_stack_main_task(void *pvParameters)
{
    zb_stack_init();

    ESP_ERROR_CHECK(esp_zigbee_start(false));

    ///  ESP_ERROR_CHECK(ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_FORMATION));

    esp_zigbee_launch_mainloop();

    esp_zigbee_deinit();
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    //    ESP_ERROR_CHECK(esp_zb_console_init());
    xTaskCreate(zb_stack_main_task, "Zigbee_main", 4096, NULL, 5, NULL);
    //    ESP_LOGI(TAG, "Start ESP Zigbee Console");
    //   ESP_ERROR_CHECK(esp_zb_console_start());
}
