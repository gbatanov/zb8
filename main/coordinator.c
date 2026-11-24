
#include "coordinator.h"

extern SemaphoreHandle_t ack_sem;
extern QueueHandle_t q_to_pc;
extern SemaphoreHandle_t tx_mutex;
extern volatile uint8_t waiting_ack_seq;
extern const char *TAG;

uint8_t last_rx_seq = 0xFF;
volatile bool is_first_rx_packet = true;

volatile uint8_t last_af_trans_id = 0; // последний номер транзакции

// Перехватчики APSDE-DATA стека EZBEE
static bool bridge_aps_data_ind_cb(const ezb_apsde_data_ind_t *ind)
{
    if (!ind)
        return false;

    uint16_t src_short = (ind->src_address.addr_mode == EZB_ADDR_MODE_SHORT)
                             ? ind->src_address.u.short_addr
                             : 0xFFFF;

    // 1. Системные ZDO-сообщения (Profile ID == 0x0000)
    if (ind->profile_id == 0x0000)
    {
        // Device_annce (0x0013) -> ZDO_END_DEVICE_ANNCE_IND (0x45C1)
        // ASDU ZDO: [seq (1B)] [nwk_addr (2B)] [ieee_addr (8B)] [capability (1B)] = 12 байт
        // ZNP 0x45C1: [SrcAddr (2B)] [NwkAddr (2B)] [IEEEAddr (8B)] [Capabilities (1B)] = 13 байт
        if (ind->cluster_id == 0x0013 && ind->asdu_length >= 12 && ind->asdu != NULL)
        {
            uint8_t znp_annce[PROTO_MAX_PAYLOAD];
            znp_annce[0] = (uint8_t)(src_short & 0xFF);
            znp_annce[1] = (uint8_t)((src_short >> 8) & 0xFF);
            memcpy(&znp_annce[2], ind->asdu + 1, 11); // NwkAddr(2) + IEEE(8) + Cap(1)
            bridge_enqueue_to_pc(0x45C1, znp_annce, 13);
        }
        else if (ind->cluster_id == 0x0036)
        {
            // игнорируем запрос от себя же
        }
        // Остальные ZDO-ответы (ActiveEP 0x8005 -> 0x4585, SimpleDesc 0x8004 -> 0x4584 и т.д.)
        // В ZNP формат: [SrcAddr (2B)] + [ASDU без первого байта seq (asdu_length - 1)]
        else if ((ind->cluster_id & 0x8000) && ind->asdu_length >= 2 && ind->asdu != NULL)
        {
            uint16_t znp_cmd = 0x4580 | (uint8_t)(ind->cluster_id & 0x7F);
            uint8_t znp_zdo[PROTO_MAX_PAYLOAD];
            uint16_t payload_len = (uint16_t)(ind->asdu_length - 1);

            if (payload_len + 2 <= PROTO_MAX_PAYLOAD)
            {
                *(znp_zdo + 0) = (uint8_t)(src_short & 0xFF);
                *(znp_zdo + 1) = (uint8_t)((src_short >> 8) & 0xFF);
                memcpy(znp_zdo + 2, ind->asdu + 1, payload_len);
                bridge_enqueue_to_pc(znp_cmd, znp_zdo, payload_len + 2);
                ESP_LOGI(TAG, "znp_cmd 0x%04X", znp_cmd);
            }
        }
        return false;
    }

    // 2. Прикладные ZCL-сообщения (Profile ID != 0x0000) -> AF_INCOMING_MSG (0x4481)
    // Формат ZNP 0x4481 (17 байт заголовка + Data):
    // [GroupID (2B)] [ClusterID (2B)] [SrcAddr (2B)] [SrcEndpoint (1B)] [DstEndpoint (1B)]
    // [WasBroadcast (1B)] [LinkQuality (1B)] [SecurityUse (1B)] [TimeStamp (4B)]
    // [TransSeqNumber (1B)] [Len (1B)] [Data (Len B)]
    // То, что приходит в Message
    if (ind->asdu_length + 17 <= PROTO_MAX_PAYLOAD)
    {
        uint8_t out[PROTO_MAX_PAYLOAD];
        int idx = 0;

        // GroupID (0x0000 если не групповая рассылка)
        uint16_t group_id = (ind->dst_address.addr_mode == EZB_ADDR_MODE_GROUP)
                                ? ind->dst_address.u.group_addr.group
                                : 0x0000;
        out[idx++] = (uint8_t)(group_id & 0xFF);
        out[idx++] = (uint8_t)((group_id >> 8) & 0xFF);

        // ClusterID
        out[idx++] = (uint8_t)(ind->cluster_id & 0xFF);
        out[idx++] = (uint8_t)((ind->cluster_id >> 8) & 0xFF);

        // SrcAddr
        out[idx++] = (uint8_t)(src_short & 0xFF);
        out[idx++] = (uint8_t)((src_short >> 8) & 0xFF);

        // SrcEndpoint, DstEndpoint
        out[idx++] = ind->src_endpoint;
        out[idx++] = ind->dst_endpoint;

        // WasBroadcast, LinkQuality, SecurityUse
        out[idx++] = 0x00;
        out[idx++] = ind->lqi;
        out[idx++] = 0x00;

        // TimeStamp (4 байта)
        out[idx++] = 0x00;
        out[idx++] = 0x00;
        out[idx++] = 0x00;
        out[idx++] = 0x00;

        // TransSeqNumber (берём из ZCL-заголовка, если есть, или 0)
        uint8_t tsn = (ind->asdu_length >= 2 && ind->asdu != NULL) ? *(ind->asdu + 1) : 0x00;
        out[idx++] = tsn;

        // Len + Data
        out[idx++] = (uint8_t)ind->asdu_length;
        if (ind->asdu_length > 0 && ind->asdu != NULL)
        {
            memcpy(&out[idx], ind->asdu, ind->asdu_length);
            idx += ind->asdu_length;
        }

        bridge_enqueue_to_pc(0x4481, out, (uint16_t)idx);
    }

    return false;
}

// Отправка AF_DATA_CONFIRM (0x4480)
static void bridge_aps_data_cnf_cb(const ezb_apsde_data_confirm_t *confirm)
{
    if (!confirm)
        return;

    // В TI ZNP AF_DATA_CONFIRM (0x4480) отправляется только для прикладных эндпоинтов (1..240), не для ZDO (0)
    if (confirm->src_endpoint == 0)
        return;

    uint8_t out[3];
    out[0] = (uint8_t)confirm->status; // Status (0x00 = Success)
    out[1] = confirm->src_endpoint;    // Endpoint
    out[2] = last_af_trans_id;         // TransID

    bridge_enqueue_to_pc(0x4480, out, sizeof(out));
}
//-------------------------------------------
// Настройка ZIGBEE координатора

// Вывод основных параметров сети
static void log_nwk_info(const char *status_string)
{
    ezb_extaddr_t extpanid;
    ezb_get_extended_panid(&extpanid);
    ESP_LOGI(TAG, "%s (ExtPAN: 0x%016" PRIx64 ", PAN: 0x%04hx, CH: %d, Short: 0x%04hx)",
             status_string, extpanid.u64, ezb_get_panid(), ezb_nwk_get_current_channel(), ezb_get_short_address());
}

static void setup_coord_trust_center_addr(void)
{
    ezb_extaddr_t coord_ieee = {0};
    ezb_nwk_get_extended_address(&coord_ieee);
    ezb_aps_set_trust_center_address(&coord_ieee);
}

// регистрация обязательного эндпойнта самого координатора
static void coordinator_register_endpoint(void)
{
    ezb_af_device_desc_t dev_desc = ezb_af_create_device_desc();

    ezb_af_ep_config_t ep_cfg = {
        .ep_id = ZB_COORD_ENDPOINT,
        .app_profile_id = 0x0104, // Zigbee Home Automation (HA)
        .app_device_id = 0x0050,  // Home Gateway
        .app_device_version = 0,
    };

    ezb_af_ep_desc_t ep_desc = ezb_af_create_gateway_endpoint(&ep_cfg);

    // Добавляем клиентские кластеры для приёма отчётов и команд (обязательный набор)
    ezb_af_endpoint_add_cluster_desc(ep_desc, ezb_zcl_power_config_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_CLIENT));
    ezb_af_endpoint_add_cluster_desc(ep_desc, ezb_zcl_on_off_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_CLIENT));
    ezb_af_endpoint_add_cluster_desc(ep_desc, ezb_zcl_ias_zone_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_CLIENT));
    ezb_af_endpoint_add_cluster_desc(ep_desc, ezb_zcl_level_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_CLIENT));
    ezb_af_endpoint_add_cluster_desc(ep_desc, ezb_zcl_occupancy_sensing_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_CLIENT));
    ezb_af_device_add_endpoint_desc(dev_desc, ep_desc);
    ezb_af_device_desc_register(dev_desc);
}

// Обработчик сообщзений от Zigbee
static bool zb_app_signal_handler(const ezb_app_signal_t *app_signal)
{
    ezb_app_signal_type_t signal_type = ezb_app_signal_get_type(app_signal);

    union
    {
        const ezb_bdb_signal_simple_params_t *bdb;
        const ezb_zdo_signal_leave_indication_params_t *leave_ind;
        const ezb_zdo_signal_device_annce_params_t *dev_annce;
        const ezb_zdo_signal_device_authorized_params_t *dev_auth;
        const ezb_nwk_signal_permit_join_status_params_t *permit_join;
        const void *param;
    } signal_params = {.param = ezb_app_signal_get_params(app_signal)};

    switch (signal_type)
    {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        setup_coord_trust_center_addr();
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
        break;

    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
        if (signal_params.bdb->status == EZB_ERR_NONE)
        {
            ESP_LOGI(TAG, "Factory new start, forming network...");
            ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_FORMATION);
        }
        break;

    case EZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (signal_params.bdb->status == EZB_ERR_NONE)
        {
            log_nwk_info("Network restored from NVS");
        }
        else
        {
            ESP_LOGE(TAG, "Reboot commissioning failed: %d", signal_params.bdb->status);
        }
        break;

    case EZB_BDB_SIGNAL_FORMATION:
        if (signal_params.bdb->status == EZB_BDB_STATUS_SUCCESS)
        {
            log_nwk_info("Network formed successfully");
            setup_coord_trust_center_addr();
            ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to form network (status: %d)", signal_params.bdb->status);
        }
        break;

    case EZB_ZDO_SIGNAL_DEVICE_ANNCE: // после отладки удалить
    {
        uint16_t short_addr = signal_params.dev_annce->short_addr;
        ezb_extaddr_t ieee_addr = {0};
        ezb_address_extended_by_short(short_addr, &ieee_addr);

        ESP_LOGI(TAG, "Device Announce: short=0x%04x, ieee=0x%016" PRIx64, short_addr, ieee_addr.u64);
        break;
    }

    case EZB_ZDO_SIGNAL_DEVICE_AUTHORIZED: // после отладки удалить
        ESP_LOGI(TAG, "Device authorized: short=0x%04x, status=%d, type=%d",
                 signal_params.dev_auth->short_addr,
                 signal_params.dev_auth->status,
                 signal_params.dev_auth->type);
        break;

    case EZB_ZDO_SIGNAL_LEAVE_INDICATION:
    {
        // Когда пользователь нажимает кнопку сброса на датчике и тот сам уходит из сети,
        // это событие часто приходит не в виде ZCL/ZDO-пакета в bridge_aps_data_ind_cb,
        // а именно как сетевой сигнал LEAVE_INDICATION.
        // Надо отправить команду на удаление
        uint16_t short_addr = signal_params.leave_ind->short_addr;
        ESP_LOGI(TAG, "Device Left: short=0x%04x", short_addr);
        break;
    }

    case EZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        // TODO: отправить сообщение в приложение о статусе
        ESP_LOGI(TAG, "Permit Join status: %u sec", signal_params.permit_join->duration);
        break;

    default:
        break;
    }
    return true;
}

void esp_zb_task(void *pvParameters)
{
    esp_zigbee_config_t zb_cfg = {
        .platform_config = {
            .storage_partition_name = ESP_ZIGBEE_STORAGE_PARTITION,
            .radio_config = {
                .radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE,
            },
        },
        .device_config = {
            .device_type = EZB_NWK_DEVICE_TYPE_COORDINATOR,
            .install_code_policy = INSTALLCODE_POLICY_ENABLE,
            .zczr_config = {
                .max_children = MAX_CHILDREN,
            },
        },
    };

    esp_zigbee_init(&zb_cfg);

    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(ZB_CHANNELS_MASK));
    ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(0));
    /* Set default allowed network channels */
    ESP_ERROR_CHECK(ezb_set_channel_mask(ZB_CHANNELS_MASK));

    static const uint8_t zigbee_alliance09_key[16] = {
        0x5A, 0x69, 0x67, 0x42, 0x65, 0x65, 0x41, 0x6C,
        0x6C, 0x69, 0x61, 0x6E, 0x63, 0x65, 0x30, 0x39};
    ezb_secur_set_global_link_key(zigbee_alliance09_key);

    ezb_bdb_set_router_rejoin_required(true);
    ezb_secur_set_kn_method_enabled(EZB_SECUR_KN_METHOD_STATIC_REQUEST_KEY, true);
    // ezb_secur_set_tclk_exchange_required(true);
    //  Отключаем принудительный кик по 15-секундному таймеру BDB
    ezb_secur_set_tclk_exchange_required(false);

    ezb_secur_tcpol_set_allow_rejoins_with_well_known_key(true);
    ezb_secur_set_kn_method_enabled(EZB_SECUR_KN_METHOD_ECSPEKE_CURVE25519_AESMMO, true);
    ezb_secur_set_kn_method_enabled(EZB_SECUR_KN_METHOD_ECSPEKE_CURVE25519_SHA256, true);

    // Set default network type
    ezb_aps_secur_enable_distributed_security(true);
    ezb_af_node_desc_set_manuf_code(0x115F); // TODO: делать только для Aqara

    // Регистрация обработчика всех входящих ZCL-событий и ответов
    setup_coord_trust_center_addr();

    ezb_app_signal_add_handler(zb_app_signal_handler);
    coordinator_register_endpoint();

    ezb_apsde_data_indication_handler_register(bridge_aps_data_ind_cb);
    ezb_apsde_data_confirm_handler_register(bridge_aps_data_cnf_cb);

    ESP_ERROR_CHECK(esp_zigbee_start(false));
    esp_zigbee_launch_mainloop();
}
