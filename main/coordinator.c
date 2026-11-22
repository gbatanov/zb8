
#include "coordinator.h"

extern SemaphoreHandle_t ack_sem;
extern QueueHandle_t q_to_pc;
extern SemaphoreHandle_t tx_mutex;
extern volatile uint8_t waiting_ack_seq;
extern const char *TAG;

uint8_t last_rx_seq = 0xFF;
volatile bool is_first_rx_packet = true;

// Выполнение команд от ПО Zhub  в стеке zigbee(ezbee) 
static void zdo_bind_bridge_cb(const ezb_zdp_bind_req_result_t *result, void *user_ctx)
{
    ESP_LOGI(TAG, "ZDO Bind completed (result=%p)", result);
}

static void execute_pc_command(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    switch (cmd)
    {
    case CMD_SYS_PING:
    {
        uint8_t status[7];
        uint32_t uptime = (uint32_t)(xTaskGetTickCount() / configTICK_RATE_HZ);

        esp_zigbee_lock_acquire(portMAX_DELAY);
        uint16_t pan_id = ezb_get_panid();
        uint8_t channel = ezb_nwk_get_current_channel();
        esp_zigbee_lock_release();

        status[0] = (uint8_t)(uptime & 0xFF);
        status[1] = (uint8_t)((uptime >> 8) & 0xFF);
        status[2] = (uint8_t)((uptime >> 16) & 0xFF);
        status[3] = (uint8_t)((uptime >> 24) & 0xFF);
        status[4] = (uint8_t)(pan_id & 0xFF);
        status[5] = (uint8_t)((pan_id >> 8) & 0xFF);
        status[6] = channel;

        bridge_enqueue_to_pc(CMD_SYS_PING, status, sizeof(status));
        break;
    }

    case CMD_PERMIT_JOIN:
    {
        if (len >= 1)
        {
            uint8_t duration = payload[0];
            esp_zigbee_lock_acquire(portMAX_DELAY);
            ezb_err_t err = ezb_bdb_open_network(duration);
            esp_zigbee_lock_release();
            ESP_LOGI(TAG, "Permit Join (%u sec) -> err=0x%x", duration, err);
        }
        break;
    }

    case CMD_APS_SEND_RAW:
    {
        // Формат: [dst_short (2B)] [src_ep (1B)] [dst_ep (1B)] [profile_id (2B)] [cluster_id (2B)] [tx_opt (1B)] [asdu...]
        if (len >= 9)
        {
            uint16_t dst_short = payload[0] | ((uint16_t)payload[1] << 8);
            uint8_t src_ep = payload[2];
            uint8_t dst_ep = payload[3];
            uint16_t profile_id = payload[4] | ((uint16_t)payload[5] << 8);
            uint16_t cluster_id = payload[6] | ((uint16_t)payload[7] << 8);
            uint8_t tx_opt = payload[8];
            uint16_t asdu_len = len - 9;

            uint8_t *asdu_copy = NULL;
            if (asdu_len > 0)
            {
                asdu_copy = (uint8_t *)malloc(asdu_len);
                if (!asdu_copy)
                {
                    ESP_LOGE(TAG, "No memory for APS ASDU");
                    break;
                }
                memcpy(asdu_copy, &payload[9], asdu_len);
            }

            ezb_apsde_data_req_t req = {
                .dst_address = {
                    .addr_mode = EZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                    .u.short_addr = dst_short,
                },
                .src_endpoint = src_ep,
                .dst_endpoint = dst_ep,
                .cluster_id = cluster_id,
                .profile_id = profile_id,
                .radius = 0,
                .tx_options = (tx_opt == 0) ? (EZB_APSDE_TX_OPT_SECURITY_ENABLED | EZB_APSDE_TX_OPT_ACK_TX) : tx_opt,
                .asdu_length = asdu_len,
                .asdu = asdu_copy,
            };

            esp_zigbee_lock_acquire(portMAX_DELAY);
            ezb_err_t err = ezb_apsde_data_request(&req);
            esp_zigbee_lock_release();

            if (asdu_copy)
            {
                free(asdu_copy);
            }

            if (err != EZB_ERR_NONE)
            {
                ESP_LOGE(TAG, "APS send failed to 0x%04X cluster 0x%04X: err=0x%x", dst_short, cluster_id, err);
            }
        }
        break;
    }

    case CMD_ZDO_BIND_REQ:
    {
        // Формат: [dst_short (2B)] [src_ieee (8B)] [src_ep (1B)] [cluster_id (2B)] [dst_ep (1B)]
        if (len >= 14)
        {
            uint16_t dst_short = payload[0] | ((uint16_t)payload[1] << 8);
            uint8_t src_ep = payload[10];
            uint16_t cluster_id = payload[11] | ((uint16_t)payload[12] << 8);
            uint8_t dst_ep = payload[13];

            esp_zigbee_lock_acquire(portMAX_DELAY);
            ezb_extaddr_t coord_ieee = {0};
            ezb_nwk_get_extended_address(&coord_ieee);

            ezb_zdo_bind_req_t bind_req = {
                .dst_nwk_addr = dst_short,
                .field = {
                    .src_ep = src_ep,
                    .cluster_id = cluster_id,
                    .dst_addr_mode = 0x03,
                    .dst_ep = (dst_ep == 0) ? ZB_COORD_ENDPOINT : dst_ep,
                },
                .cb = zdo_bind_bridge_cb,
                .user_ctx = NULL,
            };
            memcpy(&bind_req.field.src_addr, &payload[2], sizeof(ezb_extaddr_t));
            memcpy(&bind_req.field.dst_addr, &coord_ieee, sizeof(ezb_extaddr_t));

            ezb_err_t err = ezb_zdo_bind_req(&bind_req);
            esp_zigbee_lock_release();

            ESP_LOGI(TAG, "ZDO Bind to 0x%04X cluster 0x%04X -> err=0x%x", dst_short, cluster_id, err);
        }
        break;
    }

    default:
        ESP_LOGW(TAG, "Unknown CMD from PC: 0x%02X", cmd);
        break;
    }
}

/* ==================== ЗАДАЧА RX (ПАРСЕР ВХОДЯЩИХ КАДРОВ ОТ ПК) ==================== */
void on_valid_pc_frame(uint8_t seq, uint8_t type, uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    if (type == PKT_TYPE_ACK)
    {
        if (seq == waiting_ack_seq)
        {
            xSemaphoreGive(ack_sem);
        }
        return;
    }

    if (type == PKT_TYPE_DATA)
    {
        uart_send_raw_frame(seq, PKT_TYPE_ACK, cmd, NULL, 0);

        if (!is_first_rx_packet && seq == last_rx_seq)
        {
            ESP_LOGW(TAG, "Duplicate frame SEQ=%u ignored", seq);
            return;
        }
        is_first_rx_packet = false;
        last_rx_seq = seq;

        execute_pc_command(cmd, payload, len);
    }
}

/* ==================== ПЕРЕХВАТЧИКИ APSDE-DATA СТЕКА EZBEE ==================== */
static bool bridge_aps_data_ind_cb(const ezb_apsde_data_ind_t *ind)
{
    if (!ind)
        return false;

    // Формат CMD_APS_RECV_RAW (10 байт заголовка + ASDU):
    // [src_short (2B)] [src_ep (1B)] [dst_ep (1B)] [profile_id (2B)] [cluster_id (2B)] [rssi (1B)] [lqi (1B)] [asdu...]
    if (ind->asdu_length + 10 <= PROTO_MAX_PAYLOAD)
    {
        uint8_t out[PROTO_MAX_PAYLOAD];
        int idx = 0;

        uint16_t src_short = (ind->src_address.addr_mode == EZB_ADDR_MODE_SHORT)
                                 ? ind->src_address.u.short_addr
                                 : 0xFFFF;

        out[idx++] = (uint8_t)(src_short & 0xFF);
        out[idx++] = (uint8_t)((src_short >> 8) & 0xFF);
        out[idx++] = ind->src_endpoint;
        out[idx++] = ind->dst_endpoint;
        out[idx++] = (uint8_t)(ind->profile_id & 0xFF);
        out[idx++] = (uint8_t)((ind->profile_id >> 8) & 0xFF);
        out[idx++] = (uint8_t)(ind->cluster_id & 0xFF);
        out[idx++] = (uint8_t)((ind->cluster_id >> 8) & 0xFF);
        out[idx++] = (uint8_t)ind->rssi;
        out[idx++] = ind->lqi;

        if (ind->asdu_length > 0 && ind->asdu != NULL)
        {
            memcpy(&out[idx], ind->asdu, ind->asdu_length);
            idx += ind->asdu_length;
        }

        bridge_enqueue_to_pc(CMD_APS_RECV_RAW, out, (uint16_t)idx);
    }

    // Возвращаем false, чтобы системный стек продолжил базовую обработку (ZDO/BDB/DefaultRsp)
    return false;
}

static void bridge_aps_data_cnf_cb(const ezb_apsde_data_confirm_t *confirm)
{
    if (!confirm)
        return;

    uint8_t out[6];
    uint16_t dst_short = (confirm->dst_address.addr_mode == EZB_ADDR_MODE_SHORT)
                             ? confirm->dst_address.u.short_addr
                             : 0xFFFF;

    out[0] = (uint8_t)(dst_short & 0xFF);
    out[1] = (uint8_t)((dst_short >> 8) & 0xFF);
    out[2] = confirm->dst_endpoint;
    out[3] = (uint8_t)(confirm->cluster_id & 0xFF);
    out[4] = (uint8_t)((confirm->cluster_id >> 8) & 0xFF);
    out[5] = confirm->status;

    bridge_enqueue_to_pc(CMD_APS_DATA_CNF, out, sizeof(out));
}

/* ==================== НАСТРОЙКА ZIGBEE КООРДИНАТОРА ==================== */
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
    ezb_af_device_add_endpoint_desc(dev_desc, ep_desc);
    ezb_af_device_desc_register(dev_desc);
}

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

    case EZB_ZDO_SIGNAL_DEVICE_ANNCE:
    {
        uint16_t short_addr = signal_params.dev_annce->short_addr;
        ezb_extaddr_t ieee_addr = {0};
        ezb_address_extended_by_short(short_addr, &ieee_addr);

        ESP_LOGI(TAG, "Device Announce: short=0x%04x, ieee=0x%016" PRIx64, short_addr, ieee_addr.u64);

        uint8_t buf[11];
        buf[0] = (uint8_t)(short_addr & 0xFF);
        buf[1] = (uint8_t)((short_addr >> 8) & 0xFF);
        memcpy(&buf[2], ieee_addr.u8, 8);
        buf[10] = signal_params.dev_annce->capability;
        bridge_enqueue_to_pc(CMD_DEV_ANNOUNCE, buf, sizeof(buf));
        break;
    }

    case EZB_ZDO_SIGNAL_DEVICE_AUTHORIZED:
        ESP_LOGI(TAG, "Device authorized: short=0x%04x, status=%d, type=%d",
                 signal_params.dev_auth->short_addr,
                 signal_params.dev_auth->status,
                 signal_params.dev_auth->type);
        break;

    case EZB_ZDO_SIGNAL_LEAVE_INDICATION:
    {
        uint16_t short_addr = signal_params.leave_ind->short_addr;
        ESP_LOGI(TAG, "Device Left: short=0x%04x", short_addr);

        uint8_t buf[10];
        buf[0] = (uint8_t)(short_addr & 0xFF);
        buf[1] = (uint8_t)((short_addr >> 8) & 0xFF);
        memcpy(&buf[2], signal_params.leave_ind->device_addr.u8, 8);
        bridge_enqueue_to_pc(CMD_DEV_LEAVE, buf, sizeof(buf));
        break;
    }

    case EZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
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

    ezb_app_signal_add_handler(zb_app_signal_handler);
    coordinator_register_endpoint();

    ezb_apsde_data_indication_handler_register(bridge_aps_data_ind_cb);
    ezb_apsde_data_confirm_handler_register(bridge_aps_data_cnf_cb);

    ezb_set_channel_mask(ZB_CHANNELS_MASK);

    ESP_ERROR_CHECK(esp_zigbee_start(false));
    esp_zigbee_launch_mainloop();
}
