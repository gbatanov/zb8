#ifndef OLD_VERSION
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_zigbee.h"
#include "ezbee/af.h"
#include "ezbee/aps.h"
#include "ezbee/bdb.h"
#include "ezbee/nwk.h"
#include "ezbee/secur.h"
#include "ezbee/zcl.h"
#include "ezbee/zdo/zdo_bind_mgmt.h"
#include "ezbee/zdo/zdo_dev_srv_disc.h"

static const char *TAG = "GSB_ZB8_NCP";

/* ==================== НАСТРОЙКИ ZIGBEE ==================== */
#define MAX_CHILDREN 32
#define INSTALLCODE_POLICY_ENABLE false
#define ESP_ZIGBEE_STORAGE_PARTITION "nvs"
#define ZB_COORD_ENDPOINT 1
// Все каналы 11..26: 0x07FFF800U (сейчас оставлен 15-й канал 0x00008000U)
#define ZB_CHANNELS_MASK (0x00008000U)

/* ==================== НАСТРОЙКИ UART0 И ПРОТОКОЛА ==================== */
#define PROTO_UART_NUM UART_NUM_0
#define PROTO_UART_BAUD 115200
#define PROTO_MAX_PAYLOAD 256
#define PROTO_ACK_TIMEOUT_MS 150
#define PROTO_MAX_RETRIES 3

#define PROTO_SOF_0 0xC0
#define PROTO_SOF_1 0xC0

typedef enum
{
    PKT_TYPE_DATA = 0x01,
    PKT_TYPE_ACK = 0x02,
    PKT_TYPE_NACK = 0x03,
} proto_pkt_type_t;

/* Команды бинарного протокола (ESP32-H2 <-> Golang ПК) */
typedef enum
{
    CMD_PERMIT_JOIN = 0x01,  // ПК -> ESP: [duration_sec (1B)]
    CMD_DEV_ANNOUNCE = 0x02, // ESP -> ПК: [short (2B)] [ieee (8B)] [capability (1B)]
    CMD_DEV_LEAVE = 0x03,    // ESP -> ПК: [short (2B)] [ieee (8B)]
    CMD_ZDO_BIND_REQ = 0x12, // ПК -> ESP: [dst_short (2B)] [src_ieee (8B)] [src_ep (1B)] [cluster (2B)] [dst_ep (1B)]
    CMD_APS_SEND_RAW = 0x20, // ПК -> ESP: [dst_short (2B)] [src_ep (1B)] [dst_ep (1B)] [profile_id (2B)] [cluster_id (2B)] [tx_opt (1B)] [asdu...]
    CMD_APS_RECV_RAW = 0x21, // ESP -> ПК: [src_short (2B)] [src_ep (1B)] [dst_ep (1B)] [profile_id (2B)] [cluster_id (2B)] [rssi (1B)] [lqi (1B)] [asdu...]
    CMD_APS_DATA_CNF = 0x22, // ESP -> ПК: [dst_short (2B)] [dst_ep (1B)] [cluster_id (2B)] [status (1B)]
    CMD_SYS_PING = 0xF0,     // ПК <-> ESP: [uptime_sec (4B)] [pan_id (2B)] [channel (1B)]
} proto_cmd_t;

typedef struct
{
    uint8_t cmd;
    uint16_t len;
    uint8_t payload[PROTO_MAX_PAYLOAD];
} bridge_msg_t;

typedef enum
{
    STATE_WAIT_SOF0,
    STATE_WAIT_SOF1,
    STATE_READ_HEADER,
    STATE_READ_PAYLOAD,
    STATE_READ_CRC
} parser_state_t;

static QueueHandle_t q_to_pc;
static SemaphoreHandle_t tx_mutex;
static SemaphoreHandle_t ack_sem;

static uint8_t tx_seq_counter = 0;
static uint8_t last_rx_seq = 0xFF;
static volatile uint8_t waiting_ack_seq = 0;
static volatile bool is_first_rx_packet = true;

/* ==================== CRC16 (MODBUS) ==================== */
static uint16_t calc_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            crc = (crc & 0x0001) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
        }
    }
    return crc;
}

/* ==================== НИЗКОУРОВНЕВАЯ ОТПРАВКА В UART0 ==================== */
static void uart_send_raw_frame(uint8_t seq, uint8_t type, uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    uint8_t header[7];
    header[0] = PROTO_SOF_0;
    header[1] = PROTO_SOF_1;
    header[2] = seq;
    header[3] = type;
    header[4] = cmd;
    header[5] = (uint8_t)(len & 0xFF);
    header[6] = (uint8_t)((len >> 8) & 0xFF);

    uint16_t crc = 0xFFFF;
    for (int i = 2; i < 7; i++)
    {
        crc ^= header[i];
        for (int j = 0; j < 8; j++)
        {
            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
        }
    }
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= payload[i];
        for (int j = 0; j < 8; j++)
        {
            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
        }
    }

    uint8_t crc_bytes[2] = {(uint8_t)(crc & 0xFF), (uint8_t)((crc >> 8) & 0xFF)};

    xSemaphoreTake(tx_mutex, portMAX_DELAY);
    uart_write_bytes(PROTO_UART_NUM, (const char *)header, sizeof(header));
    if (len > 0 && payload != NULL)
    {
        uart_write_bytes(PROTO_UART_NUM, (const char *)payload, len);
    }
    uart_write_bytes(PROTO_UART_NUM, (const char *)crc_bytes, sizeof(crc_bytes));
    xSemaphoreGive(tx_mutex);
}

static void bridge_enqueue_to_pc(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    if (len > PROTO_MAX_PAYLOAD)
    {
        ESP_LOGW(TAG, "Payload too large for PC: %u", len);
        return;
    }
    bridge_msg_t msg = {
        .cmd = cmd,
        .len = len,
    };
    if (len > 0 && payload != NULL)
    {
        memcpy(msg.payload, payload, len);
    }
    if (xQueueSend(q_to_pc, &msg, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        ESP_LOGE(TAG, "TX queue to PC full! Dropping CMD=0x%02X", cmd);
    }
}

/* ==================== ЗАДАЧА TX (НАДЕЖНАЯ ОТПРАВКА НА ПК С ACK) ==================== */
static void proto_tx_task(void *arg)
{
    ESP_LOGI(TAG, "proto_tx_task start");

    bridge_msg_t msg;
    while (1)
    {
        if (xQueueReceive(q_to_pc, &msg, portMAX_DELAY) == pdTRUE)
        {
            uint8_t seq = ++tx_seq_counter;
            waiting_ack_seq = seq;
            xSemaphoreTake(ack_sem, 0);

            bool delivered = false;
            for (int attempt = 0; attempt < PROTO_MAX_RETRIES; attempt++)
            {
                ESP_LOGI(TAG, "proto_tx_task uart_send_raw_frame");
                uart_send_raw_frame(seq, PKT_TYPE_DATA, msg.cmd, msg.payload, msg.len);
                if (xSemaphoreTake(ack_sem, pdMS_TO_TICKS(PROTO_ACK_TIMEOUT_MS)) == pdTRUE)
                {
                    delivered = true;
                    break;
                }
                ESP_LOGW(TAG, "ACK timeout SEQ=%u (CMD=0x%02X), retry %d", seq, msg.cmd, attempt + 1);
            }

            if (!delivered)
            {
                ESP_LOGE(TAG, "Failed to deliver SEQ=%u to PC", seq);
            }
        }
    }
}

/* ==================== ИСПОЛНЕНИЕ КОМАНД ОТ ПК В СТЕКЕ EZBEE ==================== */
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
static void on_valid_pc_frame(uint8_t seq, uint8_t type, uint8_t cmd, const uint8_t *payload, uint16_t len)
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

static void proto_rx_task(void *arg)
{
    ESP_LOGI(TAG, "proto_rx_task start");
    parser_state_t state = STATE_WAIT_SOF0;
    uint8_t hdr[5]; // [0]=seq, [1]=type, [2]=cmd, [3]=len_l, [4]=len_h
    uint8_t hdr_idx = 0;
    uint16_t payload_len = 0;
    uint16_t payload_idx = 0;
    uint8_t payload_buf[PROTO_MAX_PAYLOAD];
    uint8_t crc_buf[2];
    uint8_t crc_idx = 0;

    uint8_t rx_chunk[64];
    while (1)
    {
        int n = uart_read_bytes(PROTO_UART_NUM, rx_chunk, sizeof(rx_chunk), pdMS_TO_TICKS(20));
        if (n > 0)
            ESP_LOGI(TAG, "rx_task n=%d", n);
        for (int i = 0; i < n; i++)
        {
            uint8_t b = rx_chunk[i];
            ESP_LOGI(TAG, "rx_task state=%d", state);
            switch (state)
            {
            case STATE_WAIT_SOF0:
                if (b == PROTO_SOF_0)
                    state = STATE_WAIT_SOF1;
                break;

            case STATE_WAIT_SOF1:
                if (b == PROTO_SOF_1)
                {
                    hdr_idx = 0;
                    state = STATE_READ_HEADER;
                }
                else if (b != PROTO_SOF_0)
                {
                    state = STATE_WAIT_SOF0;
                }
                break;

            case STATE_READ_HEADER:
                hdr[hdr_idx++] = b;
                if (hdr_idx == 5)
                {
                    payload_len = hdr[3] | ((uint16_t)hdr[4] << 8);
                    if (payload_len > PROTO_MAX_PAYLOAD)
                    {
                        state = STATE_WAIT_SOF0;
                    }
                    else if (payload_len == 0)
                    {
                        crc_idx = 0;
                        state = STATE_READ_CRC;
                    }
                    else
                    {
                        payload_idx = 0;
                        state = STATE_READ_PAYLOAD;
                    }
                }
                break;

            case STATE_READ_PAYLOAD:
                payload_buf[payload_idx++] = b;
                if (payload_idx == payload_len)
                {
                    crc_idx = 0;
                    state = STATE_READ_CRC;
                }
                break;

            case STATE_READ_CRC:
                crc_buf[crc_idx++] = b;
                if (crc_idx == 2)
                {
                    uint16_t rx_crc = crc_buf[0] | ((uint16_t)crc_buf[1] << 8);
                    uint8_t check_buf[5 + PROTO_MAX_PAYLOAD];
                    memcpy(check_buf, hdr, 5);
                    if (payload_len > 0)
                    {
                        memcpy(check_buf + 5, payload_buf, payload_len);
                    }

                    if (calc_crc16(check_buf, 5 + payload_len) == rx_crc)
                    {
                        on_valid_pc_frame(hdr[0], hdr[1], hdr[2], payload_buf, payload_len);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "CRC mismatch on SEQ=%u", hdr[0]);
                        uart_send_raw_frame(hdr[0], PKT_TYPE_NACK, hdr[2], NULL, 0);
                    }
                    state = STATE_WAIT_SOF0;
                }
                break;
            }
        }
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

static void esp_zb_task(void *pvParameters)
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

/* ==================== ТОЧКА ВХОДА ==================== */
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    q_to_pc = xQueueCreate(32, sizeof(bridge_msg_t));
    tx_mutex = xSemaphoreCreateMutex();
    ack_sem = xSemaphoreCreateBinary();

    const uart_config_t uart_config = {
        .baud_rate = PROTO_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(PROTO_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(PROTO_UART_NUM, 24, 23, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(PROTO_UART_NUM, 2048, 2048, 0, NULL, 0));

    xTaskCreate(proto_rx_task, "proto_rx", 4096, NULL, 10, NULL);
    xTaskCreate(proto_tx_task, "proto_tx", 4096, NULL, 9, NULL);
    xTaskCreate(esp_zb_task, "zb_main", 8192, NULL, 5, NULL);
}

#endif