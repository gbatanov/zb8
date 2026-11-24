#include "coordinator.h"

extern const char *TAG;
extern volatile uint8_t last_af_trans_id;
//  Конечный автомат приёма кадров из UART0
typedef enum
{
    ZNP_STATE_SOF = 0,
    ZNP_STATE_LEN,
    ZNP_STATE_CMD0,
    ZNP_STATE_CMD1,
    ZNP_STATE_DATA,
    ZNP_STATE_FCS
} znp_rx_state_t;

// Обработчик входящих команд от  Go-программы
static void znp_handle_command(uint16_t cmd_id, const uint8_t *payload, uint8_t len)
{
    ESP_LOGI(TAG, "RX ZNP cmd: 0x%04X, len=%u", cmd_id, len);

    switch (cmd_id)
    {
    case 0x4100: // SYS_RESET_REQ
        ESP_LOGW(TAG, "Rebooting by SYS_RESET_REQ...");
        vTaskDelay(pdMS_TO_TICKS(50));
        esp_restart();
        break;

    case 0x2101: // SYS_PING (пример синхронного ответа SRSP = 0x6101)
    {
        uint8_t caps[2] = {0x79, 0x07}; // Capabilities (MT_CAP_SYS | MT_CAP_ZDO | MT_CAP_AF)
        znp_send_frame(0x6101, caps, sizeof(caps));
        break;
    }
    case ZDO_MGMT_PERMIT_JOIN_REQ:
    {
        uint8_t duration = 60; // значение по умолчанию, если len < 4
        if (len >= 4)
        {
            duration = *(payload + 3);
        }

        ESP_LOGI(TAG, "ZDO_MGMT_PERMIT_JOIN_REQ: duration=%u sec", duration);

        // 1. Открываем (или закрываем при duration == 0) сеть в стеке ESP-Zigbee
        esp_zigbee_lock_acquire(portMAX_DELAY);
        esp_err_t err = ezb_bdb_open_network(duration);
        esp_zigbee_lock_release();

        uint8_t status = (err == ESP_OK) ? 0x00 : 0x01;

        // 1. Ставим в очередь синхронный ответ SRSP (0x6536)
        bridge_enqueue_to_pc(0x6536, &status, 1);

        // 2. Ставим в очередь асинхронное подтверждение ZDO_MGMT_PERMIT_JOIN_RSP (0x45B6)
        //        uint8_t rsp_payload[] = {0x00, 0x00, status};
        //       bridge_enqueue_to_pc(0x45B6, rsp_payload, sizeof(rsp_payload));
        break;
    }
    case ZDO_BIND_REQ: // 0x2521 ZDO_BIND_REQ -> SRSP 0x6521
    {
        uint8_t status = 0x02;

        if (len >= 16)
        {
            ezb_zdo_bind_req_t bind_req = {0};

            bind_req.dst_nwk_addr = (uint16_t)(*(payload + 0)) | ((uint16_t)(*(payload + 1)) << 8);
            memcpy(bind_req.field.src_addr.u8, payload + 2, 8);
            bind_req.field.src_ep = *(payload + 10);
            bind_req.field.cluster_id = (uint16_t)(*(payload + 11)) | ((uint16_t)(*(payload + 12)) << 8);
            uint8_t addr_mode = *(payload + 13);

            if (addr_mode == 0x03 && len >= 23)
            {
                 memcpy(&bind_req.field.dst_addr, payload + 14, 8);
                bind_req.field.dst_ep = *(payload + 22);
                status = 0x00;
            }
            else if (addr_mode == 0x01 && len >= 16)
            {
            bind_req.field.dst_addr.group_addr.group =
                (uint16_t)(*(payload + 14)) | ((uint16_t)(*(payload + 15)) << 8);
            status = 0x00;
            }

            if (status == 0x00)
            {
                esp_zigbee_lock_acquire(portMAX_DELAY);
                esp_err_t err = ezb_zdo_bind_req(&bind_req);
                esp_zigbee_lock_release();
                if (err != ESP_OK)
                {
                    status = 0x01;
                }
            }
        }

        bridge_enqueue_to_pc(0x6521, &status, 1);
        break;
    }
    case 0x2505: // ZDO_ACTIVE_EP_REQ -> SRSP 0x6505
    {
        uint8_t status = 0x02;
        if (len >= 4)
        {
            ezb_zdo_active_ep_req_t req = {0};
            req.dst_nwk_addr = (uint16_t)(*(payload + 0)) | ((uint16_t)(*(payload + 1)) << 8);
            req.field.nwk_addr_of_interest = (uint16_t)(*(payload + 2)) | ((uint16_t)(*(payload + 3)) << 8);

            esp_zigbee_lock_acquire(portMAX_DELAY);
            esp_err_t err = ezb_zdo_active_ep_req(&req);
            esp_zigbee_lock_release();
            status = (err == ESP_OK) ? 0x00 : 0x01;
        }
        bridge_enqueue_to_pc(0x6505, &status, 1);
        break;
    }

    case 0x2504: // ZDO_SIMPLE_DESC_REQ -> SRSP 0x6504
    {
        uint8_t status = 0x02;
        if (len >= 5)
        {
            ezb_zdo_simple_desc_req_t req = {0};
            req.dst_nwk_addr = (uint16_t)(*(payload + 0)) | ((uint16_t)(*(payload + 1)) << 8);
            req.field.nwk_addr_of_interest = (uint16_t)(*(payload + 2)) | ((uint16_t)(*(payload + 3)) << 8);
            req.field.endpoint = *(payload + 4);

            esp_zigbee_lock_acquire(portMAX_DELAY);
            esp_err_t err = ezb_zdo_simple_desc_req(&req);
            esp_zigbee_lock_release();
            status = (err == ESP_OK) ? 0x00 : 0x01;
        }
        bridge_enqueue_to_pc(0x6504, &status, 1);
        break;
    }

    case 0x2502: // ZDO_NODE_DESC_REQ -> SRSP 0x6502
    {
        uint8_t status = 0x02;
        if (len >= 4)
        {
            ezb_zdo_node_desc_req_t req = {0};
            req.dst_nwk_addr = (uint16_t)(*(payload + 0)) | ((uint16_t)(*(payload + 1)) << 8);
            req.field.nwk_addr_of_interest = (uint16_t)(*(payload + 2)) | ((uint16_t)(*(payload + 3)) << 8);

            esp_zigbee_lock_acquire(portMAX_DELAY);
            esp_err_t err = ezb_zdo_node_desc_req(&req);
            esp_zigbee_lock_release();
            status = (err == ESP_OK) ? 0x00 : 0x01;
        }
        bridge_enqueue_to_pc(0x6502, &status, 1);
        break;
    }

    case 0x2401: // AF_DATA_REQUEST -> SRSP 0x6401 (+ AREQ 0x4480 из bridge_aps_data_cnf_cb)
    {
        uint8_t status = 0x02;

        if (len >= 10)
        {
            uint16_t dst_addr = (uint16_t)(*(payload + 0)) | ((uint16_t)(*(payload + 1)) << 8);
            uint8_t dst_ep = *(payload + 2);
            uint8_t src_ep = *(payload + 3);
            uint16_t cluster_id = (uint16_t)(*(payload + 4)) | ((uint16_t)(*(payload + 5)) << 8);
            uint8_t trans_id = *(payload + 6);
            uint8_t radius = *(payload + 8);
            uint8_t data_len = *(payload + 9);

            if (len >= (uint16_t)(10 + data_len))
            {
                last_af_trans_id = trans_id;

                ezb_apsde_data_req_t req = {0};
                req.dst_address.addr_mode = EZB_ADDR_MODE_SHORT;
                req.dst_address.u.short_addr = dst_addr;
                req.dst_endpoint = dst_ep;
                req.src_endpoint = src_ep;
                req.profile_id = 0x0104; // Home Automation
                req.cluster_id = cluster_id;
                req.asdu_length = data_len;
                req.asdu = (data_len > 0) ? (uint8_t *)(payload + 10) : NULL;
                req.radius = radius;
                req.tx_options = EZB_APSDE_TX_OPT_ACK_TX;

                esp_zigbee_lock_acquire(portMAX_DELAY);
                esp_err_t err = ezb_apsde_data_request(&req);
                esp_zigbee_lock_release();

                status = (err == ESP_OK) ? 0x00 : 0x01;
            }
        }

        bridge_enqueue_to_pc(0x6401, &status, 1);
        break;
    }

    default:
        ESP_LOGW(TAG, "Unhandled command: 0x%04X", cmd_id);
        for (uint8_t i = 0; i < len; i++)
        {
            ESP_LOGI(TAG, "%02X ", payload[i]);
        }
        break;
    }
}

void proto_rx_task(void *arg)
{
    ESP_LOGI(TAG, "proto_rx_task (ZNP 0xFE) started");

    znp_rx_state_t state = ZNP_STATE_SOF;
    uint8_t data_len = 0;
    uint8_t cmd0 = 0;
    uint8_t cmd1 = 0;
    uint8_t data_idx = 0;
    uint8_t fcs_calc = 0;
    uint8_t payload_buf[ZNP_MAX_DATA];
    uint8_t rx_chunk[64];

    // Небольшая пауза и отправка SYS_RESET_IND при старте
    vTaskDelay(pdMS_TO_TICKS(200));
    znp_send_sys_reset_ind();

    while (1)
    {
        int n = uart_read_bytes(PROTO_UART_NUM, rx_chunk, sizeof(rx_chunk), pdMS_TO_TICKS(20));
        for (int i = 0; i < n; i++)
        {
            uint8_t b = *(rx_chunk + i);

            switch (state)
            {
            case ZNP_STATE_SOF:
                if (b == ZNP_SOF)
                {
                    state = ZNP_STATE_LEN;
                }
                break;

            case ZNP_STATE_LEN:
                data_len = b;
                fcs_calc = b;
                state = ZNP_STATE_CMD0;
                break;

            case ZNP_STATE_CMD0:
                cmd0 = b;
                fcs_calc ^= b;
                state = ZNP_STATE_CMD1;
                break;

            case ZNP_STATE_CMD1:
                cmd1 = b;
                fcs_calc ^= b;
                data_idx = 0;
                state = (data_len > 0) ? ZNP_STATE_DATA : ZNP_STATE_FCS;
                break;

            case ZNP_STATE_DATA:
                *(payload_buf + data_idx) = b;
                data_idx++;
                fcs_calc ^= b;
                if (data_idx >= data_len)
                {
                    state = ZNP_STATE_FCS;
                }
                break;

            case ZNP_STATE_FCS:
                if (b == fcs_calc)
                {
                    uint16_t cmd_id = ((uint16_t)cmd0 << 8) | (uint16_t)cmd1;
                    znp_handle_command(cmd_id, payload_buf, data_len);
                }
                else
                {
                    ESP_LOGE(TAG, "FCS mismatch: got 0x%02X, expected 0x%02X", b, fcs_calc);
                }
                state = ZNP_STATE_SOF;
                break;
            }
        }
    }
}
