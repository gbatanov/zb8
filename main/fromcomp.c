#include "coordinator.h"

extern const char *TAG;

void proto_rx_task(void *arg)
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
