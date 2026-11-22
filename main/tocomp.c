#include "coordinator.h"

uint8_t tx_seq_counter = 0;
QueueHandle_t q_to_pc;
SemaphoreHandle_t tx_mutex;
SemaphoreHandle_t ack_sem;
volatile uint8_t waiting_ack_seq = 0;

extern const char *TAG;

/* ==================== CRC16 (MODBUS) ==================== */
uint16_t calc_crc16(const uint8_t *data, size_t len)
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
void uart_send_raw_frame(uint8_t seq, uint8_t type, uint8_t cmd, const uint8_t *payload, uint16_t len)
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

void bridge_enqueue_to_pc(uint8_t cmd, const uint8_t *payload, uint16_t len)
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
void proto_tx_task(void *arg)
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
