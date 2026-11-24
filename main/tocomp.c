#include "coordinator.h"

uint8_t tx_seq_counter = 0;
QueueHandle_t q_to_pc;
SemaphoreHandle_t tx_mutex;
SemaphoreHandle_t ack_sem;
volatile uint8_t waiting_ack_seq = 0;

extern const char *TAG;

 
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
/*
// Отправка любого пакета в формате вашего Go-кода
void znp_send_frame(uint16_t cmd_id, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[260];
    uint8_t *p = frame;

    uint8_t cmd_hi = (uint8_t)((cmd_id >> 8) & 0xFF);
    uint8_t cmd_lo = (uint8_t)(cmd_id & 0xFF);

    *p++ = ZNP_SOF;
    *p++ = len;
    *p++ = cmd_hi;
    *p++ = cmd_lo;

    uint8_t fcs = len ^ cmd_hi ^ cmd_lo;

    for (uint8_t i = 0; i < len; i++) {
        uint8_t b = *(payload + i);
        *p++ = b;
        fcs ^= b;
    }
    *p++ = fcs;

    uart_write_bytes(PROTO_UART_NUM, (const char *)frame, (size_t)(p - frame));
}
*/
// Отправка SYS_RESET_IND (0x4180) при запуске координатора
void znp_send_sys_reset_ind(void)
{
    uint8_t payload[6] = {
        0x00, // Reason: Power-up
        0x02, // TransportRev: 2
        0x08, // ProductID: 8
        0x09, // MajorRel: 9
        0x00, // MinorRel: 0
        0x0C  // MaintRel: 13
    };
    bridge_enqueue_to_pc(0x4180, payload, sizeof(payload));
}
/*
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
*/
void bridge_enqueue_to_pc(uint16_t cmd, const uint8_t *payload, uint16_t len)
{
    if (len > PROTO_MAX_PAYLOAD)
    {
        ESP_LOGW(TAG, "Payload too large for PC: %u", len);
        return;
    }
    bridge_msg_t msg = {
        .cmd = cmd,
        .len = (uint8_t)len,
    };
    if (len > 0 && payload != NULL)
    {
        memcpy(msg.payload, payload, len);
    }
    if (xQueueSend(q_to_pc, &msg, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        ESP_LOGE(TAG, "TX queue to PC full! Dropping CMD=0x%04X", cmd);
    }
}

// Постановка любого ZNP-пакета в очередь отправки на ПК
// дубликат, найти и заменить вызовы
void znp_send_frame(uint16_t cmd_id, const uint8_t *payload, uint8_t len)
{
   bridge_enqueue_to_pc(cmd_id, payload, len);
}
// Низкоуровневая сборка и отправка кадра 0xFE в UART
// SOF Length Cmd0 Cmd1 Payload FCS
static void znp_write_uart(uint16_t cmd_id, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[260];
    uint8_t *p = frame;

    uint8_t cmd_hi = (uint8_t)((cmd_id >> 8) & 0xFF);
    uint8_t cmd_lo = (uint8_t)(cmd_id & 0xFF);

    *p++ = ZNP_SOF;
    *p++ = len;
    *p++ = cmd_hi;
    *p++ = cmd_lo;

    uint8_t fcs = len ^ cmd_hi ^ cmd_lo;
    for (uint8_t i = 0; i < len; i++)
    {
        uint8_t b = *(payload + i);
        *p++ = b;
        fcs ^= b;
    }
    *p++ = fcs;

    xSemaphoreTake(tx_mutex, portMAX_DELAY);
    uart_write_bytes(PROTO_UART_NUM, (const char *)frame, (size_t)(p - frame));
    xSemaphoreGive(tx_mutex);
}

// Задача TX: забирает из очереди и отправляет в ПК без ретраев и ACK
void proto_tx_task(void *arg)
{
    ESP_LOGI(TAG, "proto_tx_task (ZNP) start");

    bridge_msg_t msg;
    while (1)
    {
        if (xQueueReceive(q_to_pc, &msg, portMAX_DELAY) == pdTRUE)
        {
            znp_write_uart(msg.cmd, msg.payload, msg.len);
            // Небольшая пауза 2-5 мс, чтобы USB-UART драйвер macOS не склеивал пакеты
            vTaskDelay(pdMS_TO_TICKS(3));
        }
    }
}