#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"

extern const char *TAG;

#define PROTO_SOF_0         0xAA
#define PROTO_SOF_1         0x55
#define PROTO_MAX_PAYLOAD   256

typedef enum {
    PKT_TYPE_DATA = 0x01,
    PKT_TYPE_ACK  = 0x02,
    PKT_TYPE_NACK = 0x03,
} proto_pkt_type_t;

// Примеры команд для связки с Zigbee
typedef enum {
    CMD_ZB_ATTR_REPORT  = 0x10, // ESP -> ПК: пришел отчет от датчика/реле Aqara
    CMD_ZB_SEND_ON_OFF  = 0x20, // ПК -> ESP: включить/выключить реле
    CMD_ZB_RAW_ZCL      = 0x21, // ПК -> ESP: отправить произвольный ZCL кадр
    CMD_SYS_PING        = 0xF0, // Проверка связи
} proto_cmd_t;

// Состояния парсера
typedef enum {
    STATE_WAIT_SOF0,
    STATE_WAIT_SOF1,
    STATE_READ_HEADER,
    STATE_READ_PAYLOAD,
    STATE_READ_CRC
} parser_state_t;

static uint8_t tx_seq_counter = 0;
static uint8_t last_processed_rx_seq = 0xFF;
static SemaphoreHandle_t tx_mutex;

// Событие получения ACK от ПК
static volatile uint8_t waiting_ack_seq = 0;
static SemaphoreHandle_t ack_sem;

// Расчет CRC16 (Modbus)
static uint16_t calc_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

// Базовая отправка кадра в UART (замените uart_write_bytes на usb_serial_jtag_write_bytes при необходимости)
static void send_raw_frame(uint8_t seq, uint8_t type, uint8_t cmd, const uint8_t *payload, uint16_t len) {
    uint8_t header[7];
    header[0] = PROTO_SOF_0;
    header[1] = PROTO_SOF_1;
    header[2] = seq;
    header[3] = type;
    header[4] = cmd;
    header[5] = len & 0xFF;
    header[6] = (len >> 8) & 0xFF;

    // Считаем CRC по полям SEQ..LEN и PAYLOAD
    uint16_t crc = 0xFFFF;
    for (int i = 2; i < 7; i++) {
        crc ^= header[i];
        for (int j = 0; j < 8; j++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    for (uint16_t i = 0; i < len; i++) {
        crc ^= payload[i];
        for (int j = 0; j < 8; j++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }

    uint8_t crc_bytes[2] = { crc & 0xFF, (crc >> 8) & 0xFF };

    xSemaphoreTake(tx_mutex, portMAX_DELAY);
    uart_write_bytes(UART_NUM_0, (const char *)header, sizeof(header));
    if (len > 0 && payload != NULL) {
        uart_write_bytes(UART_NUM_0, (const char *)payload, len);
    }
    uart_write_bytes(UART_NUM_0, (const char *)crc_bytes, sizeof(crc_bytes));
    xSemaphoreGive(tx_mutex);
}

// Надежная отправка данных с ожиданием ACK и ретрансляцией
bool proto_send_reliable(uint8_t cmd, const uint8_t *payload, uint16_t len) {
    uint8_t seq = ++tx_seq_counter;
    waiting_ack_seq = seq;
    xSemaphoreTake(ack_sem, 0); // Сброс семафора

    for (int attempt = 0; attempt < 3; attempt++) {
        send_raw_frame(seq, PKT_TYPE_DATA, cmd, payload, len);
        if (xSemaphoreTake(ack_sem, pdMS_TO_TICKS(150)) == pdTRUE) {
            return true; // ACK получен!
        }
        ESP_LOGW(TAG, "Timeout waiting ACK for seq %d, retry %d", seq, attempt + 1);
    }
    ESP_LOGE(TAG, "Failed to deliver seq %d", seq);
    return false;
}

// Обработчик входящего валидного пакета от ПК
static void on_valid_packet_received(uint8_t seq, uint8_t type, uint8_t cmd, const uint8_t *payload, uint16_t len) {
    if (type == PKT_TYPE_ACK) {
        if (seq == waiting_ack_seq) {
            xSemaphoreGive(ack_sem);
        }
        return;
    }

    if (type == PKT_TYPE_DATA) {
        // Всегда отвечаем ACK на валидный пакет данных
        send_raw_frame(seq, PKT_TYPE_ACK, cmd, NULL, 0);

        // Проверка на дубликат пакета
        if (seq == last_processed_rx_seq) {
            ESP_LOGW(TAG, "Duplicate packet seq %d ignored", seq);
            return;
        }
        last_processed_rx_seq = seq;

        // Передаем команду в Zigbee-стек (в реальном коде здесь лучше положить в QueueHandle_t)
        switch (cmd) {
            case CMD_ZB_SEND_ON_OFF:
                // payload: [short_addr_L, short_addr_H, endpoint, on_off_state]
                if (len >= 4) {
                    uint16_t short_addr = payload[0] | (payload[1] << 8);
                    uint8_t ep = payload[2];
                    uint8_t state = payload[3];
                    ESP_LOGI(TAG, "CMD: Set relay 0x%04X EP:%d -> %d", short_addr, ep, state);
                    // TODO: Вызов esp_zb_zcl_on_off_cmd_req(...) через esp_zb_scheduler_alarm
                }
                break;
            case CMD_SYS_PING:
                ESP_LOGI(TAG, "PING from PC");
                break;
            default:
                ESP_LOGW(TAG, "Unknown CMD: 0x%02X", cmd);
                break;
        }
    }
}

// Задача разбора входящего потока (FSM)
void proto_rx_task(void *arg) {
    parser_state_t state = STATE_WAIT_SOF0;
    uint8_t hdr[5]; // seq, type, cmd, len_l, len_h
    uint8_t hdr_idx = 0;
    uint16_t payload_len = 0;
    uint16_t payload_idx = 0;
    uint8_t payload_buf[PROTO_MAX_PAYLOAD];
    uint8_t crc_buf[2];
    uint8_t crc_idx = 0;

    uint8_t byte;
    while (1) {
        int read = uart_read_bytes(UART_NUM_0, &byte, 1, portMAX_DELAY);
        if (read <= 0) continue;

        switch (state) {
            case STATE_WAIT_SOF0:
                if (byte == PROTO_SOF_0) state = STATE_WAIT_SOF1;
                break;

            case STATE_WAIT_SOF1:
                if (byte == PROTO_SOF_1) {
                    hdr_idx = 0;
                    state = STATE_READ_HEADER;
                } else if (byte != PROTO_SOF_0) {
                    state = STATE_WAIT_SOF0;
                }
                break;

            case STATE_READ_HEADER:
                hdr[hdr_idx++] = byte;
                if (hdr_idx == 5) {
                    payload_len = hdr[3] | (hdr[4] << 8);
                    if (payload_len > PROTO_MAX_PAYLOAD) {
                        state = STATE_WAIT_SOF0; // Защита от переполнения
                    } else if (payload_len == 0) {
                        crc_idx = 0;
                        state = STATE_READ_CRC;
                    } else {
                        payload_idx = 0;
                        state = STATE_READ_PAYLOAD;
                    }
                }
                break;

            case STATE_READ_PAYLOAD:
                payload_buf[payload_idx++] = byte;
                if (payload_idx == payload_len) {
                    crc_idx = 0;
                    state = STATE_READ_CRC;
                }
                break;

            case STATE_READ_CRC:
                crc_buf[crc_idx++] = byte;
                if (crc_idx == 2) {
                    uint16_t rx_crc = crc_buf[0] | (crc_buf[1] << 8);
                    // Проверяем CRC по hdr (5 байт) + payload
                    uint8_t check_buf[5 + PROTO_MAX_PAYLOAD];
                    memcpy(check_buf, hdr, 5);
                    if (payload_len > 0) memcpy(check_buf + 5, payload_buf, payload_len);

                    if (calc_crc16(check_buf, 5 + payload_len) == rx_crc) {
                        on_valid_packet_received(hdr[0], hdr[1], hdr[2], payload_buf, payload_len);
                    } else {
                        ESP_LOGE(TAG, "CRC mismatch on seq %d!", hdr[0]);
                        send_raw_frame(hdr[0], PKT_TYPE_NACK, hdr[2], NULL, 0);
                    }
                    state = STATE_WAIT_SOF0;
                }
                break;
        }
    }
}
