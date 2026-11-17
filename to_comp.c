#include "driver/usb_serial_jtag.h"

// Простой бинарный заголовок фрейма (передается в обрамлении COBS или с маркером старта)
typedef struct __attribute__((packed)) {
    uint8_t  sof;        // Маркер начала: 0xAA
    uint8_t  cmd;        // 0x01 = Отчет атрибута, 0x02 = Новый девайс, 0x10 = Permit Join
    uint16_t short_addr; // Сетевой адрес устройства
    uint8_t  endpoint;   // Эндпоинт
    uint16_t cluster_id; // ID кластера (например, 0x0006 On/Off, 0x0402 Temp)
    uint16_t attr_id;    // ID атрибута
    uint8_t  data_type;  // Тип данных ZCL
    uint8_t  len;        // Длина значения
    uint8_t  payload[32];// Значение атрибута
} host_frame_t;

void usb_transport_init(void) {
    usb_serial_jtag_driver_config_t cfg = {
        .rx_buffer_size = 1024,
        .tx_buffer_size = 1024,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));
}

// Пример отправки отчета от датчика в Linux
static void zb_action_handler(ezb_zcl_core_action_callback_id_t cb_id, const void *message) {
    if (cb_id == EZB_ZCL_CORE_REPORT_ATTR_CB_ID) {
        const ezb_zcl_cmd_report_attr_message_t *rep = (const ezb_zcl_cmd_report_attr_message_t *)message;
        for (ezb_zcl_report_attr_variable_t *v = rep->variables; v != NULL; v = v->next) {
            host_frame_t frame = {
                .sof        = 0xAA,
                .cmd        = 0x01,
                .short_addr = rep->info.src_addr.u.short_addr,
                .endpoint   = rep->info.src_ep,
                .cluster_id = rep->info.cluster_id,
                .attr_id    = v->id,
                .data_type  = v->data_type,
                .len        = v->data_size > 32 ? 32 : v->data_size,
            };
            memcpy(frame.payload, v->data_p, frame.len);
            usb_serial_jtag_write_bytes((const char *)&frame, sizeof(host_frame_t), pdMS_TO_TICKS(50));
        }
    }
}

// Задача чтения команд от ПК (например, открыть сеть на 60 сек)
void usb_rx_task(void *arg) {
    uint8_t buf[64];
    while (1) {
        int len = usb_serial_jtag_read_bytes(buf, sizeof(buf), portMAX_DELAY);
        if (len >= 2 && buf[0] == 0xAA) {
            if (buf[1] == 0x10) { // Команда Permit Join
                uint8_t duration = (len >= 3) ? buf[2] : 60;
                esp_zigbee_lock_acquire(portMAX_DELAY);
                ezb_bdb_open_network(duration);
                esp_zigbee_lock_release();
            }
        }
    }
}
