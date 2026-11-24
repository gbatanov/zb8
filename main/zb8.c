#include "coordinator.h"

extern SemaphoreHandle_t ack_sem;
extern QueueHandle_t q_to_pc;
extern SemaphoreHandle_t tx_mutex;
extern volatile uint8_t waiting_ack_seq;

const char *TAG = "GSB_ZB8_NCP";

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

