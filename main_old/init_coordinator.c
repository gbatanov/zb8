#include "esp_zigbee.h"
#include "ezbee/core.h"
#include "ezbee/bdb.h"
#include "ezbee/zcl.h"
#include "nvs_flash.h"

void zigbee_init_coordinator(void) {
    // 1. Инициализация NVS для хранения сети и ключей
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nvs_flash_init_partition("zb_storage"));

    // 2. Конфигурация роли координатора
    esp_zigbee_config_t zb_cfg = {
        .device_config = {
            .device_type = EZB_NWK_DEVICE_TYPE_COORDINATOR,
            .install_code_policy = false,
            .zczr_config = {
                .max_children = 32, // Прямые потомки (остальные пойдут через роутеры)
            },
        },
        .platform_config = {
            .radio_config = { .radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE },
        },
    };
    ESP_ERROR_CHECK(esp_zigbee_init(&zb_cfg));

    // 3. Расширение таблиц памяти под сеть 100+ устройств
    ezb_mem_config_t mem_cfg = {
        .buffer_pool_size           = 120, // Пул пакетов (по умолчанию ~64-80)
        .address_table_size         = 128, // Сопоставление IEEE (64-bit) <-> Short (16-bit)
        .neighbor_table_size        = 48,  // Прямые соседи и роутеры вокруг
        .route_table_size           = 100, // Таблица маршрутизации
        .route_discovery_table_size = 32,  // Одновременный поиск маршрутов
        .route_record_table_size    = 64,  // Записи маршрутов (Many-to-One)
        .aps_key_pair_set_size      = 128, // Ключи безопасности TCLK для каждого устройства
        .aps_bind_table_src_size    = 64,  // Таблица привязок (источники)
        .aps_bind_table_dst_size    = 64,  // Таблица привязок (получатели)
        .aps_group_table_size       = 32,  // Группы
    };
    ESP_ERROR_CHECK(ezb_config_memory(&mem_cfg));

    // 4. Регистрация обработчиков событий и ZCL-отчетов
    ezb_app_signal_add_handler(zb_app_signal_handler);
    ezb_zcl_core_action_handler_register(zb_action_handler);

    // 5. Запуск сети (например, на 15-м канале)
    ezb_set_channel_mask(1 << 15);
    ESP_ERROR_CHECK(esp_zigbee_start(true));
    esp_zigbee_launch_mainloop();
}
