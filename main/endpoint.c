#include "esp_err.h"
#include <stddef.h>
#include "ezbee/af.h"
#include "ezbee/zcl.h"
#include "ezbee/zha.h" // <-- Здесь объявлены ID устройств профиля ZHA

#define COORDINATOR_ENDPOINT 1

// Вызывайте эту функцию после ezb_config_memory() и до esp_zigbee_start().
void coordinator_register_endpoint(void) {
    // 1. Создаем корневой дескриптор устройства
    ezb_af_device_desc_t dev_desc = ezb_af_create_device_desc();

    // 2. Настраиваем конфигурацию эндпоинта (профиль Home Automation)
    ezb_af_ep_config_t ep_cfg = {
        .ep_id = COORDINATOR_ENDPOINT,
        .app_profile_id = EZB_AF_HA_PROFILE_ID,
        .app_device_id = EZB_ZHA_COMBINED_INTERFACE_DEVICE_ID, // Универсальный шлюз/интерфейс
        .app_device_version = 0,
    };
    ezb_af_ep_desc_t ep_desc = ezb_af_create_endpoint_desc(&ep_cfg);

    // 3. Добавляем обязательные SERVER-кластеры самого координатора (Basic и Identify)
    ezb_zcl_basic_cluster_config_t basic_cfg = {
        .zcl_version = EZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = EZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
    };
    ezb_af_endpoint_add_cluster_desc(ep_desc,
        ezb_zcl_basic_create_cluster_desc(&basic_cfg, EZB_ZCL_CLUSTER_SERVER));

    ezb_zcl_identify_cluster_config_t identify_cfg = {
        .identify_time = 0,
    };
    ezb_af_endpoint_add_cluster_desc(ep_desc,
        ezb_zcl_identify_create_cluster_desc(&identify_cfg, EZB_ZCL_CLUSTER_SERVER));

    // 4. Добавляем CLIENT-кластеры для приема отчетов и управления устройствами
    // Список стандартных кластеров (датчики, реле, свет, протечка/движение IAS Zone, батарея)
    const uint16_t client_clusters[] = {
        EZB_ZCL_CLUSTER_ID_BASIC,                 // 0x0000: Чтение модели и производителя
        EZB_ZCL_CLUSTER_ID_POWER_CONFIG,          // 0x0001: Заряд батареи датчиков
        EZB_ZCL_CLUSTER_ID_ON_OFF,                // 0x0006: Реле, розетки, кнопки
        EZB_ZCL_CLUSTER_ID_LEVEL,                 // 0x0008: Диммеры, шторы
        EZB_ZCL_CLUSTER_ID_MULTISTATE_INPUT,      // 0x0012: Мультиклики кнопок (Aqara и др.)
        EZB_ZCL_CLUSTER_ID_ANALOG_INPUT,          // 0x000C: Аналоговые значения (мощность и т.д.)
        EZB_ZCL_CLUSTER_ID_COLOR_CONTROL,         // 0x0300: RGB/CCT лампы
        EZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT, // 0x0400: Датчики освещенности
        EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,  // 0x0402: Температура
        EZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,  // 0x0403: Атмосферное давление
        EZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, // 0x0405: Влажность
        EZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,     // 0x0406: Датчики присутствия/движения
        EZB_ZCL_CLUSTER_ID_IAS_ZONE,              // 0x0500: Герконы, протечка, дым, движение
        EZB_ZCL_CLUSTER_ID_METERING,              // 0x0702: Счетчики энергии (кВт*ч)
        EZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT // 0x0B04: Напряжение, ток, ватты (В, А, Вт)
    };

    for (size_t i = 0; i < sizeof(client_clusters) / sizeof(client_clusters[0]); i++) {
        ezb_zcl_custom_cluster_config_t custom_cfg = {
            .cluster_id = client_clusters[i],
        };
        ezb_zcl_cluster_desc_t cl_desc = ezb_zcl_custom_create_cluster_desc(
            &custom_cfg, EZB_ZCL_CLUSTER_CLIENT);
        ezb_af_endpoint_add_cluster_desc(ep_desc, cl_desc);
    }

    // 5. Привязываем эндпоинт к устройству и регистрируем в стеке
    ezb_af_device_add_endpoint_desc(dev_desc, ep_desc);
    ESP_ERROR_CHECK(ezb_af_device_desc_register(dev_desc));
}
