#pragma once

#include "esp_zigbee.h"
// #include "esp_zigbee_console.h"

#define MAX_CHILDREN 32                 // the max amount of connected devices
#define INSTALLCODE_POLICY_ENABLE false // enable the install code policy for security

#define ESP_ZIGBEE_STORAGE_PARTITION_NAME "nvs"
// каналы с 11 по 26, 11 - 0x00000800, 15 - 0x00008000, 11+15 - 0x00008800
// #define ESP_ZIGBEE_ALL_CHANNELS_MASK (0x07FFF800U)
// для теста берем с 16 по 26 каналы
#define ESP_ZIGBEE_ALL_CHANNELS_MASK (0x00008000U)

#define ESP_ZIGBEE_ZR_CONFIG()                            \
    {                                                     \
        .device_type = EZB_NWK_DEVICE_TYPE_COORDINATOR,   \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE, \
        .zczr_config = {                                  \
            .max_children = MAX_CHILDREN,                 \
        },                                                \
    }

#if CONFIG_SOC_IEEE802154_SUPPORTED
#define ESP_ZIGBEE_DEFAULT_RADIO_CONFIG()           \
    {                                               \
        .radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE, \
    }
#else
#error "CLI example for Gateway is not ready!"
#endif

#define ESP_ZIGBEE_DEFAULT_PLATFORM_CONFIG()                         \
    {                                                                \
        .storage_partition_name = ESP_ZIGBEE_STORAGE_PARTITION_NAME, \
        .radio_config = ESP_ZIGBEE_DEFAULT_RADIO_CONFIG(),           \
    }

#define ESP_ZIGBEE_DEFAULT_CONFIG()                              \
    {                                                            \
        .platform_config = ESP_ZIGBEE_DEFAULT_PLATFORM_CONFIG(), \
        .device_config = ESP_ZIGBEE_ZR_CONFIG(),                 \
    }

void coordinator_register_endpoint(void);
void permit_join(uint8_t duration_sec, bool with_mutex );
void permit_join_via_router(uint16_t router_short_addr, uint8_t duration_sec);
void test_send_active_ep(uint16_t short_addr);
