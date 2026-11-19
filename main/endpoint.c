#include "esp_err.h"
#include <stddef.h>
#include "ezbee/af.h"
#include "ezbee/zcl.h"
#include "ezbee/zha.h" // <-- Здесь объявлены ID устройств профиля ZHA

#define COORDINATOR_ENDPOINT 1

// Вызывайте эту функцию после ezb_config_memory() и до esp_zigbee_start().
void coordinator_register_endpoint(void)
{
    ezb_af_device_desc_t dev_desc = ezb_af_create_device_desc();

    ezb_af_ep_config_t ep_cfg = {
        .ep_id = 1,
        .app_profile_id = 0x0104, // Zigbee Home Automation (HA)
        .app_device_id = 0x0050,  // Home Gateway (стандартный Device ID для шлюза/координатора)
        .app_device_version = 0,
    };

    // Создаём готовый шлюзовой эндпоинт (поддерживает все клиентские кластеры, включая Basic 0x0000)
    ezb_af_ep_desc_t ep_desc = ezb_af_create_gateway_endpoint(&ep_cfg);

    ezb_af_device_add_endpoint_desc(dev_desc, ep_desc);
    ezb_af_device_desc_register(dev_desc);
}