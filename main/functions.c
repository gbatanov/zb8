
#include "ezbee/bdb.h"
#include "esp_zigbee.h"
#include "ezbee/zdo/zdo_nwk_mgmt.h"
#include "ezbee/zdo.h"
#include "coordinator.h"

extern const char *TAG;

// Открыть сеть на duration_sec секунд (вызывать под блокировкой, если из другой задачи FreeRTOS)
void permit_join(uint8_t duration_sec, bool with_mutex)
{
    if (with_mutex)
        esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_err_t err = ezb_bdb_open_network(duration_sec);
    if (with_mutex)
        esp_zigbee_lock_release();

    if (err != EZB_ERR_NONE)
    {
        ESP_LOGE(TAG, "Ошибка открытия сети: 0x%x", err);
    }
}

// Если вам нужно разрешить подключение нового датчика только через конкретный роутер
// (например, стоящий в дальней комнате, чтобы датчик не прицепился
// к слабому сигналу координатора), используется низкоуровневая ZDO-функция
// ezb_zdo_permit_joining_req() из ezbee/zdo.h
void permit_join_via_router(uint16_t router_short_addr, uint8_t duration_sec)
{
    ezb_zdo_nwk_mgmt_permit_joining_req_t req = {
        .dst_nwk_addr = router_short_addr, // Короткий адрес роутера или 0xFFFC для всех роутеров
        .field = {
            .permit_duration = duration_sec, // Время в секундах (0 = закрыть)
            .tc_significance = 0x01,         // Обязательно 0x01 для Zigbee 3.0
        },
        .cb = NULL, // Можно передать колбэк ezb_zdo_nwk_mgmt_permit_joining_req_callback_t
        .user_ctx = NULL,
    };

    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_zdo_nwk_mgmt_permit_joining_req(&req);
    esp_zigbee_lock_release();
}

void test_send_active_ep(uint16_t short_addr) {
    ezb_zdo_active_ep_req_t req = {
        .dst_nwk_addr = short_addr,
        .field = {
            .nwk_addr_of_interest = short_addr,
        },
        .cb = NULL,
        .user_ctx = NULL,
    };
    ezb_err_t err = ezb_zdo_active_ep_req(&req);
    ESP_LOGI(TAG, "Отправлен Active_EP_req на 0x%04hx, err=0x%x", short_addr, err);
}