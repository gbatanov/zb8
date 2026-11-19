
#include "ezbee/bdb.h"
#include "esp_zigbee.h"
#include "ezbee/zdo/zdo_nwk_mgmt.h"
#include "ezbee/zdo.h"
#include "ezbee/zcl.h"
#include "ezbee/zcl/zcl_general_cmd.h"
#include "ezbee/secur.h"
#include "ezbee/af.h"
#include "ezbee/zdo/zdo_dev_srv_disc.h"
#include "ezbee/zdo/zdo_bind_mgmt.h"
#include <string.h>
#include "esp_mac.h"
#include "ezbee/nwk.h"
#include "ezbee/aps.h"

#include "coordinator.h"

extern const char *TAG;
extern joining_device_ctx_t s_joining_dev;

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

void test_send_active_ep(uint16_t short_addr)
{
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

void aqara_read_basic_attrs(uint16_t short_addr)
{
    static uint16_t attr_ids[] = {
        EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, // 0x0004
                                                 //        EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,  // 0x0005
    };

    ezb_zcl_read_attr_cmd_t cmd_req = {
        .cmd_ctrl = {
            .dst_addr = {
                .addr_mode = EZB_ADDR_MODE_SHORT,
                .u.short_addr = short_addr,
            },
            .dst_ep = 1,
            .src_ep = 1,
            .cluster_id = EZB_ZCL_CLUSTER_ID_BASIC,
        },
        .payload = {
            .attr_number = sizeof(attr_ids) / sizeof(attr_ids[0]),
            .attr_field = attr_ids,
        },
    };

    ezb_err_t err = ezb_zcl_read_attr_cmd_req(&cmd_req);
    ESP_LOGI(TAG, "Отправлен ZCL Read Basic (0x0004, 0x0005) на 0x%04hx, err=0x%x", short_addr, err);
}

// 1. Callback-функция, которая примет ответ Active Endpoint Response от устройства
// 1. Callback-функция с точным типом из zdo_dev_srv_disc.h

// Callback результата Bind Request
void zdo_bind_cb(const ezb_zdp_bind_req_result_t *result, void *user_ctx)
{
    uint16_t cluster_id = (uint16_t)(uintptr_t)user_ctx;
    if (!result || !result->rsp)
    {
        ESP_LOGW(TAG, "Bind Req (cluster 0x%04x) failed, err=0x%x",
                 cluster_id, result ? result->error : 0xFF);
        return;
    }
    ESP_LOGI(TAG, "Bind Response (cluster 0x%04x): status=0x%02x",
             cluster_id, result->rsp->status);
}

// Вспомогательная функция отправки Bind Request (аналог кадров 331 и 337 из эталона)
void send_bind_request(uint16_t dst_short_addr, const ezb_extaddr_t *dev_ieee,
                       uint8_t src_ep, uint16_t cluster_id)
{
    ezb_extaddr_t coord_ieee = {0};
    // Получаем собственный IEEE-адрес координатора (0x0000)
    ezb_nwk_get_extended_address(&coord_ieee);

    ezb_zdo_bind_req_t bind_req = {
        .dst_nwk_addr = dst_short_addr,
        .field = {
            .src_ep = src_ep,
            .cluster_id = cluster_id,
            .dst_addr_mode = 0x03, // 64-bit extended address
            .dst_ep = 1,           // Эндпоинт координатора
        },
        .cb = zdo_bind_cb,
        .user_ctx = (void *)(uintptr_t)cluster_id,
    };

    memcpy(&bind_req.field.src_addr, dev_ieee, sizeof(ezb_extaddr_t));
    memcpy(&bind_req.field.dst_addr, &coord_ieee, sizeof(ezb_extaddr_t));

    ezb_err_t err = ezb_zdo_bind_req(&bind_req);
    ESP_LOGI(TAG, "Sent Bind Req (cluster 0x%04x) to 0x%04x, err=0x%x",
             cluster_id, dst_short_addr, err);
}

// Callback ответа Active Endpoint Request
void zdo_active_ep_cb(const ezb_zdo_active_ep_req_result_t *result, void *user_ctx)
{
    uint8_t target_ep = 1;

    if (!result || !result->rsp)
    {
        ESP_LOGW(TAG, "Active EP request failed or timeout");
        return;
    }
     ESP_LOGI(TAG, "Active EP response from 0x%04x: count=%d",
             s_joining_dev.short_addr, result->rsp->active_ep_count);

    for (uint8_t i = 0; i < result->rsp->active_ep_count; i++)
    {
        ESP_LOGI(TAG, "  -> EP[%d] = %d", i, result->rsp->active_ep_list[i]);
        if (i == 0)
        {
            target_ep = result->rsp->active_ep_list[0];
        }
    }
       // 1. Сюда перенесите ваш вызов ZCL Read Basic (0x0004, 0x0005) на s_joining_dev.short_addr, target_ep
    send_zcl_read_basic(s_joining_dev.short_addr, target_ep);

    // 2. Отправляем два Bind Request точно как в эталонной трассе TI (кадры 331 и 337)
    send_bind_request(s_joining_dev.short_addr, &s_joining_dev.ieee_addr, target_ep, 0x0006); // On/Off
    send_bind_request(s_joining_dev.short_addr, &s_joining_dev.ieee_addr, target_ep, 0x0001); // Power Config


}

void send_zcl_read_basic(uint16_t short_addr, uint8_t target_ep)
{
    static uint16_t attr_ids[] = {
        EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, // 0x0004
        EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,  // 0x0005
    };

    ezb_zcl_read_attr_cmd_t cmd_req = {
        .cmd_ctrl = {
            .dst_addr = {
                .addr_mode = EZB_ADDR_MODE_SHORT,
                .u.short_addr = short_addr,
            },
            .dst_ep = target_ep,
            .src_ep = 1,
            .cluster_id = EZB_ZCL_CLUSTER_ID_BASIC,
        },
        .payload = {
            .attr_number = sizeof(attr_ids) / sizeof(attr_ids[0]),
            .attr_field = attr_ids,
        },
    };

    ezb_err_t err = ezb_zcl_read_attr_cmd_req(&cmd_req);
    ESP_LOGI(TAG, "Отправлен ZCL Read Basic (0x0004, 0x0005) на 0x%04hx, err=0x%x", short_addr, err);
}



void setup_coord_trust_center_addr(void)
{
    uint8_t mac_802154[8] = {0};
    esp_read_mac(mac_802154, ESP_MAC_IEEE802154);

    ezb_extaddr_t coord_ieee = {0};
    uint8_t *dst = (uint8_t *)&coord_ieee;
    for (int i = 0; i < 8; i++) {
        dst[i] = mac_802154[7 - i];
    }

    ezb_nwk_set_extended_address(&coord_ieee);
    ezb_aps_set_trust_center_address(&coord_ieee);

    ezb_extaddr_t tc_check = {0};
    ezb_aps_get_trust_center_address(&tc_check);
    const uint8_t *p = (const uint8_t *)&tc_check;
    ESP_LOGI(TAG, "TC IEEE: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
             *(p + 7), *(p + 6), *(p + 5), *(p + 4),
             *(p + 3), *(p + 2), *(p + 1), *(p + 0));
}