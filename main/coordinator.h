#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_zigbee.h"
#include "ezbee/af.h"
#include "ezbee/aps.h"
#include "ezbee/bdb.h"
#include "ezbee/nwk.h"
#include "ezbee/secur.h"
#include "ezbee/zcl.h"
#include "ezbee/zdo/zdo_bind_mgmt.h"
#include "ezbee/zdo/zdo_dev_srv_disc.h"

// Настройки ZIGBEE 
#define MAX_CHILDREN 32
#define INSTALLCODE_POLICY_ENABLE false
#define ESP_ZIGBEE_STORAGE_PARTITION "nvs"
#define ZB_COORD_ENDPOINT 1
// Все каналы 11..26: 0x07FFF800U (сейчас оставлен 15-й канал 0x00008000U)
#define ZB_CHANNELS_MASK (0x00008000U)

// Настройки UART0 и протокола
#define PROTO_UART_NUM UART_NUM_0
#define PROTO_UART_BAUD 115200
#define PROTO_MAX_PAYLOAD 255

#define ZNP_SOF 0xFE
#define ZNP_MAX_DATA 255

#define ZDO_BIND_REQ   0x2521
#define ZDO_MGMT_PERMIT_JOIN_REQ 0x2536
#define ZDO_MGMT_PERMIT_JOIN_SRSP 0x6536
#define ZDO_MGMT_PERMIT_JOIN_RSP 0x45b6
#define ZDO_PERMIT_JOIN_IND 0x45cb
#define SYS_RESET_REQ 0x4100
#define SYS_PING 0x2101
#define AF_DATA_CONFIRM 0x4480
#define AF_DATA_REQUEST 0x2401 

typedef struct {
    uint16_t cmd;
    uint8_t len;
    uint8_t payload[ZNP_MAX_DATA]; // 255 байт
} bridge_msg_t;

typedef enum
{
    STATE_WAIT_SOF0,
    STATE_WAIT_SOF1,
    STATE_READ_HEADER,
    STATE_READ_PAYLOAD,
    STATE_READ_CRC
} parser_state_t;

void bridge_enqueue_to_pc(uint16_t cmd, const uint8_t *payload, uint16_t len);
void proto_tx_task(void *arg);
void proto_rx_task(void *arg);

void esp_zb_task(void *pvParameters);
void znp_send_sys_reset_ind(void);
void bridge_enqueue_to_pc(uint16_t cmd, const uint8_t *payload, uint16_t len);
