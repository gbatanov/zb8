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



/* ==================== НАСТРОЙКИ ZIGBEE ==================== */
#define MAX_CHILDREN 32
#define INSTALLCODE_POLICY_ENABLE false
#define ESP_ZIGBEE_STORAGE_PARTITION "nvs"
#define ZB_COORD_ENDPOINT 1
// Все каналы 11..26: 0x07FFF800U (сейчас оставлен 15-й канал 0x00008000U)
#define ZB_CHANNELS_MASK (0x00008000U)

/* ==================== НАСТРОЙКИ UART0 И ПРОТОКОЛА ==================== */
#define PROTO_UART_NUM UART_NUM_0
#define PROTO_UART_BAUD 115200
#define PROTO_MAX_PAYLOAD 256
#define PROTO_ACK_TIMEOUT_MS 150
#define PROTO_MAX_RETRIES 3

#define PROTO_SOF_0 0xC0
#define PROTO_SOF_1 0xC0

typedef enum
{
    PKT_TYPE_DATA = 0x01,
    PKT_TYPE_ACK = 0x02,
    PKT_TYPE_NACK = 0x03,
} proto_pkt_type_t;

/* Команды бинарного протокола (ESP32-H2 <-> Golang ПК) */
typedef enum
{
    CMD_PERMIT_JOIN = 0x01,  // ПК -> ESP: [duration_sec (1B)]
    CMD_DEV_ANNOUNCE = 0x02, // ESP -> ПК: [short (2B)] [ieee (8B)] [capability (1B)]
    CMD_DEV_LEAVE = 0x03,    // ESP -> ПК: [short (2B)] [ieee (8B)]
    CMD_ZDO_BIND_REQ = 0x12, // ПК -> ESP: [dst_short (2B)] [src_ieee (8B)] [src_ep (1B)] [cluster (2B)] [dst_ep (1B)]
    CMD_APS_SEND_RAW = 0x20, // ПК -> ESP: [dst_short (2B)] [src_ep (1B)] [dst_ep (1B)] [profile_id (2B)] [cluster_id (2B)] [tx_opt (1B)] [asdu...]
    CMD_APS_RECV_RAW = 0x21, // ESP -> ПК: [src_short (2B)] [src_ep (1B)] [dst_ep (1B)] [profile_id (2B)] [cluster_id (2B)] [rssi (1B)] [lqi (1B)] [asdu...]
    CMD_APS_DATA_CNF = 0x22, // ESP -> ПК: [dst_short (2B)] [dst_ep (1B)] [cluster_id (2B)] [status (1B)]
    CMD_SYS_PING = 0xF0,     // ПК <-> ESP: [uptime_sec (4B)] [pan_id (2B)] [channel (1B)]
} proto_cmd_t;

typedef struct
{
    uint8_t cmd;
    uint16_t len;
    uint8_t payload[PROTO_MAX_PAYLOAD];
} bridge_msg_t;

typedef enum
{
    STATE_WAIT_SOF0,
    STATE_WAIT_SOF1,
    STATE_READ_HEADER,
    STATE_READ_PAYLOAD,
    STATE_READ_CRC
} parser_state_t;

uint16_t calc_crc16(const uint8_t *data, size_t len);
void uart_send_raw_frame(uint8_t seq, uint8_t type, uint8_t cmd, const uint8_t *payload, uint16_t len);
void bridge_enqueue_to_pc(uint8_t cmd, const uint8_t *payload, uint16_t len);
void proto_tx_task(void *arg);
void proto_rx_task(void *arg);

void esp_zb_task(void *pvParameters);
void on_valid_pc_frame(uint8_t seq, uint8_t type, uint8_t cmd, const uint8_t *payload, uint16_t len);
