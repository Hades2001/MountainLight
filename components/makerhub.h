#ifndef __MAKERHUB_H__
#define __MAKERHUB_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp_heap_caps.h"
#include "cJSON.h"

// ------ Feature switches ------

#define MAKERHUB_WIFI_ENABLED   1   // 1: enable Wi-Fi, 0: build without Wi-Fi

#define MAKERHUB_WIFI_SCAN_ENABLED     1   // 1: enable background Wi-Fi scan, 0: disable

#define ROOTMAKER_CPU_USAGE_ENABLED       1      // 1: enable CPU usage monitor

#define ROOTMAKER_CPU_USAGE_INTERVAL_MS   1000   // sampling interval in ms

#define MAKERHUB_WIFI_SCAN_INTERVAL_MS (10 * 1000)   // 10 秒扫一次

#define MAKERHUB_NTP_ENABLED 1
#define MAKERHUB_NTP_SYNC_INTERVAL_MS (60 * 60 * 1000) // 1 小时同步一次

#define MAKERHUB_WIFI_SSID_MAX_LEN 32

#define MAKERHUB_WIFI_PASS_MAX_LEN 64

#define MAKERHUB_SLOT_MAX_LEN 128

#define MAX_SCAN_AP  16

#define STA_SSID              "RootMaker"
#define STA_PASS              "12345678"

/* ==== UART0 protocol config ==== */
#define UART_PORT              UART_NUM_0
#define UART_BAUD_RATE         115200
#define UART_RX_BUF_SIZE       2048
#define UART_FRAME_MAX_DATA    1024   // LENGTH MAX = 1024 bytes

/* Serial frame header & CMDs */
#define FRAME_HEADER_0         0xAA
#define FRAME_HEADER_1         0x55

#define CMD_CONFIG_UPDATE_PC2DEV   0x01
#define CMD_STATUS_REQUEST_PC2DEV  0x02
#define CMD_DEVICE_STATUS_DEV2PC   0x11
#define CMD_CONFIG_UPDATE_ACK_DEV2PC 0x12
#define CMD_ERROR_REPORT_DEV2PC    0x7F

/* NVS namespace & keys */
#define NVS_NAMESPACE_NETCFG   "netcfg"
#define NVS_KEY_WIFI_SSID      "wifi_ssid"
#define NVS_KEY_WIFI_PASS      "wifi_pass"
#define NVS_KEY_SLOT1          "slot1"
#define NVS_KEY_SLOT2          "slot2"
#define NVS_KEY_SLOT3          "slot3"

typedef void (*makerhub_wifi_simple_cb_t)(void *user_ctx);
typedef void (*makerhub_wifi_got_ip_cb_t)(const char *ip, void *user_ctx);
typedef void (*makerhub_wifi_fail_cb_t)(wifi_err_reason_t reason, void *user_ctx);

typedef struct {
    makerhub_wifi_simple_cb_t on_disconnected;
    makerhub_wifi_simple_cb_t on_connecting;
    makerhub_wifi_got_ip_cb_t on_got_ip;
    makerhub_wifi_simple_cb_t on_connected;
    makerhub_wifi_fail_cb_t on_connect_failed;
    void *user_ctx;
} makerhub_wifi_event_callbacks_t;

typedef enum {
    MAKERHUB_NTP_STATE_STOPPED = 0,
    MAKERHUB_NTP_STATE_SYNCING,
    MAKERHUB_NTP_STATE_SYNCED,
    MAKERHUB_NTP_STATE_FAILED,
} makerhub_ntp_state_t;

typedef enum {
    RX_WAIT_HEADER_0 = 0,
    RX_WAIT_HEADER_1,
    RX_WAIT_CMD,
    RX_WAIT_LEN_L,
    RX_WAIT_LEN_H,
    RX_WAIT_DATA,
    RX_WAIT_CRC_L,
    RX_WAIT_CRC_H
} rx_state_t;


typedef struct 
{
    rx_state_t   s_rx_state;
    uint8_t      s_rx_cmd;
    uint16_t     s_rx_len;
    uint16_t     s_rx_crc;
    uint16_t     s_rx_crc_calc;
    uint16_t     s_rx_data_index;
    uint8_t      s_rx_data_buf[UART_FRAME_MAX_DATA];

}makerhub_protocol_t;


typedef struct {
    // WiFi/Network layer
    bool    wifi_enabled;          // compile-time feature flag
    bool    wifi_connected;        // current connection state
    char    ip[16];                // "xxx.xxx.xxx.xxx"
    char    ssid[MAKERHUB_WIFI_SSID_MAX_LEN + 1];              // current connected SSID (if any)
    char    pass[MAKERHUB_WIFI_PASS_MAX_LEN + 1];          // current connected password (if any)
    char    slot1[MAKERHUB_SLOT_MAX_LEN + 1];          // current connected slot1 (if any)
    char    slot2[MAKERHUB_SLOT_MAX_LEN + 1];          // current connected slot2 (if any)
    char    slot3[MAKERHUB_SLOT_MAX_LEN + 1];          // current connected slot3 (if any)
    esp_netif_t *sta_netif;

    // Timestamps (ms since boot)
    int64_t last_status_ts_ms;     // last time we built a device_status
    int64_t last_scan_ts_ms;       // last time we updated scan_list

    // Cached WiFi scan result
    uint16_t wifi_ap_count;
    struct {
        char    ssid[MAKERHUB_WIFI_SSID_MAX_LEN + 1];
        int8_t  rssi;
    } wifi_scan_list[MAX_SCAN_AP];

    // 可以预留一些串口协议相关字段（例如最近一次 seq、错误计数等）
    uint32_t last_rx_seq;
    uint32_t last_tx_seq;
    makerhub_protocol_t makerhub_protocol;
} dev_comm_ctx_t;

void makerhub_init(void);
esp_err_t makerhub_nvs_has_config(bool *has_config);
esp_err_t makerhub_nvs_clear_config(void);
esp_err_t makerhub_ntp_start(void);
bool makerhub_ntp_is_synced(void);
makerhub_ntp_state_t makerhub_ntp_get_state(void);
esp_err_t makerhub_ntp_set_timezone(double timezone_hours);
double makerhub_ntp_get_timezone(void);
esp_err_t makerhub_ntp_get_time(time_t *now, struct tm *utc_time);
esp_err_t makerhub_ntp_get_local_time(time_t *now, struct tm *local_time);

#if MAKERHUB_WIFI_ENABLED
void makerhub_wifi_set_event_callbacks(const makerhub_wifi_event_callbacks_t *callbacks);
bool wifi_is_connected(void);
bool wifi_wait_connected(int timeout_ms);
void wifi_get_ip(char *ip, size_t size);
#endif

#endif