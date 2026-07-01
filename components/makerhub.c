#include "makerhub.h"

static const char *TAG = "makerhub";

static dev_comm_ctx_t g_comm_ctx = {
    .wifi_enabled     = MAKERHUB_WIFI_ENABLED,
    .wifi_connected   = false,
    .ip               = "0.0.0.0",
    .ssid             = {0},
    .pass             = {0},
    .last_status_ts_ms = 0,
    .last_scan_ts_ms   = 0,
    .wifi_ap_count     = 0,
    .last_rx_seq       = 0,
    .last_tx_seq       = 0,
    .makerhub_protocol = {
        .s_rx_state = RX_WAIT_HEADER_0,
        .s_rx_cmd = 0,
        .s_rx_len = 0,
        .s_rx_crc = 0,
        .s_rx_crc_calc = 0,
        .s_rx_data_index = 0,
    },
    .sta_netif = NULL,
    .slot1            = {0},
    .slot2            = {0},
    .slot3            = {0},
};


/* CRC-16/Modbus:
 * poly 0xA001, init 0xFFFF, reflect in/out, no xorout，范围 CMD..DATA:contentReference[oaicite:4]{index=4}
 */
static uint16_t crc16_modbus(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}


static esp_err_t save_wifi_and_custom_to_nvs()
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE_NETCFG, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    if (g_comm_ctx.ssid[0] != '\0') {
    err = nvs_set_str(h, NVS_KEY_WIFI_SSID, g_comm_ctx.ssid);
    if (err != ESP_OK) goto out;
    }
    if (g_comm_ctx.pass[0] != '\0') {
    err = nvs_set_str(h, NVS_KEY_WIFI_PASS, g_comm_ctx.pass);
    if (err != ESP_OK) goto out;
    }
    if (g_comm_ctx.slot1[0] != '\0') {
    err = nvs_set_str(h, NVS_KEY_SLOT1, g_comm_ctx.slot1);
    if (err != ESP_OK) goto out;
    }
    if (g_comm_ctx.slot2[0] != '\0') {
    err = nvs_set_str(h, NVS_KEY_SLOT2, g_comm_ctx.slot2);
    if (err != ESP_OK) goto out;
    }
    if (g_comm_ctx.slot3[0] != '\0') {
    err = nvs_set_str(h, NVS_KEY_SLOT3, g_comm_ctx.slot3);
    if (err != ESP_OK) goto out;
    }
    err = nvs_commit(h);
    out:
    nvs_close(h);
    return err;
}


#if MAKERHUB_WIFI_ENABLED

static esp_err_t load_wifi_config_from_nvs(bool *has_config)
{
    *has_config = false;
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE_NETCFG, NVS_READONLY, &h);
    if (err != ESP_OK) {
    return err;
    }

    size_t len_ssid = sizeof(g_comm_ctx.ssid);
    size_t len_pass = sizeof(g_comm_ctx.pass);
    err = nvs_get_str(h, NVS_KEY_WIFI_SSID, g_comm_ctx.ssid, &len_ssid);
    if (err != ESP_OK) {
    nvs_close(h);
    return err;
    }
    err = nvs_get_str(h, NVS_KEY_WIFI_PASS, g_comm_ctx.pass, &len_pass);
    if (err != ESP_OK) {
    nvs_close(h);
    return err;
    }

    nvs_close(h);
    if (g_comm_ctx.ssid[0] != '\0') {
    *has_config = true;
    }
    return ESP_OK;
}

static void wifi_reload_from_nvs_and_reconnect(void)
{
    bool has_config = false;
    if (load_wifi_config_from_nvs(&has_config) != ESP_OK || !has_config) {
        ESP_LOGW(TAG, "No Wi-Fi config in NVS when reloading.");
        return;
    }

    wifi_config_t wifi_config = { 0 };
    strncpy((char *)wifi_config.sta.ssid, g_comm_ctx.ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, g_comm_ctx.pass, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg = (wifi_pmf_config_t){
        .capable = true,
        .required = false,
    };

    ESP_LOGI(TAG, "Reload Wi-Fi from NVS: SSID=%s", g_comm_ctx.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    esp_wifi_disconnect();
    esp_wifi_connect();
}

static void wifi_scan_update_cache(void)
{
#if !MAKERHUB_WIFI_SCAN_ENABLED
    // 如果扫描功能整体关闭，直接返回
    return;
#endif

    if (!g_comm_ctx.wifi_enabled) return;

    // 只在未连接时才扫描
    //if (g_comm_ctx.wifi_connected) {
    //    ESP_LOGI(TAG, "Skip Wi-Fi scan: already connected.");
    //    return;
    //}

    wifi_scan_config_t scan_cfg = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
    };

    ESP_LOGI(TAG, "Start Wi-Fi scan (disconnected state)...");
    esp_err_t scan_ret = esp_wifi_scan_start(&scan_cfg, true);
    if (scan_ret != ESP_OK) {
        ESP_LOGW(TAG, "Wi-Fi scan failed: %s", esp_err_to_name(scan_ret));
        return;
    }

    uint16_t ap_num = MAX_SCAN_AP;
    wifi_ap_record_t ap_records[MAX_SCAN_AP] = {0};
    if (esp_wifi_scan_get_ap_records(&ap_num, ap_records) != ESP_OK) {
        ESP_LOGW(TAG, "Wi-Fi scan get records failed.");
        return;
    }

    g_comm_ctx.wifi_ap_count = ap_num;
    for (int i = 0; i < ap_num; ++i) {
        strncpy(g_comm_ctx.wifi_scan_list[i].ssid,
                (const char *)ap_records[i].ssid,
                sizeof(g_comm_ctx.wifi_scan_list[i].ssid) - 1);
        g_comm_ctx.wifi_scan_list[i].ssid[sizeof(g_comm_ctx.wifi_scan_list[i].ssid) - 1] = '\0';
        g_comm_ctx.wifi_scan_list[i].rssi = ap_records[i].rssi;
    }

    ESP_LOGI(TAG, "Wi-Fi scan done: %u AP found.", ap_num);
}

/* ==== Wi-Fi 事件处理（STA 模式） ==== */
static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Wi-Fi STA started, connecting...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi disconnected");
        g_comm_ctx.wifi_connected = false;
        strcpy(g_comm_ctx.ip, "0.0.0.0");
        g_comm_ctx.ssid[0] = '\0';
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        snprintf(g_comm_ctx.ip, sizeof(g_comm_ctx.ip), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Got IP: %s", g_comm_ctx.ip);
        g_comm_ctx.wifi_connected = true;

        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            strncpy(g_comm_ctx.ssid, (const char *)ap.ssid, sizeof(g_comm_ctx.ssid) - 1);
        } else {
            g_comm_ctx.ssid[0] = '\0';
        }
    }
}

static void wifi_init_sta(void)
{
    if (!g_comm_ctx.wifi_enabled) {
        ESP_LOGW(TAG, "Wi-Fi disabled by macro.");
        return;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    g_comm_ctx.sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        NULL));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &wifi_event_handler,
        NULL,
        NULL));

    // Try load Wi-Fi config from NVS first
    bool has_config = false;
    load_wifi_config_from_nvs(&has_config);

    wifi_config_t wifi_config = { 0 };

    if (has_config) {
        strncpy((char *)wifi_config.sta.ssid, g_comm_ctx.ssid, MAKERHUB_WIFI_SSID_MAX_LEN);
        strncpy((char *)wifi_config.sta.password, g_comm_ctx.pass, MAKERHUB_WIFI_PASS_MAX_LEN);
        ESP_LOGI(TAG, "Using Wi-Fi config from NVS: SSID=%s", g_comm_ctx.ssid);
    } else {
        // fallback to compile-time default
        strncpy((char *)wifi_config.sta.ssid, STA_SSID, MAKERHUB_WIFI_SSID_MAX_LEN);
        strncpy((char *)wifi_config.sta.password, STA_PASS, MAKERHUB_WIFI_PASS_MAX_LEN);
        ESP_LOGI(TAG, "Using default Wi-Fi: SSID=%s", STA_SSID);
    }

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg = (wifi_pmf_config_t){
        .capable = true,
        .required = false,
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi STA init done.");
}

bool wifi_is_connected(void)
{
    return g_comm_ctx.wifi_connected;
}

bool wifi_wait_connected(int timeout_ms)
{
    int start_time = xTaskGetTickCount();
    while (!g_comm_ctx.wifi_connected) {
        if (xTaskGetTickCount() - start_time > timeout_ms / portTICK_PERIOD_MS) {
            ESP_LOGW(TAG, "WiFi connect timeout");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "WiFi connected");
    return true;
}

void wifi_get_ip(char *ip, size_t size)
{
    strncpy(ip, g_comm_ctx.ip, size);
}

#endif


#if MAKERHUB_WIFI_SCAN_ENABLED && MAKERHUB_WIFI_ENABLED

static void wifi_scan_task(void *arg)
{
    // 可选：启动前稍等一下，让 WiFi 初始化起来
    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1) {
        // 如果未连接网络，就尝试更新扫描结果
        //if (!g_comm_ctx.wifi_connected) {
        //    wifi_scan_update_cache();
        //} else {
        //    // 已连接时什么也不做，只是保持 scan_list 为上一次缓存
        //    ESP_LOGD(TAG, "wifi_scan_task: connected, skip scan.");
        //}
        wifi_scan_update_cache();
        vTaskDelay(pdMS_TO_TICKS(MAKERHUB_WIFI_SCAN_INTERVAL_MS));
    }
}

static void wifi_scan_task_start(void)
{
    if (!g_comm_ctx.wifi_enabled) return;
    BaseType_t ret = xTaskCreate(
        wifi_scan_task,
        "wifi_scan_task",
        4096,
        NULL,
        4,
        NULL
    );
    if (ret != pdPASS) {
        ESP_LOGW(TAG, "Failed to create wifi_scan_task");
    }
}

#endif
static void uart_send_json_line(const char *json_str)
{
    if (!json_str) {
        return;
    }
    size_t len = strlen(json_str);
    if (len == 0) {
        return;
    }
    uart_write_bytes(UART_PORT, json_str, len);
    const char newline = '\n';
    uart_write_bytes(UART_PORT, &newline, 1);
}

static void handle_config_update(const uint8_t *data, uint16_t len)
{
    // JSON 字符串复制并补 '\0'
    char *json = malloc(len + 1);
    if (!json) return;
    memcpy(json, data, len);
    json[len] = '\0';

    int seq = 0;

    cJSON *root = cJSON_Parse(json);
    if (!root) {
        ESP_LOGW(TAG, "config_update: invalid JSON");
        free(json);
        return;
    }

    cJSON *seq_item = cJSON_GetObjectItem(root, "seq");
    if (cJSON_IsNumber(seq_item)) {
        seq = seq_item->valueint;
    }

    cJSON *config = cJSON_GetObjectItem(root, "config");
    if (cJSON_IsObject(config)) {
        cJSON *wifi = cJSON_GetObjectItem(config, "wifi");
        if (cJSON_IsObject(wifi)) {
            cJSON *ssid = cJSON_GetObjectItem(wifi, "ssid");
            cJSON *pass = cJSON_GetObjectItem(wifi, "password");
            if (cJSON_IsString(ssid) && ssid->valuestring) strncpy(g_comm_ctx.ssid, ssid->valuestring, MAKERHUB_WIFI_SSID_MAX_LEN);
            if (cJSON_IsString(pass) && pass->valuestring) strncpy(g_comm_ctx.pass, pass->valuestring, MAKERHUB_WIFI_PASS_MAX_LEN);
        }

        cJSON *custom = cJSON_GetObjectItem(config, "custom_data");
        if (cJSON_IsObject(custom)) {
            cJSON *s1 = cJSON_GetObjectItem(custom, "slot1");
            cJSON *s2 = cJSON_GetObjectItem(custom, "slot2");
            cJSON *s3 = cJSON_GetObjectItem(custom, "slot3");
            if (cJSON_IsString(s1) && s1->valuestring) strncpy(g_comm_ctx.slot1, s1->valuestring, MAKERHUB_SLOT_MAX_LEN);
            if (cJSON_IsString(s2) && s2->valuestring) strncpy(g_comm_ctx.slot2, s2->valuestring, MAKERHUB_SLOT_MAX_LEN);
            if (cJSON_IsString(s3) && s3->valuestring) strncpy(g_comm_ctx.slot3, s3->valuestring, MAKERHUB_SLOT_MAX_LEN);
        }
    }
    // 构造 ack JSON（CMD = 0x12）:contentReference[oaicite:6]{index=6}
    esp_err_t err = save_wifi_and_custom_to_nvs();

    cJSON *ack = cJSON_CreateObject();
    cJSON_AddNumberToObject(ack, "ver", 1);
    cJSON_AddStringToObject(ack, "msg_type", "config_update_ack");
    cJSON_AddNumberToObject(ack, "seq", seq);

    cJSON *result = cJSON_CreateObject();
    cJSON_AddStringToObject(result, "status", (err == ESP_OK) ? "ok" : "error");
    cJSON_AddNumberToObject(result, "error_code", (err == ESP_OK) ? 0 : (int)err);
    cJSON_AddStringToObject(result, "error_msg", (err == ESP_OK) ? "" : "nvs_save_failed");
    cJSON_AddItemToObject(ack, "result", result);

    char *ack_str = cJSON_PrintUnformatted(ack);
    if (ack_str) {
        // Device -> PC: JSON + '\n'
        uart_send_json_line(ack_str);
        cJSON_free(ack_str);
    }
    cJSON_Delete(ack);
    cJSON_Delete(root);
    free(json);

    #if MAKERHUB_WIFI_ENABLED
        if (err == ESP_OK && g_comm_ctx.ssid[0] != '\0' && g_comm_ctx.pass[0] != '\0') {
            wifi_reload_from_nvs_and_reconnect();
        }
    #endif
}

static void handle_status_request(const uint8_t *data, uint16_t len)
{
    int seq = 0;
    char *json = malloc(len + 1);
    if (!json) return;
    memcpy(json, data, len);
    json[len] = '\0';

    cJSON *root = cJSON_Parse(json);
    if (root) {
        cJSON *seq_item = cJSON_GetObjectItem(root, "seq");
        if (cJSON_IsNumber(seq_item)) seq = seq_item->valueint;
        cJSON_Delete(root);
    }
    free(json);

    size_t heap_free = esp_get_free_heap_size();
    size_t heap_total = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
    size_t heap_used = heap_total - heap_free;

    size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    size_t psram_free  = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t psram_used  = (psram_total > 0) ? (psram_total - psram_free) : 0;

    // 构造 device_status JSON:contentReference[oaicite:7]{index=7}
    cJSON *obj = cJSON_CreateObject();
    cJSON_AddNumberToObject(obj, "ver", 1);
    cJSON_AddStringToObject(obj, "msg_type", "device_status");
    cJSON_AddNumberToObject(obj, "seq", seq);

    cJSON *data_obj = cJSON_CreateObject();

    cJSON *ram = cJSON_CreateObject();
    cJSON_AddNumberToObject(ram, "total", (double)heap_total);
    cJSON_AddNumberToObject(ram, "free",  (double)heap_free);
    cJSON_AddNumberToObject(ram, "used",  (double)heap_used);
    cJSON_AddStringToObject(ram, "unit", "bytes");
    cJSON_AddItemToObject(data_obj, "ram", ram);

    cJSON *psram = cJSON_CreateObject();
    cJSON_AddNumberToObject(psram, "total", (double)psram_total);
    cJSON_AddNumberToObject(psram, "free",  (double)psram_free);
    cJSON_AddNumberToObject(psram, "used",  (double)psram_used);
    cJSON_AddStringToObject(psram, "unit", "bytes");
    cJSON_AddItemToObject(data_obj, "psram", psram);

    cJSON *cpu = cJSON_CreateObject();
    cJSON_AddNumberToObject(cpu, "usage_pct", 0.0); // 简化处理
    cJSON_AddItemToObject(data_obj, "cpu", cpu);

    cJSON *net = cJSON_CreateObject();
#if MAKERHUB_WIFI_ENABLED
    cJSON_AddStringToObject(net,
                            "state",
                            g_comm_ctx.wifi_connected ? "connected" : "disconnected");
    cJSON_AddStringToObject(net, "ip",   g_comm_ctx.ip);
    cJSON_AddStringToObject(net, "ssid", g_comm_ctx.ssid);

  #if MAKERHUB_WIFI_SCAN_ENABLED
    // 使用异步任务维护的缓存
    cJSON *scan_arr = cJSON_CreateArray();
    for (int i = 0; i < g_comm_ctx.wifi_ap_count && i < MAX_SCAN_AP; ++i) {
        cJSON *ap = cJSON_CreateObject();
        cJSON_AddStringToObject(ap, "ssid", g_comm_ctx.wifi_scan_list[i].ssid);
        cJSON_AddNumberToObject(ap, "rssi", g_comm_ctx.wifi_scan_list[i].rssi);
        cJSON_AddItemToArray(scan_arr, ap);
    }
    cJSON_AddItemToObject(net, "scan_list", scan_arr);
  #else
    // 扫描功能关闭时，scan_list 为空数组
    cJSON_AddItemToObject(net, "scan_list", cJSON_CreateArray());
  #endif

#else
    // WiFi 总开关关闭时
    cJSON_AddStringToObject(net, "state", "disabled");
    cJSON_AddStringToObject(net, "ip",   "0.0.0.0");
    cJSON_AddStringToObject(net, "ssid", "");
    cJSON_AddItemToObject(net, "scan_list", cJSON_CreateArray());
#endif


    cJSON_AddItemToObject(data_obj, "network", net);
    cJSON_AddItemToObject(obj, "data", data_obj);

    char *out = cJSON_PrintUnformatted(obj);
    if (out) {
        // Device -> PC: JSON + '\n'
        uart_send_json_line(out);
        cJSON_free(out);
    }
    cJSON_Delete(obj);
}

static void protocol_reset_state(void)
{
    g_comm_ctx.makerhub_protocol.s_rx_state = RX_WAIT_HEADER_0;
    g_comm_ctx.makerhub_protocol.s_rx_cmd = 0;
    g_comm_ctx.makerhub_protocol.s_rx_len = 0;
    g_comm_ctx.makerhub_protocol.s_rx_crc = 0;
    g_comm_ctx.makerhub_protocol.s_rx_crc_calc = 0;
    g_comm_ctx.makerhub_protocol.s_rx_data_index = 0;
}

static void protocol_handle_frame(uint8_t cmd, const uint8_t *data, uint16_t len)
{
    switch (cmd) {
    case CMD_CONFIG_UPDATE_PC2DEV:
        handle_config_update(data, len);
        break;
    case CMD_STATUS_REQUEST_PC2DEV:
        handle_status_request(data, len);
        break;
    default:
        ESP_LOGW(TAG, "Unknown CMD: 0x%02X", cmd);
        break;
    }
}

static void protocol_feed_byte(uint8_t byte)
{
    switch (g_comm_ctx.makerhub_protocol.s_rx_state) {
    case RX_WAIT_HEADER_0:
        if (byte == FRAME_HEADER_0) {
            g_comm_ctx.makerhub_protocol.s_rx_state = RX_WAIT_HEADER_1;
        }
        break;
    case RX_WAIT_HEADER_1:
        if (byte == FRAME_HEADER_1) {
            g_comm_ctx.makerhub_protocol.s_rx_state = RX_WAIT_CMD;
        } else {
            g_comm_ctx.makerhub_protocol.s_rx_state = RX_WAIT_HEADER_0;
        }
        break;
    case RX_WAIT_CMD:
        g_comm_ctx.makerhub_protocol.s_rx_cmd = byte;
        g_comm_ctx.makerhub_protocol.s_rx_state = RX_WAIT_LEN_L;
        break;
    case RX_WAIT_LEN_L:
        g_comm_ctx.makerhub_protocol.s_rx_len = byte;
        g_comm_ctx.makerhub_protocol.s_rx_state = RX_WAIT_LEN_H;
        break;
    case RX_WAIT_LEN_H:
        g_comm_ctx.makerhub_protocol.s_rx_len |= ((uint16_t)byte << 8);
        if (g_comm_ctx.makerhub_protocol.s_rx_len > UART_FRAME_MAX_DATA) {
            ESP_LOGW(TAG, "Frame data too long: %u", g_comm_ctx.makerhub_protocol.s_rx_len);
            protocol_reset_state();
        } else {
            g_comm_ctx.makerhub_protocol.s_rx_data_index = 0;
            g_comm_ctx.makerhub_protocol.s_rx_state = (g_comm_ctx.makerhub_protocol.s_rx_len == 0) ? RX_WAIT_CRC_L : RX_WAIT_DATA;
        }
        break;
    case RX_WAIT_DATA:
        g_comm_ctx.makerhub_protocol.s_rx_data_buf[g_comm_ctx.makerhub_protocol.s_rx_data_index++] = byte;
        if (g_comm_ctx.makerhub_protocol.s_rx_data_index >= g_comm_ctx.makerhub_protocol.s_rx_len) {
            g_comm_ctx.makerhub_protocol.s_rx_state = RX_WAIT_CRC_L;
        }
        break;
    case RX_WAIT_CRC_L:
        g_comm_ctx.makerhub_protocol.s_rx_crc = byte;
        g_comm_ctx.makerhub_protocol.s_rx_state = RX_WAIT_CRC_H;
        break;
    case RX_WAIT_CRC_H:
        g_comm_ctx.makerhub_protocol.s_rx_crc |= ((uint16_t)byte << 8);

        // 计算 CRC: CMD + LEN_L + LEN_H + DATA
        {
            uint8_t *crc_buf = malloc(3 + g_comm_ctx.makerhub_protocol.s_rx_len);
            if (crc_buf) {
                crc_buf[0] = g_comm_ctx.makerhub_protocol.s_rx_cmd;
                crc_buf[1] = g_comm_ctx.makerhub_protocol.s_rx_len & 0xFF;
                crc_buf[2] = (g_comm_ctx.makerhub_protocol.s_rx_len >> 8) & 0xFF;
                memcpy(&crc_buf[3], g_comm_ctx.makerhub_protocol.s_rx_data_buf, g_comm_ctx.makerhub_protocol.s_rx_len);
                g_comm_ctx.makerhub_protocol.s_rx_crc_calc = crc16_modbus(crc_buf, 3 + g_comm_ctx.makerhub_protocol.s_rx_len);
                free(crc_buf);
            } else {
                g_comm_ctx.makerhub_protocol.s_rx_crc_calc = 0;
            }
        }

        if (g_comm_ctx.makerhub_protocol.s_rx_crc_calc == g_comm_ctx.makerhub_protocol.s_rx_crc) {
            protocol_handle_frame(g_comm_ctx.makerhub_protocol.s_rx_cmd, g_comm_ctx.makerhub_protocol.s_rx_data_buf, g_comm_ctx.makerhub_protocol.s_rx_len);
        } else {
            ESP_LOGW(TAG, "CRC error: recv=0x%04X calc=0x%04X", g_comm_ctx.makerhub_protocol.s_rx_crc, g_comm_ctx.makerhub_protocol.s_rx_crc_calc);
        }
        protocol_reset_state();
        break;
    default:
        protocol_reset_state();
        break;
    }
}

static void uart_protocol_task(void *arg)
{
    uint8_t buf[128];

    while (1) {
        int len = uart_read_bytes(UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(100));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                protocol_feed_byte(buf[i]);
            }
        }
    }
}

static void uart_protocol_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_PORT, &uart_config);
    // 使用芯片默认 UART0 引脚；若需要自定义，可 uart_set_pin().
    uart_driver_install(UART_PORT, UART_RX_BUF_SIZE, 0, 0, NULL, 0);

    protocol_reset_state();
    xTaskCreate(uart_protocol_task, "uart_protocol_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "UART protocol on UART0 @ %d 8N1 ready", UART_BAUD_RATE);
}


void makerhub_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

#if MAKERHUB_WIFI_ENABLED
    wifi_init_sta();
#endif

#if MAKERHUB_WIFI_SCAN_ENABLED && MAKERHUB_WIFI_ENABLED
    wifi_scan_update_cache();
    wifi_scan_task_start();
#endif

    uart_protocol_init();
}
