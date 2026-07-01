#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "esp_log.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"

#include "web_server.h"

static const char *TAG = "WEB_SERVER";
static const char *NVS_NAMESPACE = "sky_cfg";

extern const unsigned char index_html_start[] asm("_binary_index_html_start");
extern const unsigned char index_html_end[]   asm("_binary_index_html_end");

extern const unsigned char title_png_start[] asm("_binary_title_png_start");
extern const unsigned char title_png_end[]   asm("_binary_title_png_end");

static QueueHandle_t g_state_queue = NULL;

static sky_state_t g_state = {
    .mode = SKY_MODE_TIME,

    .angle = 90,
    .brightness = 80,

    .max_brightness = 100,
    .latitude = 22.5431,
    .longitude = 114.0579,
    .timezone = 8.0,
    .moon_mode = true,

    .night_brightness = 40,
    .color_temp = 3000,
};

static TimerHandle_t g_nvs_save_timer = NULL;
static TaskHandle_t g_nvs_save_task_handle = NULL;

#define NVS_SAVE_DELAY_MS 2000

static portMUX_TYPE g_state_mux = portMUX_INITIALIZER_UNLOCKED;

static sky_state_t get_sky_state_snapshot(void)
{
    sky_state_t snapshot;

    portENTER_CRITICAL(&g_state_mux);
    snapshot = g_state;
    portEXIT_CRITICAL(&g_state_mux);

    return snapshot;
}

static void set_sky_state_mode(sky_state_t *state)
{
    portENTER_CRITICAL(&g_state_mux);
    memcpy(&g_state, state, sizeof(sky_state_t));
    portEXIT_CRITICAL(&g_state_mux);
}

QueueHandle_t get_state_queue(void)
{
    return g_state_queue;
}

void set_sky_state(sky_state_t *state)
{
    set_sky_state_mode(state);
}

static const char *mode_to_string(sky_mode_t mode)
{
    switch (mode) {
        case SKY_MODE_FIXED: return "fixed";
        case SKY_MODE_TIME:  return "time";
        case SKY_MODE_NIGHT: return "night";
        case SKY_MODE_OFF:   return "off";
        default:             return "off";
    }
}

static sky_mode_t string_to_mode(const char *mode)
{
    if (mode == NULL) return SKY_MODE_OFF;

    if (strcmp(mode, "fixed") == 0) return SKY_MODE_FIXED;
    if (strcmp(mode, "time") == 0)  return SKY_MODE_TIME;
    if (strcmp(mode, "night") == 0) return SKY_MODE_NIGHT;
    if (strcmp(mode, "off") == 0)   return SKY_MODE_OFF;

    return SKY_MODE_OFF;
}

static int clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}


static esp_err_t save_sky_state_to_nvs(const sky_state_t *state)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_i32(handle, "mode", (int32_t)state->mode);
    if (ret != ESP_OK) goto done;

    ret = nvs_set_i32(handle, "angle", state->angle);
    if (ret != ESP_OK) goto done;

    ret = nvs_set_i32(handle, "brightness", state->brightness);
    if (ret != ESP_OK) goto done;

    ret = nvs_set_i32(handle, "max_bright", state->max_brightness);
    if (ret != ESP_OK) goto done;

    /*
     * NVS 不建议直接存 double。
     * 这里将经纬度放大 1,000,000 倍保存为 int32。
     */
    int32_t lat_e6 = (int32_t)llround(state->latitude * 1000000.0);
    int32_t lon_e6 = (int32_t)llround(state->longitude * 1000000.0);

    /*
     * 时区可能有半区，例如 UTC+5.5。
     * 这里放大 100 倍保存。
     */
    int32_t tz_x100 = (int32_t)llround(state->timezone * 100.0);

    ret = nvs_set_i32(handle, "lat_e6", lat_e6);
    if (ret != ESP_OK) goto done;

    ret = nvs_set_i32(handle, "lon_e6", lon_e6);
    if (ret != ESP_OK) goto done;

    ret = nvs_set_i32(handle, "tz_x100", tz_x100);
    if (ret != ESP_OK) goto done;

    ret = nvs_set_u8(handle, "moon_mode", state->moon_mode ? 1 : 0);
    if (ret != ESP_OK) goto done;

    ret = nvs_set_i32(handle, "night_bri", state->night_brightness);
    if (ret != ESP_OK) goto done;

    ret = nvs_set_i32(handle, "color_temp", state->color_temp);
    if (ret != ESP_OK) goto done;

    ret = nvs_commit(handle);

done:
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "save NVS failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "state saved to NVS");
    }

    nvs_close(handle);
    return ret;
}

static void nvs_save_timer_cb(TimerHandle_t xTimer)
{
    if (g_nvs_save_task_handle != NULL) {
        xTaskNotifyGive(g_nvs_save_task_handle);
    }
}

static void schedule_save_sky_state_to_nvs(void)
{
    if (g_nvs_save_timer == NULL) {
        g_nvs_save_timer = xTimerCreate(
            "nvs_save",
            pdMS_TO_TICKS(NVS_SAVE_DELAY_MS),
            pdFALSE,
            NULL,
            nvs_save_timer_cb
        );

        if (g_nvs_save_timer == NULL) {
            ESP_LOGE(TAG, "failed to create NVS save timer");
            return;
        }
    }

    xTimerStop(g_nvs_save_timer, 0);
    xTimerStart(g_nvs_save_timer, 0);
}

static void nvs_save_task(void *arg)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sky_state_t snapshot = get_sky_state_snapshot();

        esp_err_t ret = save_sky_state_to_nvs(&snapshot);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "NVS save failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "NVS saved");
        }
    }
}

esp_err_t load_sky_state_from_nvs(sky_state_t *state)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);

    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "NVS namespace not found, use default state");
        memcpy(state, &g_state, sizeof(sky_state_t));
        return ret;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    int32_t value_i32 = 0;
    uint8_t value_u8 = 0;

    if (nvs_get_i32(handle, "mode", &value_i32) == ESP_OK) {
        if (value_i32 >= SKY_MODE_FIXED && value_i32 <= SKY_MODE_OFF) {
            state->mode = (sky_mode_t)value_i32;
        }
    }

    if (nvs_get_i32(handle, "angle", &value_i32) == ESP_OK) {
        state->angle = clamp_int(value_i32, 0, 180);
    }

    if (nvs_get_i32(handle, "brightness", &value_i32) == ESP_OK) {
        state->brightness = clamp_int(value_i32, 0, 100);
    }

    if (nvs_get_i32(handle, "max_bright", &value_i32) == ESP_OK) {
        state->max_brightness = clamp_int(value_i32, 0, 100);
    }

    if (nvs_get_i32(handle, "lat_e6", &value_i32) == ESP_OK) {
        double latitude = value_i32 / 1000000.0;
        if (latitude >= -90.0 && latitude <= 90.0) {
            state->latitude = latitude;
        }
    }

    if (nvs_get_i32(handle, "lon_e6", &value_i32) == ESP_OK) {
        double longitude = value_i32 / 1000000.0;
        if (longitude >= -180.0 && longitude <= 180.0) {
            state->longitude = longitude;
        }
    }

    if (nvs_get_i32(handle, "tz_x100", &value_i32) == ESP_OK) {
        double timezone = value_i32 / 100.0;
        if (timezone >= -12.0 && timezone <= 14.0) {
            state->timezone = timezone;
        }
    }

    if (nvs_get_u8(handle, "moon_mode", &value_u8) == ESP_OK) {
        state->moon_mode = value_u8 ? true : false;
    }

    if (nvs_get_i32(handle, "night_bri", &value_i32) == ESP_OK) {
        state->night_brightness = clamp_int(value_i32, 0, 100);
    }

    if (nvs_get_i32(handle, "color_temp", &value_i32) == ESP_OK) {
        state->color_temp = clamp_int(value_i32, 1700, 6500);
    }

    nvs_close(handle);

    ESP_LOGI(TAG,
             "state loaded from NVS: mode=%s angle=%d brightness=%d "
             "max_brightness=%d lat=%.6f lon=%.6f tz=%.2f moon=%d "
             "night_brightness=%d color_temp=%d",
             mode_to_string(state->mode),
             state->angle,
             state->brightness,
             state->max_brightness,
             state->latitude,
             state->longitude,
             state->timezone,
             state->moon_mode,
             state->night_brightness,
             state->color_temp);

    return ESP_OK;
}

/*
 * 这里连接你的实际控制逻辑：
 *
 * 固定角度：
 *   mode=fixed, angle, brightness
 *
 * 时间同步：
 *   mode=time, latitude, longitude, timezone, max_brightness, moon_mode
 *
 * 夜灯：
 *   mode=night, night_brightness, color_temp
 *
 * 关闭：
 *   mode=off
 */
static void apply_sky_state(const sky_state_t *state)
{
    ESP_LOGI(TAG,
             "Apply state: mode=%s angle=%d brightness=%d max_brightness=%d "
             "lat=%.6f lon=%.6f tz=%.2f moon=%d night_brightness=%d color_temp=%d",
             mode_to_string(state->mode),
             state->angle,
             state->brightness,
             state->max_brightness,
             state->latitude,
             state->longitude,
             state->timezone,
             state->moon_mode,
             state->night_brightness,
             state->color_temp);

    if(xQueueSend(g_state_queue, state, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGE(TAG, "Failed to send state to queue");
    }
}

static esp_err_t send_json_state(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    if (!root) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON create failed");
        return ESP_FAIL;
    }

    sky_state_t snapshot = get_sky_state_snapshot();
    cJSON_AddStringToObject(root, "mode", mode_to_string(snapshot.mode));
    cJSON_AddNumberToObject(root, "angle", snapshot.angle);
    cJSON_AddNumberToObject(root, "brightness", snapshot.brightness);
    cJSON_AddNumberToObject(root, "max_brightness", snapshot.max_brightness);
    cJSON_AddNumberToObject(root, "latitude", snapshot.latitude);
    cJSON_AddNumberToObject(root, "longitude", snapshot.longitude);
    cJSON_AddNumberToObject(root, "timezone", snapshot.timezone);
    cJSON_AddBoolToObject(root, "moon_mode", snapshot.moon_mode);
    cJSON_AddNumberToObject(root, "night_brightness", snapshot.night_brightness);
    cJSON_AddNumberToObject(root, "color_temp", snapshot.color_temp);
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (!json) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON print failed");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr(req, json);

    free(json);
    return ESP_OK;
}

static esp_err_t index_get_handler(httpd_req_t *req)
{
    const size_t index_html_size = index_html_end - index_html_start;

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_send(req, (const char *)index_html_start, index_html_size);
}

static esp_err_t title_png_get_handler(httpd_req_t *req)
{
    const size_t title_png_size = title_png_end - title_png_start;

    httpd_resp_set_type(req, "image/png");
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=86400");
    return httpd_resp_send(req, (const char *)title_png_start, title_png_size);
}

static esp_err_t api_state_get_handler(httpd_req_t *req)
{
    return send_json_state(req);
}

static esp_err_t api_control_post_handler(httpd_req_t *req)
{
    char buf[768];

    int total_len = req->content_len;
    if (total_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }

    if (total_len >= sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body too large");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Received %d bytes", total_len);

    int received = 0;
    while (received < total_len) {
        int ret = httpd_req_recv(req, buf + received, total_len - received);
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }

            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive failed");
            return ESP_FAIL;
        }

        received += ret;
    }

    buf[received] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *item = NULL;
    sky_state_t new_state = get_sky_state_snapshot();
    item = cJSON_GetObjectItem(root, "mode");
    if (cJSON_IsString(item)) {
        new_state.mode = string_to_mode(item->valuestring);
    }

    item = cJSON_GetObjectItem(root, "angle");
    if (cJSON_IsNumber(item)) {
        new_state.angle = clamp_int(item->valueint, 0, 180);
    }

    item = cJSON_GetObjectItem(root, "brightness");
    if (cJSON_IsNumber(item)) {
        new_state.brightness = clamp_int(item->valueint, 0, 100);
    }

    item = cJSON_GetObjectItem(root, "max_brightness");
    if (cJSON_IsNumber(item)) {
        new_state.max_brightness = clamp_int(item->valueint, 0, 100);
    }

    item = cJSON_GetObjectItem(root, "latitude");
    if (cJSON_IsNumber(item)) {
        if (item->valuedouble >= -90.0 && item->valuedouble <= 90.0) {
            new_state.latitude = item->valuedouble;
        }
    }

    item = cJSON_GetObjectItem(root, "longitude");
    if (cJSON_IsNumber(item)) {
        if (item->valuedouble >= -180.0 && item->valuedouble <= 180.0) {
            new_state.longitude = item->valuedouble;
        }
    }

    item = cJSON_GetObjectItem(root, "timezone");
    if (cJSON_IsNumber(item)) {
        if (item->valuedouble >= -12.0 && item->valuedouble <= 14.0) {
            new_state.timezone = item->valuedouble;
        }
    }

    item = cJSON_GetObjectItem(root, "moon_mode");
    if (cJSON_IsBool(item)) {
        new_state.moon_mode = cJSON_IsTrue(item);
    }

    item = cJSON_GetObjectItem(root, "night_brightness");
    if (cJSON_IsNumber(item)) {
        new_state.night_brightness = clamp_int(item->valueint, 0, 100);
    }

    item = cJSON_GetObjectItem(root, "color_temp");
    if (cJSON_IsNumber(item)) {
        new_state.color_temp = clamp_int(item->valueint, 1700, 6500);
    }

    cJSON_Delete(root);

    apply_sky_state(&new_state);
    set_sky_state_mode(&new_state);

    schedule_save_sky_state_to_nvs();

    return send_json_state(req);
}

static esp_err_t api_control_options_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET,POST,OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static void init_nvs_save_task(void)
{
    if (g_nvs_save_task_handle != NULL) {
        return;
    }

    BaseType_t ret = xTaskCreate(
        nvs_save_task,
        "nvs_save_task",
        4096,
        NULL,
        4,
        &g_nvs_save_task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "failed to create nvs_save_task");
    }
}

void start_web_server(void)
{
    g_state_queue = xQueueCreate(5, sizeof(sky_state_t));
    if (g_state_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create state queue");
        return;
    }

    init_nvs_save_task();
    /*
     * 先从 NVS 加载上次保存的网页配置。
     * 如果没有保存过，则继续使用 g_state 默认值。
     */
    //load_sky_state_from_nvs(&g_state);

    /*
     * 启动后应用一次状态。
     * 这样设备重启后，会恢复到上次的模式和参数。
     */
    //apply_sky_state(&g_state);

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.server_port = 80;
    config.uri_match_fn = httpd_uri_match_wildcard;

    /*
     * 如果页面资源较多，可以适当加大。
     */
    config.max_uri_handlers = 16;
    config.stack_size = 8192;

    httpd_handle_t server = NULL;

    esp_err_t ret = httpd_start(&server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        return;
    }

    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_get_handler,
        .user_ctx = NULL
    };

    httpd_uri_t index_html_uri = {
        .uri = "/index.html",
        .method = HTTP_GET,
        .handler = index_get_handler,
        .user_ctx = NULL
    };

    httpd_uri_t title_png_uri = {
        .uri = "/title.png",
        .method = HTTP_GET,
        .handler = title_png_get_handler,
        .user_ctx = NULL
    };

    httpd_uri_t api_state_uri = {
        .uri = "/api/state",
        .method = HTTP_GET,
        .handler = api_state_get_handler,
        .user_ctx = NULL
    };

    httpd_uri_t api_control_uri = {
        .uri = "/api/control",
        .method = HTTP_POST,
        .handler = api_control_post_handler,
        .user_ctx = NULL
    };

    httpd_uri_t api_control_options_uri = {
        .uri = "/api/control",
        .method = HTTP_OPTIONS,
        .handler = api_control_options_handler,
        .user_ctx = NULL
    };

    httpd_register_uri_handler(server, &index_uri);
    httpd_register_uri_handler(server, &index_html_uri);
    httpd_register_uri_handler(server, &title_png_uri);
    httpd_register_uri_handler(server, &api_state_uri);
    httpd_register_uri_handler(server, &api_control_uri);
    httpd_register_uri_handler(server, &api_control_options_uri);

    ESP_LOGI(TAG, "HTTP server started");
}