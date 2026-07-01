/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"

#include "makerhub.h"
#include "astro_calc.h"
#include "web_server.h"

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM      13

#define EXAMPLE_LED_NUMBERS         35

#define EXAMPLE_FRAME_DURATION_MS   20
#define EXAMPLE_ANGLE_INC_FRAME     0.02
#define EXAMPLE_ANGLE_INC_LED       0.3

static const char *TAG = "example";

static uint8_t led_strip_pixels[EXAMPLE_LED_NUMBERS * 3];

static const rmt_symbol_word_t ws2812_zero = {
    .level0 = 1,
    .duration0 = 0.3 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000, // T0H=0.3us
    .level1 = 0,
    .duration1 = 0.9 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000, // T0L=0.9us
};

static const rmt_symbol_word_t ws2812_one = {
    .level0 = 1,
    .duration0 = 0.9 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000, // T1H=0.9us
    .level1 = 0,
    .duration1 = 0.3 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000, // T1L=0.3us
};

//reset defaults to 50uS
static const rmt_symbol_word_t ws2812_reset = {
    .level0 = 1,
    .duration0 = RMT_LED_STRIP_RESOLUTION_HZ / 1000000 * 50 / 2,
    .level1 = 0,
    .duration1 = RMT_LED_STRIP_RESOLUTION_HZ / 1000000 * 50 / 2,
};

static size_t encoder_callback(const void *data, size_t data_size,
                               size_t symbols_written, size_t symbols_free,
                               rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    if (symbols_free < 8) {
        return 0;
    }
    size_t data_pos = symbols_written / 8;
    uint8_t *data_bytes = (uint8_t*)data;
    if (data_pos < data_size) {
        // Encode a byte
        size_t symbol_pos = 0;
        for (int bitmask = 0x80; bitmask != 0; bitmask >>= 1) {
            if (data_bytes[data_pos]&bitmask) {
                symbols[symbol_pos++] = ws2812_one;
            } else {
                symbols[symbol_pos++] = ws2812_zero;
            }
        }
        return symbol_pos;
    } else {
        symbols[0] = ws2812_reset;
        *done = 1; //Indicate end of the transaction.
        return 1; //we only wrote one symbol
    }
}

typedef struct leds_handle{
    rmt_channel_handle_t led_chan;
    rmt_encoder_handle_t simple_encoder;
    rmt_transmit_config_t tx_config;
    uint8_t *ledptr;
    size_t led_size;
} leds_handle_t;

leds_handle_t leds_wwa;

void led_strip_init(leds_handle_t *leds_handle_ptr, uint8_t *ledptr, size_t led_size){

    leds_handle_ptr->ledptr = ledptr;
    leds_handle_ptr->led_size = led_size;
    leds_handle_ptr->led_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = RMT_LED_STRIP_GPIO_NUM,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &leds_handle_ptr->led_chan));

    ESP_LOGI(TAG, "Create simple callback-based encoder");
    leds_handle_ptr->simple_encoder = NULL;
    const rmt_simple_encoder_config_t simple_encoder_cfg = {
        .callback = encoder_callback
        //Note we don't set min_chunk_size here as the default of 64 is good enough.
    };
    ESP_ERROR_CHECK(rmt_new_simple_encoder(&simple_encoder_cfg, &leds_handle_ptr->simple_encoder));

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(leds_handle_ptr->led_chan));

    ESP_LOGI(TAG, "Start LED rainbow chase");
    leds_handle_ptr->tx_config.loop_count = 0;
}

void led_strip_transmit(leds_handle_t *leds_handle_ptr){
    ESP_ERROR_CHECK(rmt_transmit(leds_handle_ptr->led_chan, leds_handle_ptr->simple_encoder, leds_handle_ptr->ledptr, leds_handle_ptr->led_size, &leds_handle_ptr->tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(leds_handle_ptr->led_chan, portMAX_DELAY));
}



static void print_horizontal(const char *name, AstroHorizontal p)
{
    printf("%s\n", name);
    printf("  Altitude   : %8.3f deg\n", p.altitude_deg);
    printf("  Azimuth    : %8.3f deg\n", p.azimuth_deg);
    printf("  Hour angle : %8.3f deg\n", p.hour_angle_deg);
    printf("  Side       : %s\n", east_west_side(p.hour_angle_deg));
    printf("  Visibility : %s\n", visible_state(p.altitude_deg));
}

#define TIME_ANIM_SIGMA             1.2f
#define TIME_ANIM_MIN_FACTOR        0.01f
#define SUN_LED_BRIGHTNESS_MAX      254.0f

static float time_anim_gaussian_factor(float pos, float param)
{
    float diff = pos - param;
    float g = expf(-(diff * diff) / (2.0f * TIME_ANIM_SIGMA * TIME_ANIM_SIGMA));
    return (1.0f - TIME_ANIM_MIN_FACTOR) * g;
}

typedef struct {
    float white;   // 正白，0.0 ~ 1.0
    float warm;    // 暖白，0.0 ~ 1.0
    float amber;   // 橙黄，0.0 ~ 1.0
} SunLedMix;

SunLedMix mix_sun_led_natural(float brightness, float cct)
{
    const float CCT_MIN   = 1700.0f;
    //const float CCT_WARM  = 3000.0f;
    const float CCT_WHITE = 6500.0f;

    SunLedMix out = {0};

    brightness = clampf(brightness, 0.0f, 1.0f);
    cct = clampf(cct, CCT_MIN, CCT_WHITE);

    /*
     * Amber:
     * strongest at low CCT,
     * fades out by about 4200K.
     */
    out.amber = 1.0f - smoothstepf(1800.0f, 4200.0f, cct);

    /*
     * Warm white:
     * rises from low CCT,
     * stays strong in middle,
     * fades at high CCT.
     */
    float warm_in  = smoothstepf(1700.0f, 2800.0f, cct);
    float warm_out = 1.0f - smoothstepf(4200.0f, 6500.0f, cct);
    out.warm = warm_in * warm_out;

    /*
     * White:
     * starts joining after 3500K,
     * strongest at high CCT.
     */
    out.white = smoothstepf(3500.0f, 6500.0f, cct);

    /*
     * Normalize channels so max channel = 1.0.
     * This keeps brightness stable.
     */
    float max_ch = out.amber;
    if (out.warm > max_ch) max_ch = out.warm;
    if (out.white > max_ch) max_ch = out.white;

    if (max_ch > 0.0001f) {
        out.amber /= max_ch;
        out.warm  /= max_ch;
        out.white /= max_ch;
    }

    /*
     * Apply global brightness.
     */
    out.amber *= brightness;
    out.warm  *= brightness;
    out.white *= brightness;

    return out;
}

static float sun_arc_angle_to_led_pos(float arc_angle_deg, int led_num)
{
    if (led_num <= 1) {
        return 0.0f;
    }

    arc_angle_deg = clampf(arc_angle_deg, 0.0f, 180.0f);

    return arc_angle_deg / 180.0f * (float)(led_num - 1);
}

void calculate_sun_led_strip(
    float arc_angle_deg,
    float brightness,
    float cct)
{

    /*
    * 太阳中心位置，允许小数，用于平滑移动。
    */
    float sun_pos = sun_arc_angle_to_led_pos(arc_angle_deg, EXAMPLE_LED_NUMBERS);

    /*
    * 基础三通道颜色。
    */
    SunLedMix base_mix = mix_sun_led_natural(1.0f, cct);

    brightness = clampf(brightness, 0.0f, 1.0f);

    for (int i = 0; i < EXAMPLE_LED_NUMBERS; i++) {
        /*
        * 高斯扩散权重。
        * i 越接近 sun_pos，factor 越大。
        */
        float factor = time_anim_gaussian_factor((float)i, sun_pos);

        /*
        * 当前 LED 的最终亮度。
        */
        float led_brightness = brightness * factor;

        SunLedMix led_mix;

        led_mix.white = base_mix.white * led_brightness;
        led_mix.warm  = base_mix.warm  * led_brightness;
        led_mix.amber = base_mix.amber * led_brightness;

        led_strip_pixels[i * 3 + 0] = led_mix.white * SUN_LED_BRIGHTNESS_MAX;
        led_strip_pixels[i * 3 + 2] = led_mix.warm * SUN_LED_BRIGHTNESS_MAX;
        led_strip_pixels[i * 3 + 1] = led_mix.amber * SUN_LED_BRIGHTNESS_MAX;
    }
}

void astro_led_strip_update(struct tm *local_time,
    double latitude,
    double longitude,
    double timezone)
{
        AstroHorizontal sun = calculate_sun_horizontal(
            local_time->tm_year, local_time->tm_mon, local_time->tm_mday,
            local_time->tm_hour, local_time->tm_min, local_time->tm_sec,
            latitude, longitude, timezone
        );
        
        double arc_angle;
        bool ok = calculate_sun_arc_angle(
            local_time->tm_year, local_time->tm_mon, local_time->tm_mday,
            local_time->tm_hour, local_time->tm_min, local_time->tm_sec,
            latitude, longitude, timezone,
            &arc_angle
        );
        
        float brightness = (float)sun_brightness_from_altitude(
            sun.altitude_deg
        );
        
        float cct = (float)sun_cct_with_morning_evening(
            sun.altitude_deg,
            sun.hour_angle_deg
        );
        
        calculate_sun_led_strip(
            (float)arc_angle,
            brightness,
            cct
        );
}

void led_strip_update_fixed( float brightness_max,float angle ){

    angle = clampf(angle, 0.0f, 180.0f);
    float altitude = (angle > 90.0f) ? (180.0 - angle) : angle;
    float hour_angle = angle - 90.0f;

    float brightness = (float)sun_brightness_from_altitude(altitude);
    float cct = (float)sun_cct_with_morning_evening(altitude,hour_angle);
    
    brightness_max = clampf(brightness_max, 0.0f, 1.0f);
    calculate_sun_led_strip(
        angle,
        brightness_max * brightness,
        cct
    );
}

void led_strip_update_night( float brightness,float cct ){

    brightness = clampf(brightness, 0.0f, 1.0f);
    SunLedMix base_mix = mix_sun_led_natural(brightness, cct);

    for (int i = 0; i < EXAMPLE_LED_NUMBERS; i++) {

        SunLedMix led_mix;

        led_mix.white = base_mix.white * brightness;
        led_mix.warm  = base_mix.warm  * brightness;
        led_mix.amber = base_mix.amber * brightness;

        led_strip_pixels[i * 3 + 0] = led_mix.white * SUN_LED_BRIGHTNESS_MAX;
        led_strip_pixels[i * 3 + 2] = led_mix.warm * SUN_LED_BRIGHTNESS_MAX;
        led_strip_pixels[i * 3 + 1] = led_mix.amber * SUN_LED_BRIGHTNESS_MAX;
    }
}

typedef enum {
    WIFI_STATE_DISCONNECTED = 0,
    WIFI_STATE_CONNECTING,
    WIFI_STATE_CONNECTED,
    WIFI_STATE_FAILED,
} wifi_state_t;

typedef struct {
    sky_state_t sky_state;
    time_t local_now;
    struct tm local_time;
    wifi_state_t wifi_state;
    makerhub_ntp_state_t ntp_state;
    bool initialized;
} astro_run_state_t;

astro_run_state_t astro_run_state;

void init_state_from_nvs(void){
    sky_state_t state;
    if(load_sky_state_from_nvs(&state) != ESP_OK){
        ESP_LOGE(TAG, "Failed to load state from NVS");
    }
    memcpy(&astro_run_state.sky_state, &state, sizeof(sky_state_t));
    ESP_ERROR_CHECK(makerhub_ntp_set_timezone(astro_run_state.sky_state.timezone));
    astro_run_state.wifi_state = WIFI_STATE_DISCONNECTED;
    astro_run_state.ntp_state = MAKERHUB_NTP_STATE_STOPPED;
    astro_run_state.initialized = true;

    astro_run_state.local_time.tm_year = 2026;
    astro_run_state.local_time.tm_mon = 7;
    astro_run_state.local_time.tm_mday = 1;
    astro_run_state.local_time.tm_hour = 5;
    astro_run_state.local_time.tm_min = 0;
    astro_run_state.local_time.tm_sec = 0;
    astro_run_state.local_now = mktime(&astro_run_state.local_time)+astro_run_state.sky_state.timezone*3600;

}

void wifi_connected_callback(void *user_ctx){
    ESP_LOGI(TAG, "WiFi connected");
    astro_run_state.wifi_state = WIFI_STATE_CONNECTED;
}

void wifi_disconnected_callback(void *user_ctx){
    ESP_LOGI(TAG, "WiFi disconnected");
    astro_run_state.wifi_state = WIFI_STATE_DISCONNECTED;
}

void wifi_got_ip_callback(const char *ip, void *user_ctx){
    ESP_LOGI(TAG, "WiFi got IP: %s", ip);
    astro_run_state.wifi_state = WIFI_STATE_CONNECTED;
    start_web_server();
}

void wifi_fail_callback(wifi_err_reason_t reason, void *user_ctx){
    ESP_LOGI(TAG, "WiFi failed: %s", esp_err_to_name(reason));
    astro_run_state.wifi_state = WIFI_STATE_FAILED;
}

void astro_run_task(void *pvParameter){

    bool state_flush = false;
    QueueHandle_t state_queue = get_state_queue();
    if(state_queue == NULL){
        ESP_LOGE(TAG, "Failed to get state queue");
        //return;
    }
    while(1){
        state_queue = get_state_queue();
        if (state_queue != NULL) {
            if(xQueueReceive(state_queue, &astro_run_state.sky_state, 0) == pdPASS){
                state_flush = true;
                ESP_LOGI(TAG, "State flushed");
            }
        }

        if( astro_run_state.sky_state.mode == SKY_MODE_TIME ){

        }
        else if( astro_run_state.sky_state.mode == SKY_MODE_FIXED && state_flush){
            led_strip_update_fixed(astro_run_state.sky_state.brightness / 100.0f, 180.0 - astro_run_state.sky_state.angle);
            state_flush = false;
            led_strip_transmit(&leds_wwa);
        }
        else if( astro_run_state.sky_state.mode == SKY_MODE_NIGHT && state_flush){
            led_strip_update_night(astro_run_state.sky_state.night_brightness / 100.0f, astro_run_state.sky_state.color_temp);
            state_flush = false;
            led_strip_transmit(&leds_wwa);
        }
        else if( astro_run_state.sky_state.mode == SKY_MODE_OFF && state_flush){
            memset(leds_wwa.ledptr, 0, leds_wwa.led_size);
            state_flush = false;
            led_strip_transmit(&leds_wwa);
        }
        
        vTaskDelay(pdMS_TO_TICKS(EXAMPLE_FRAME_DURATION_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Create RMT TX channel");

    led_strip_init(&leds_wwa, led_strip_pixels, sizeof(led_strip_pixels));

    makerhub_wifi_event_callbacks_t wifi_callbacks = {
        .on_disconnected = wifi_disconnected_callback,
        .on_connecting = NULL,
        .on_got_ip = wifi_got_ip_callback,
        .on_connected = wifi_connected_callback,
        .on_connect_failed = wifi_fail_callback,
        .user_ctx = NULL
    };
    makerhub_wifi_set_event_callbacks(&wifi_callbacks);

    makerhub_init();

    float offset = 0;

    int year = 2026, month = 7, day = 1;
    int hour = 5, minute = 0, second = 0;

    if(!astro_run_state.initialized){
        init_state_from_nvs();
        set_sky_state(&astro_run_state.sky_state);
    }
    astro_run_task(NULL);
    //xTaskCreate(astro_run_task, "astro_run_task", 4096, NULL, 5, NULL);
    while (1) {
        QueueHandle_t state_queue = get_state_queue();
        if (state_queue != NULL) {
            sky_state_t new_state;
            while (xQueueReceive(state_queue, &new_state, 0) == pdPASS) {
                //esp_err_t tz_ret = makerhub_ntp_set_timezone(state.timezone);
                //if (tz_ret != ESP_OK) {
                //    ESP_LOGW(TAG, "Invalid timezone %.2f: %s", state.timezone, esp_err_to_name(tz_ret));
                //}
            }
        }

        time_t local_now = 0;
        if (makerhub_ntp_get_local_time(&local_now, NULL) == ESP_OK) {
            struct tm local_time = {0};
            gmtime_r(&local_now, &local_time);

            year = local_time.tm_year + 1900;
            month = local_time.tm_mon + 1;
            day = local_time.tm_mday;
            hour = local_time.tm_hour;
            minute = local_time.tm_min;
            second = local_time.tm_sec;
        }

        astro_led_strip_update(&astro_run_state.local_time, 
                                astro_run_state.sky_state.latitude, astro_run_state.sky_state.longitude, 
                                astro_run_state.sky_state.timezone);

        led_strip_transmit(&leds_wwa);
        vTaskDelay(pdMS_TO_TICKS(EXAMPLE_FRAME_DURATION_MS));

        if (!makerhub_ntp_is_synced()) {
            if(minute == 60){
                minute = 0;
                hour = ( hour < 21 ) ? hour + 1 : 5;
            }else{
                minute++;
            }
        }
        offset = ( offset < 24 ) ? offset + 0.1 : 0;
    }
}
