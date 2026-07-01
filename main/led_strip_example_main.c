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
    const float CCT_WARM  = 3000.0f;
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
        led_strip_pixels[i * 3 + 1] = led_mix.warm * SUN_LED_BRIGHTNESS_MAX;
        led_strip_pixels[i * 3 + 2] = led_mix.amber * SUN_LED_BRIGHTNESS_MAX;
        }
}

void astro_led_strip_update(int year, int month, int day,
    int hour, int minute, int second,
    double latitude,
    double longitude,
    double timezone){

        AstroHorizontal sun = calculate_sun_horizontal(
            year, month, day,
            hour, minute, second,
            latitude, longitude, timezone
        );
        
        double arc_angle;
        bool ok = calculate_sun_arc_angle(
            year, month, day,
            hour, minute, second,
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
        led_strip_transmit(&leds_wwa);


}

void app_main(void)
{
    ESP_LOGI(TAG, "Create RMT TX channel");

    led_strip_init(&leds_wwa, led_strip_pixels, sizeof(led_strip_pixels));
    makerhub_init();

    float offset = 0;

    double latitude = 22.5431;
    double longitude = 114.0579;
    double timezone = 8.0;

    int year = 2026, month = 7, day = 1;
    int hour = 5, minute = 0, second = 0;

    while (1) {

        astro_led_strip_update(year, month, day, hour, minute, second, latitude, longitude, timezone);
        vTaskDelay(pdMS_TO_TICKS(EXAMPLE_FRAME_DURATION_MS));

        //hour = ( hour < 24 ) ? hour + 1 : 0;
        if(minute == 60){
            minute = 0;
            hour = ( hour < 21 ) ? hour + 1 : 5;
        }else{
            minute++;
        }

        offset = ( offset < 24 ) ? offset + 0.1 : 0;
    }
}
