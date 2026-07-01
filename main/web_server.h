#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SKY_MODE_FIXED = 0,
    SKY_MODE_TIME,
    SKY_MODE_NIGHT,
    SKY_MODE_OFF
} sky_mode_t;

typedef struct {
    sky_mode_t mode;

    int angle;              // 0 - 180
    int brightness;         // 0 - 100

    int max_brightness;     // 0 - 100
    double latitude;
    double longitude;
    double timezone;
    bool moon_mode;

    int night_brightness;   // 0 - 100
    int color_temp;         // 1700 - 6500
} sky_state_t;

void start_web_server(void);
QueueHandle_t get_state_queue(void);
esp_err_t load_sky_state_from_nvs(sky_state_t *state);
void set_sky_state(sky_state_t *state);

#ifdef __cplusplus
}
#endif