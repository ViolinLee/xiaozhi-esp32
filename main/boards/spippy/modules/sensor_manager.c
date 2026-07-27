#include "modules/sensor_manager.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/mcpwm_cap.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"
#include "spippy_pins.h"

static const char *TAG = "SensorManager";
static const uint32_t SENSOR_MANAGER_US_MIN_ECHO_PULSE_US = 100U;

typedef struct {
    bool measurement_valid;
    uint32_t distance_mm;
    uint32_t last_pulse_width_us;
    uint32_t tick_count;
    uint32_t sample_count;
    uint32_t timeout_count;
    uint32_t error_count;
    char last_error[32];
    mcpwm_cap_timer_handle_t capture_timer;
    mcpwm_cap_channel_handle_t capture_channel;
    uint32_t capture_timer_resolution_hz;
    uint32_t capture_start_ticks;
    bool capture_rise_seen;
    TaskHandle_t notify_task;
} spippy_ultrasonic_state_t;

static spippy_ultrasonic_state_t s_ultrasonic;
static portMUX_TYPE s_ultrasonic_trigger_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_ultrasonic_status_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_sensor_task;

static void sensor_manager_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        sensor_manager_tick();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONFIG_SPIPPY_TASK_SENSOR_PERIOD_MS));
    }
}

static bool sensor_manager_echo_capture_callback(mcpwm_cap_channel_handle_t cap_channel,
                                                 const mcpwm_capture_event_data_t *edata,
                                                 void *user_data)
{
    (void)cap_channel;

    spippy_ultrasonic_state_t *state = (spippy_ultrasonic_state_t *)user_data;
    BaseType_t high_task_wakeup = pdFALSE;

    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
        state->capture_start_ticks = edata->cap_value;
        state->capture_rise_seen = true;
    } else if (state->capture_rise_seen && (state->notify_task != NULL)) {
        uint32_t pulse_ticks = edata->cap_value - state->capture_start_ticks;
        state->capture_rise_seen = false;
        xTaskNotifyFromISR(state->notify_task, pulse_ticks, eSetValueWithOverwrite, &high_task_wakeup);
    }

    return high_task_wakeup == pdTRUE;
}

static esp_err_t sensor_manager_measure_distance_mm(uint32_t *distance_mm_out, uint32_t *pulse_width_us_out)
{
    ESP_RETURN_ON_FALSE(distance_mm_out != NULL, ESP_ERR_INVALID_ARG, TAG, "distance_mm_out is null");
    ESP_RETURN_ON_FALSE(pulse_width_us_out != NULL, ESP_ERR_INVALID_ARG, TAG, "pulse_width_us_out is null");
    ESP_RETURN_ON_FALSE((s_ultrasonic.capture_timer != NULL) &&
                        (s_ultrasonic.capture_channel != NULL) &&
                        (s_ultrasonic.capture_timer_resolution_hz > 0U),
                        ESP_ERR_INVALID_STATE,
                        TAG,
                        "ultrasonic capture is not ready");

    uint32_t stale_ticks = 0;
    (void)xTaskNotifyWait(0, UINT32_MAX, &stale_ticks, 0);
    s_ultrasonic.notify_task = xTaskGetCurrentTaskHandle();
    s_ultrasonic.capture_rise_seen = false;

    portENTER_CRITICAL(&s_ultrasonic_trigger_mux);
    gpio_set_level(SPIPPY_PIN_US_TRIG, 0);
    esp_rom_delay_us(2);
    gpio_set_level(SPIPPY_PIN_US_TRIG, 1);
    esp_rom_delay_us(CONFIG_SPIPPY_SENSOR_US_TRIGGER_PULSE_US);
    gpio_set_level(SPIPPY_PIN_US_TRIG, 0);
    portEXIT_CRITICAL(&s_ultrasonic_trigger_mux);

    uint32_t pulse_ticks = 0;
    const TickType_t wait_ticks = pdMS_TO_TICKS((CONFIG_SPIPPY_SENSOR_US_ECHO_TIMEOUT_US / 1000U) + 20U);
    if (xTaskNotifyWait(0, UINT32_MAX, &pulse_ticks, wait_ticks) != pdTRUE) {
        s_ultrasonic.notify_task = NULL;
        return ESP_ERR_TIMEOUT;
    }
    s_ultrasonic.notify_task = NULL;

    uint32_t pulse_width_us = (uint32_t)(((uint64_t)pulse_ticks * 1000000ULL) /
                                         s_ultrasonic.capture_timer_resolution_hz);
    *pulse_width_us_out = pulse_width_us;

    if (pulse_width_us < SENSOR_MANAGER_US_MIN_ECHO_PULSE_US) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (pulse_width_us > CONFIG_SPIPPY_SENSOR_US_ECHO_TIMEOUT_US) {
        return ESP_ERR_TIMEOUT;
    }

    *distance_mm_out = (pulse_width_us * 343U) / 2000U;
    return ESP_OK;
}

static esp_err_t sensor_manager_init_ultrasonic_capture(void)
{
    mcpwm_capture_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_capture_timer(&timer_config, &s_ultrasonic.capture_timer),
                        TAG,
                        "failed to create ultrasonic capture timer");

    mcpwm_capture_channel_config_t channel_config = {
        .gpio_num = SPIPPY_PIN_US_ECHO,
        .prescale = 1,
        .flags.pos_edge = true,
        .flags.neg_edge = true,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_capture_channel(s_ultrasonic.capture_timer,
                                                  &channel_config,
                                                  &s_ultrasonic.capture_channel),
                        TAG,
                        "failed to create ultrasonic capture channel");

    mcpwm_capture_event_callbacks_t callbacks = {
        .on_cap = sensor_manager_echo_capture_callback,
    };
    ESP_RETURN_ON_ERROR(mcpwm_capture_channel_register_event_callbacks(s_ultrasonic.capture_channel,
                                                                       &callbacks,
                                                                       &s_ultrasonic),
                        TAG,
                        "failed to register ultrasonic capture callback");
    ESP_RETURN_ON_ERROR(mcpwm_capture_channel_enable(s_ultrasonic.capture_channel),
                        TAG,
                        "failed to enable ultrasonic capture channel");
    ESP_RETURN_ON_ERROR(mcpwm_capture_timer_enable(s_ultrasonic.capture_timer),
                        TAG,
                        "failed to enable ultrasonic capture timer");
    ESP_RETURN_ON_ERROR(mcpwm_capture_timer_start(s_ultrasonic.capture_timer),
                        TAG,
                        "failed to start ultrasonic capture timer");
    ESP_RETURN_ON_ERROR(mcpwm_capture_timer_get_resolution(s_ultrasonic.capture_timer,
                                                           &s_ultrasonic.capture_timer_resolution_hz),
                        TAG,
                        "failed to read ultrasonic capture timer resolution");
    return ESP_OK;
}

esp_err_t sensor_manager_init(void)
{
#if !CONFIG_SPIPPY_SENSOR_ULTRASONIC_ENABLE
    ESP_LOGI(TAG, "ultrasonic disabled by Kconfig");
    return ESP_OK;
#else
    gpio_config_t trig_cfg = {
        .pin_bit_mask = (1ULL << SPIPPY_PIN_US_TRIG),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&trig_cfg), TAG, "failed to configure ultrasonic trig");

    gpio_set_level(SPIPPY_PIN_US_TRIG, 0);
    ESP_RETURN_ON_ERROR(sensor_manager_init_ultrasonic_capture(), TAG, "failed to initialize ultrasonic capture");

    ESP_LOGI(TAG,
             "initialized ultrasonic path trig=%d echo=%d capture_resolution=%" PRIu32 "Hz min_echo=%" PRIu32 "us",
             SPIPPY_PIN_US_TRIG,
             SPIPPY_PIN_US_ECHO,
             s_ultrasonic.capture_timer_resolution_hz,
             SENSOR_MANAGER_US_MIN_ECHO_PULSE_US);
    return ESP_OK;
#endif
}

esp_err_t sensor_manager_start(void)
{
#if CONFIG_SPIPPY_SENSOR_ULTRASONIC_ENABLE
    if (s_sensor_task == NULL) {
        BaseType_t created = xTaskCreate(sensor_manager_task, "spippy_sensor", 3072, NULL, 3, &s_sensor_task);
        ESP_RETURN_ON_FALSE(created == pdPASS, ESP_ERR_NO_MEM, TAG, "failed to create sensor task");
    }
#endif
    ESP_LOGI(TAG, "ultrasonic sensor ready");
    return ESP_OK;
}

esp_err_t sensor_manager_get_ultrasonic_reading(spippy_ultrasonic_reading_t *reading)
{
    if (reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_ultrasonic_status_mux);
    reading->measurement_valid = s_ultrasonic.measurement_valid;
    reading->distance_mm = s_ultrasonic.distance_mm;
    reading->sample_count = s_ultrasonic.sample_count;
    reading->timeout_count = s_ultrasonic.timeout_count;
    reading->error_count = s_ultrasonic.error_count;
    snprintf(reading->last_error, sizeof(reading->last_error), "%s", s_ultrasonic.last_error);
    portEXIT_CRITICAL(&s_ultrasonic_status_mux);
    return ESP_OK;
}

esp_err_t sensor_manager_tick(void)
{
#if !CONFIG_SPIPPY_SENSOR_ULTRASONIC_ENABLE
    return ESP_OK;
#else
    uint32_t distance_mm = 0;
    uint32_t pulse_width_us = 0;
    esp_err_t err = sensor_manager_measure_distance_mm(&distance_mm, &pulse_width_us);
    bool should_log = false;
    bool measurement_valid = false;
    uint32_t logged_distance_mm = 0;
    uint32_t logged_pulse_width_us = 0;
    char logged_error[32] = {};

    portENTER_CRITICAL(&s_ultrasonic_status_mux);
    s_ultrasonic.tick_count++;
    s_ultrasonic.sample_count++;

    if (err == ESP_OK) {
        s_ultrasonic.measurement_valid = true;
        s_ultrasonic.distance_mm = distance_mm;
        s_ultrasonic.last_pulse_width_us = pulse_width_us;
        s_ultrasonic.last_error[0] = '\0';
    } else if (err == ESP_ERR_TIMEOUT) {
        s_ultrasonic.measurement_valid = false;
        s_ultrasonic.last_pulse_width_us = pulse_width_us;
        s_ultrasonic.timeout_count++;
        snprintf(s_ultrasonic.last_error, sizeof(s_ultrasonic.last_error), "timeout");
    } else {
        s_ultrasonic.measurement_valid = false;
        s_ultrasonic.last_pulse_width_us = pulse_width_us;
        s_ultrasonic.error_count++;
        snprintf(s_ultrasonic.last_error, sizeof(s_ultrasonic.last_error), "%s", esp_err_to_name(err));
    }

    should_log = (s_ultrasonic.tick_count % CONFIG_SPIPPY_SENSOR_LOG_INTERVAL) == 0U;
    measurement_valid = s_ultrasonic.measurement_valid;
    logged_distance_mm = s_ultrasonic.distance_mm;
    logged_pulse_width_us = s_ultrasonic.last_pulse_width_us;
    snprintf(logged_error, sizeof(logged_error), "%s", s_ultrasonic.last_error);
    portEXIT_CRITICAL(&s_ultrasonic_status_mux);

    if (should_log) {
        if (measurement_valid) {
            ESP_LOGI(TAG, "ultrasonic distance=%" PRIu32 "mm", logged_distance_mm);
        } else {
            ESP_LOGW(TAG,
                     "ultrasonic measurement failed: %s last_pulse=%" PRIu32 "us",
                     logged_error,
                     logged_pulse_width_us);
        }
    }

    return ESP_OK;
#endif
}
