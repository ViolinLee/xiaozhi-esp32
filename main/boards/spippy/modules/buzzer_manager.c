#include "modules/buzzer_manager.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"

/* depends-on 的 Kconfig 整数在蜂鸣器关闭时不会生成宏，仍需保证本文件可编译。 */
#ifndef CONFIG_SPIPPY_BUZZER_ENABLE
#define CONFIG_SPIPPY_BUZZER_ENABLE 0
#endif
#ifndef CONFIG_SPIPPY_BUZZER_GPIO
#define CONFIG_SPIPPY_BUZZER_GPIO 45
#endif
#ifndef CONFIG_SPIPPY_BUZZER_LOW_POWER_ALERT_INITIAL_DELAY_MS
#define CONFIG_SPIPPY_BUZZER_LOW_POWER_ALERT_INITIAL_DELAY_MS 8000
#endif
#ifndef CONFIG_SPIPPY_BUZZER_LOW_POWER_ALERT_INTERVAL_MS
#define CONFIG_SPIPPY_BUZZER_LOW_POWER_ALERT_INTERVAL_MS 10000
#endif
#ifndef CONFIG_SPIPPY_TASK_BUZZER_PERIOD_MS
#define CONFIG_SPIPPY_TASK_BUZZER_PERIOD_MS 10
#endif

static const char *TAG = "BuzzerManager";

typedef struct {
    uint16_t frequency_hz;
    uint16_t duration_ms;
    uint16_t pause_ms;
} buzzer_segment_t;

static const buzzer_segment_t s_low_power_pattern[] = {
    {460, 90, 90},
    {460, 90, 90},
    {460, 90, 0},
};

typedef struct {
    spippy_buzzer_status_t status;
    esp_timer_handle_t tone_timer;
    TaskHandle_t task;
    size_t segment_index;
    uint64_t segment_end_us;
    uint64_t pause_end_us;
    bool output_high;
    bool in_pause;
    bool alert_enabled;
    uint64_t next_alert_us;
} buzzer_state_t;

static buzzer_state_t s_buzzer;
static portMUX_TYPE s_state_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_alert_mux = portMUX_INITIALIZER_UNLOCKED;

static void buzzer_set_error(const char *message)
{
    portENTER_CRITICAL(&s_state_mux);
    strlcpy(s_buzzer.status.last_error, message != NULL ? message : "",
            sizeof(s_buzzer.status.last_error));
    portEXIT_CRITICAL(&s_state_mux);
}

static void buzzer_output(bool high)
{
    gpio_set_level(CONFIG_SPIPPY_BUZZER_GPIO, high ? 1 : 0);
    portENTER_CRITICAL(&s_state_mux);
    s_buzzer.output_high = high;
    portEXIT_CRITICAL(&s_state_mux);
}

static void buzzer_tone_timer_cb(void *arg)
{
    (void)arg;
    bool next_level = false;
    bool should_toggle = false;
    portENTER_CRITICAL(&s_state_mux);
    if (s_buzzer.status.enabled && s_buzzer.status.busy) {
        s_buzzer.output_high = !s_buzzer.output_high;
        next_level = s_buzzer.output_high;
        should_toggle = true;
    } else {
        s_buzzer.output_high = false;
    }
    portEXIT_CRITICAL(&s_state_mux);
    gpio_set_level(CONFIG_SPIPPY_BUZZER_GPIO, should_toggle && next_level ? 1 : 0);
}

static void buzzer_stop_pattern(void)
{
    if (s_buzzer.tone_timer != NULL) {
        (void)esp_timer_stop(s_buzzer.tone_timer);
    }
    gpio_set_level(CONFIG_SPIPPY_BUZZER_GPIO, 0);
    portENTER_CRITICAL(&s_state_mux);
    s_buzzer.segment_index = 0;
    s_buzzer.segment_end_us = 0;
    s_buzzer.pause_end_us = 0;
    s_buzzer.output_high = false;
    s_buzzer.in_pause = false;
    s_buzzer.status.busy = false;
    portEXIT_CRITICAL(&s_state_mux);
}

static esp_err_t buzzer_start_segment(uint64_t now_us)
{
    const buzzer_segment_t *segment = &s_low_power_pattern[s_buzzer.segment_index];
    const uint32_t half_period_us = 1000000U / ((uint32_t)segment->frequency_hz * 2U);
    (void)esp_timer_stop(s_buzzer.tone_timer);
    gpio_set_level(CONFIG_SPIPPY_BUZZER_GPIO, 0);
    portENTER_CRITICAL(&s_state_mux);
    s_buzzer.output_high = false;
    s_buzzer.in_pause = false;
    s_buzzer.segment_end_us = now_us + ((uint64_t)segment->duration_ms * 1000ULL);
    s_buzzer.pause_end_us = 0;
    portEXIT_CRITICAL(&s_state_mux);
    return esp_timer_start_periodic(s_buzzer.tone_timer, half_period_us);
}

static esp_err_t buzzer_begin_low_power_pattern(uint64_t now_us)
{
    portENTER_CRITICAL(&s_state_mux);
    if (!s_buzzer.status.enabled || !s_buzzer.status.ready) {
        portEXIT_CRITICAL(&s_state_mux);
        return ESP_OK;
    }
    s_buzzer.segment_index = 0;
    s_buzzer.status.busy = true;
    strlcpy(s_buzzer.status.last_pattern, "LOW_POWER", sizeof(s_buzzer.status.last_pattern));
    s_buzzer.status.last_error[0] = '\0';
    portEXIT_CRITICAL(&s_state_mux);
    esp_err_t err = buzzer_start_segment(now_us);
    if (err != ESP_OK) {
        buzzer_stop_pattern();
        buzzer_set_error(esp_err_to_name(err));
    }
    return err;
}

static void buzzer_tick(void)
{
    const uint64_t now_us = (uint64_t)esp_timer_get_time();
    bool alert_enabled = false;
    uint64_t next_alert_us = 0;
    portENTER_CRITICAL(&s_alert_mux);
    alert_enabled = s_buzzer.alert_enabled;
    next_alert_us = s_buzzer.next_alert_us;
    portEXIT_CRITICAL(&s_alert_mux);

    bool busy = false;
    bool in_pause = false;
    uint64_t segment_end_us = 0;
    uint64_t pause_end_us = 0;
    portENTER_CRITICAL(&s_state_mux);
    busy = s_buzzer.status.busy;
    in_pause = s_buzzer.in_pause;
    segment_end_us = s_buzzer.segment_end_us;
    pause_end_us = s_buzzer.pause_end_us;
    portEXIT_CRITICAL(&s_state_mux);

    if (!alert_enabled) {
        if (busy) {
            buzzer_stop_pattern();
        }
        return;
    }

    if (!busy) {
        if (next_alert_us > 0 && now_us >= next_alert_us) {
            portENTER_CRITICAL(&s_alert_mux);
            if (s_buzzer.alert_enabled && s_buzzer.next_alert_us == next_alert_us) {
                s_buzzer.next_alert_us =
                    now_us + ((uint64_t)CONFIG_SPIPPY_BUZZER_LOW_POWER_ALERT_INTERVAL_MS * 1000ULL);
            }
            portEXIT_CRITICAL(&s_alert_mux);
            (void)buzzer_begin_low_power_pattern(now_us);
        }
        return;
    }

    if (in_pause) {
        if (now_us < pause_end_us) {
            return;
        }
        if (s_buzzer.segment_index >= sizeof(s_low_power_pattern) / sizeof(s_low_power_pattern[0])) {
            buzzer_stop_pattern();
        } else if (buzzer_start_segment(now_us) != ESP_OK) {
            buzzer_stop_pattern();
            buzzer_set_error("failed to start buzzer segment");
        }
        return;
    }

    if (now_us < segment_end_us) {
        return;
    }

    (void)esp_timer_stop(s_buzzer.tone_timer);
    buzzer_output(false);
    const buzzer_segment_t *completed = &s_low_power_pattern[s_buzzer.segment_index];
    s_buzzer.segment_index++;
    if (completed->pause_ms > 0) {
        portENTER_CRITICAL(&s_state_mux);
        s_buzzer.in_pause = true;
        s_buzzer.pause_end_us = now_us + ((uint64_t)completed->pause_ms * 1000ULL);
        portEXIT_CRITICAL(&s_state_mux);
    } else if (s_buzzer.segment_index >= sizeof(s_low_power_pattern) / sizeof(s_low_power_pattern[0])) {
        buzzer_stop_pattern();
    } else if (buzzer_start_segment(now_us) != ESP_OK) {
        buzzer_stop_pattern();
        buzzer_set_error("failed to start buzzer segment");
    }
}

static void buzzer_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        buzzer_tick();
        TickType_t delay_ticks = pdMS_TO_TICKS(CONFIG_SPIPPY_TASK_BUZZER_PERIOD_MS);
        /* 100 Hz tick 下 5 ms 会被截断为 0；至少让出一个 tick，避免低电任务拖死系统。 */
        vTaskDelayUntil(&last_wake, delay_ticks > 0 ? delay_ticks : 1);
    }
}

esp_err_t buzzer_manager_init(void)
{
    memset(&s_buzzer, 0, sizeof(s_buzzer));
    s_buzzer.status.enabled = CONFIG_SPIPPY_BUZZER_ENABLE;
    strlcpy(s_buzzer.status.last_pattern, "NONE", sizeof(s_buzzer.status.last_pattern));
    if (!s_buzzer.status.enabled) {
        s_buzzer.status.ready = true;
        ESP_LOGI(TAG, "disabled by Kconfig");
        return ESP_OK;
    }

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << CONFIG_SPIPPY_BUZZER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "failed to configure buzzer gpio");
    gpio_set_level(CONFIG_SPIPPY_BUZZER_GPIO, 0);

    esp_timer_create_args_t timer_args = {
        .callback = buzzer_tone_timer_cb,
        .name = "spippy_buzzer",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &s_buzzer.tone_timer), TAG,
                        "failed to create buzzer timer");
    s_buzzer.status.ready = true;
    ESP_LOGI(TAG, "ready gpio=%d", CONFIG_SPIPPY_BUZZER_GPIO);
    return ESP_OK;
}

esp_err_t buzzer_manager_start(void)
{
    if (!s_buzzer.status.enabled) {
        return ESP_OK;
    }
    if (!s_buzzer.status.ready || s_buzzer.tone_timer == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_buzzer.task != NULL) {
        return ESP_OK;
    }
    BaseType_t created = xTaskCreate(buzzer_task, "spippy_buzzer", 2048, NULL, 3, &s_buzzer.task);
    if (created != pdPASS) {
        s_buzzer.status.ready = false;
        buzzer_set_error("failed to create buzzer task");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t buzzer_manager_set_low_power_alert(bool enabled)
{
    if (!s_buzzer.status.ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_buzzer.status.enabled) {
        return ESP_OK;
    }
    portENTER_CRITICAL(&s_alert_mux);
    if (s_buzzer.alert_enabled == enabled) {
        portEXIT_CRITICAL(&s_alert_mux);
        return ESP_OK;
    }
    s_buzzer.alert_enabled = enabled;
    s_buzzer.next_alert_us = enabled ?
        (uint64_t)esp_timer_get_time() +
            ((uint64_t)CONFIG_SPIPPY_BUZZER_LOW_POWER_ALERT_INITIAL_DELAY_MS * 1000ULL) :
        0;
    portEXIT_CRITICAL(&s_alert_mux);

    portENTER_CRITICAL(&s_state_mux);
    strlcpy(s_buzzer.status.last_pattern, enabled ? "LOW_POWER_ARMED" : "NONE",
            sizeof(s_buzzer.status.last_pattern));
    portEXIT_CRITICAL(&s_state_mux);
    return ESP_OK;
}

esp_err_t buzzer_manager_get_status(spippy_buzzer_status_t *status)
{
    ESP_RETURN_ON_FALSE(status != NULL, ESP_ERR_INVALID_ARG, TAG, "status is null");
    portENTER_CRITICAL(&s_state_mux);
    *status = s_buzzer.status;
    portEXIT_CRITICAL(&s_state_mux);
    return ESP_OK;
}
