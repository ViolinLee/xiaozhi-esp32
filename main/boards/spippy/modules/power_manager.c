#include "modules/power_manager.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"

#include "sdkconfig.h"
#include "spippy_pins.h"

static const char *TAG = "PowerManager";

#define SPIPPY_BAT_ADC_UNIT       ADC_UNIT_1
#define SPIPPY_BAT_ADC_CHANNEL    ADC_CHANNEL_7
#define SPIPPY_BAT_ADC_ATTEN      ADC_ATTEN_DB_12

typedef struct {
    int last_raw;
    int last_pin_mv;
    int last_pack_mv;
    int filtered_pack_mv;
    uint32_t tick_count;
    uint32_t low_streak;
    uint32_t recover_streak;
} spippy_power_sample_state_t;

static spippy_runtime_status_t s_power_status = {
    .system_state = SPIPPY_SYSTEM_STATE_BOOTING,
    .active_action = SPIPPY_ACTION_IDLE,
    .active_gait = {
        .gait = SPIPPY_GAIT_TROT,
        .speed_level = SPIPPY_SPEED_LEVEL_LOW,
    },
    .emergency_stop = false,
    .low_power_latched = false,
};
static spippy_power_sample_state_t s_sample_state;
static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_adc_cali_handle;
static bool s_adc_calibration_ready;

static bool power_manager_adc_calibration_init(adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = SPIPPY_BAT_ADC_UNIT,
        .chan = SPIPPY_BAT_ADC_CHANNEL,
        .atten = SPIPPY_BAT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
    calibrated = (ret == ESP_OK);
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = SPIPPY_BAT_ADC_UNIT,
            .atten = SPIPPY_BAT_ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        calibrated = (ret == ESP_OK);
    }
#endif

    *out_handle = handle;
    if (calibrated) {
        ESP_LOGI(TAG, "adc calibration ready");
    } else {
        ESP_LOGW(TAG, "adc calibration unavailable, using fallback conversion");
    }
    return calibrated;
}

static int power_manager_raw_to_pin_mv(int raw)
{
    if (s_adc_calibration_ready) {
        int voltage_mv = 0;
        if (adc_cali_raw_to_voltage(s_adc_cali_handle, raw, &voltage_mv) == ESP_OK) {
            return voltage_mv;
        }
        ESP_LOGW(TAG, "adc calibration convert failed, falling back to linear estimate");
    }

    return (raw * CONFIG_SPIPPY_POWER_ADC_FALLBACK_FULL_SCALE_MV) / 4095;
}

static esp_err_t power_manager_sample_voltage(int *raw_out, int *pin_mv_out, int *pack_mv_out)
{
    ESP_RETURN_ON_FALSE(raw_out != NULL, ESP_ERR_INVALID_ARG, TAG, "raw_out is null");
    ESP_RETURN_ON_FALSE(pin_mv_out != NULL, ESP_ERR_INVALID_ARG, TAG, "pin_mv_out is null");
    ESP_RETURN_ON_FALSE(pack_mv_out != NULL, ESP_ERR_INVALID_ARG, TAG, "pack_mv_out is null");
    ESP_RETURN_ON_FALSE(s_adc_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "adc handle not ready");

    int raw_acc = 0;
    int pin_mv_acc = 0;

    for (int i = 0; i < CONFIG_SPIPPY_POWER_SAMPLE_COUNT; ++i) {
        int raw = 0;
        ESP_RETURN_ON_ERROR(adc_oneshot_read(s_adc_handle, SPIPPY_BAT_ADC_CHANNEL, &raw),
                            TAG, "adc read failed");
        raw_acc += raw;
        pin_mv_acc += power_manager_raw_to_pin_mv(raw);
    }

    *raw_out = raw_acc / CONFIG_SPIPPY_POWER_SAMPLE_COUNT;
    *pin_mv_out = pin_mv_acc / CONFIG_SPIPPY_POWER_SAMPLE_COUNT;
    *pack_mv_out = (*pin_mv_out * CONFIG_SPIPPY_POWER_ADC_DIVIDER_NUMERATOR) /
                   CONFIG_SPIPPY_POWER_ADC_DIVIDER_DENOMINATOR;
    return ESP_OK;
}

esp_err_t power_manager_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = SPIPPY_BAT_ADC_UNIT,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&init_config, &s_adc_handle), TAG, "failed to create adc unit");

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = SPIPPY_BAT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc_handle, SPIPPY_BAT_ADC_CHANNEL, &chan_config),
                        TAG, "failed to configure adc channel");

    s_adc_calibration_ready = power_manager_adc_calibration_init(&s_adc_cali_handle);

    ESP_LOGI(TAG,
             "initialized battery adc on GPIO%d divider=%d/%d threshold=%dmV",
             SPIPPY_PIN_BAT_ADC,
             CONFIG_SPIPPY_POWER_ADC_DIVIDER_NUMERATOR,
             CONFIG_SPIPPY_POWER_ADC_DIVIDER_DENOMINATOR,
             CONFIG_SPIPPY_POWER_LOW_VOLTAGE_MV);
    return ESP_OK;
}

esp_err_t power_manager_start(void)
{
    s_power_status.system_state = SPIPPY_SYSTEM_STATE_STANDBY;
    ESP_LOGI(TAG, "power monitor armed");
    return ESP_OK;
}

esp_err_t power_manager_get_status(spippy_runtime_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *status = s_power_status;
    return ESP_OK;
}

esp_err_t power_manager_get_telemetry(spippy_power_telemetry_t *telemetry)
{
    if (telemetry == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    telemetry->raw_adc = s_sample_state.last_raw;
    telemetry->pin_voltage_mv = s_sample_state.last_pin_mv;
    telemetry->battery_voltage_mv = s_sample_state.last_pack_mv;
    telemetry->filtered_battery_voltage_mv = s_sample_state.filtered_pack_mv;
    telemetry->low_streak = s_sample_state.low_streak;
    telemetry->recover_streak = s_sample_state.recover_streak;
    telemetry->low_power_latched = s_power_status.low_power_latched;
    telemetry->low_power_warning = !s_power_status.low_power_latched &&
        (s_sample_state.filtered_pack_mv > 0) &&
        (s_sample_state.filtered_pack_mv <= CONFIG_SPIPPY_POWER_LOW_VOLTAGE_MV);
    telemetry->recovery_ready = s_sample_state.recover_streak >= CONFIG_SPIPPY_POWER_RECOVER_CONFIRM_COUNT;
    snprintf(telemetry->state,
             sizeof(telemetry->state),
             "%s",
             s_power_status.low_power_latched ? (telemetry->recovery_ready ? "RECOVER" : "LATCHED") :
                (telemetry->low_power_warning ? "LOW" : "OK"));
    return ESP_OK;
}

esp_err_t power_manager_clear_low_power_latch(bool force)
{
    bool recovery_ready = s_sample_state.recover_streak >= CONFIG_SPIPPY_POWER_RECOVER_CONFIRM_COUNT;
    if (!s_power_status.low_power_latched) {
        return ESP_OK;
    }
    if (!force && !recovery_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    s_power_status.low_power_latched = false;
    s_power_status.system_state = SPIPPY_SYSTEM_STATE_STANDBY;
    s_sample_state.low_streak = 0;
    s_sample_state.recover_streak = 0;
    ESP_LOGW(TAG, "low power latch cleared force=%d filtered=%dmV", force, s_sample_state.filtered_pack_mv);
    return ESP_OK;
}

esp_err_t power_manager_tick(void)
{
    int raw = 0;
    int pin_mv = 0;
    int pack_mv = 0;

    ESP_RETURN_ON_ERROR(power_manager_sample_voltage(&raw, &pin_mv, &pack_mv), TAG, "battery sample failed");

    s_sample_state.last_raw = raw;
    s_sample_state.last_pin_mv = pin_mv;
    s_sample_state.last_pack_mv = pack_mv;
    if (s_sample_state.filtered_pack_mv == 0) {
        s_sample_state.filtered_pack_mv = pack_mv;
    } else {
        s_sample_state.filtered_pack_mv = ((s_sample_state.filtered_pack_mv * 7) + pack_mv) / 8;
    }
    s_sample_state.tick_count++;

    if (!s_power_status.low_power_latched) {
        s_sample_state.recover_streak = 0;
        if (s_sample_state.filtered_pack_mv <= CONFIG_SPIPPY_POWER_LOW_VOLTAGE_MV) {
            s_sample_state.low_streak++;
            if (s_sample_state.low_streak >= CONFIG_SPIPPY_POWER_LOW_CONFIRM_COUNT) {
                s_power_status.low_power_latched = true;
                s_power_status.system_state = SPIPPY_SYSTEM_STATE_LOW_POWER_LOCK;
                ESP_LOGW(TAG, "low power latched at filtered=%dmV raw_pack=%dmV (raw=%d pin=%dmV)",
                         s_sample_state.filtered_pack_mv, pack_mv, raw, pin_mv);
            }
        } else {
            s_sample_state.low_streak = 0;
            s_power_status.system_state = SPIPPY_SYSTEM_STATE_STANDBY;
        }
    } else if (s_sample_state.filtered_pack_mv >= CONFIG_SPIPPY_POWER_RECOVER_VOLTAGE_MV) {
        s_sample_state.recover_streak++;
    } else {
        s_sample_state.recover_streak = 0;
    }

    if ((s_sample_state.tick_count % CONFIG_SPIPPY_POWER_LOG_INTERVAL) == 0U) {
        ESP_LOGI(TAG,
                 "battery raw=%d pin=%dmV pack=%dmV latched=%d streak=%" PRIu32,
                 raw,
                 pin_mv,
                 s_sample_state.filtered_pack_mv,
                 s_power_status.low_power_latched,
                 s_sample_state.low_streak);
    }

    return ESP_OK;
}
