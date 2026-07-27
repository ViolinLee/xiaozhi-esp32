#include "modules/calibration_service.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "modules/motion_engine.h"
#include "modules/power_manager.h"
#include "modules/servo_manager.h"

static const char *TAG = "CalibrationSvc";

#define SPIPPY_CALIBRATION_MAX_ABS_DEG_X10 1800
#define SPIPPY_CALIBRATION_BASELINE_PWM_DEG 90.0f
#define SPIPPY_PREFLIGHT_EXCURSION_DEG 30.0f
#define SPIPPY_PREFLIGHT_CYCLE_MS 6000
#define SPIPPY_PREFLIGHT_QUARTER_CYCLE_MS (SPIPPY_PREFLIGHT_CYCLE_MS / 4)

static SemaphoreHandle_t s_lock;
static bool s_preview_mode_enabled;
static bool s_preflight_running;
static int64_t s_preflight_started_us;
static bool s_session_baseline_valid;
static spippy_servo_calibration_t s_session_baseline[SPIPPY_ACTIVE_SERVO_COUNT];
static float s_preview_installed_angles_deg[SPIPPY_ACTIVE_SERVO_COUNT];
static float s_preflight_center_angles_deg[SPIPPY_ACTIVE_SERVO_COUNT];

static esp_err_t calibration_service_lock(void)
{
    ESP_RETURN_ON_FALSE(s_lock != NULL, ESP_ERR_INVALID_STATE, TAG, "service lock not ready");
    return (xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE) ? ESP_OK : ESP_FAIL;
}

static void calibration_service_unlock(void)
{
    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }
}

static bool calibration_service_angle_x10_valid(int16_t value_deg_x10)
{
    return (value_deg_x10 >= -SPIPPY_CALIBRATION_MAX_ABS_DEG_X10) &&
           (value_deg_x10 <= SPIPPY_CALIBRATION_MAX_ABS_DEG_X10);
}

static esp_err_t calibration_service_load_calibration_table(spippy_servo_calibration_t *table,
                                                            spippy_servo_channel_info_t *info)
{
    ESP_RETURN_ON_ERROR(servo_manager_get_calibration(table, SPIPPY_ACTIVE_SERVO_COUNT),
                        TAG, "failed to read calibration table");
    ESP_RETURN_ON_ERROR(servo_manager_get_channel_info(info, SPIPPY_ACTIVE_SERVO_COUNT),
                        TAG, "failed to read channel info");
    return ESP_OK;
}

static esp_err_t calibration_service_apply_center_offsets_locked(const spippy_servo_calibration_t *calibration)
{
    ESP_RETURN_ON_FALSE(calibration != NULL, ESP_ERR_INVALID_ARG, TAG, "calibration table is null");

    for (size_t i = 0; i < SPIPPY_ACTIVE_SERVO_COUNT; ++i) {
        s_preview_installed_angles_deg[i] = SPIPPY_CALIBRATION_BASELINE_PWM_DEG +
                                            (calibration[i].zero_offset_deg_x10 / 10.0f);
    }
    ESP_RETURN_ON_ERROR(servo_manager_set_calibration_direct_output(s_preview_installed_angles_deg,
                                                                    SPIPPY_ACTIVE_SERVO_COUNT),
                        TAG, "failed to apply calibration center outputs");
    return ESP_OK;
}

static esp_err_t calibration_service_load_preflight_centers_locked(void)
{
    spippy_servo_calibration_t calibration[SPIPPY_ACTIVE_SERVO_COUNT];
    ESP_RETURN_ON_ERROR(servo_manager_get_calibration(calibration, SPIPPY_ACTIVE_SERVO_COUNT),
                        TAG, "failed to read preflight center offsets");
    for (size_t i = 0; i < SPIPPY_ACTIVE_SERVO_COUNT; ++i) {
        s_preflight_center_angles_deg[i] = SPIPPY_CALIBRATION_BASELINE_PWM_DEG +
                                           (calibration[i].zero_offset_deg_x10 / 10.0f);
    }
    return ESP_OK;
}

static esp_err_t calibration_service_apply_preflight_center_locked(void)
{
    ESP_RETURN_ON_ERROR(calibration_service_load_preflight_centers_locked(),
                        TAG, "failed to load preflight centers");
    ESP_RETURN_ON_ERROR(servo_manager_set_calibration_direct_output(s_preflight_center_angles_deg,
                                                                    SPIPPY_ACTIVE_SERVO_COUNT),
                        TAG, "failed to apply preflight centers");
    return ESP_OK;
}

static float calibration_service_preflight_excursion_deg(int64_t elapsed_ms)
{
    const int64_t phase_ms = elapsed_ms % SPIPPY_PREFLIGHT_CYCLE_MS;
    const float quarter = (float)SPIPPY_PREFLIGHT_QUARTER_CYCLE_MS;

    if (phase_ms < SPIPPY_PREFLIGHT_QUARTER_CYCLE_MS) {
        return SPIPPY_PREFLIGHT_EXCURSION_DEG * ((float)phase_ms / quarter);
    }
    if (phase_ms < (SPIPPY_PREFLIGHT_QUARTER_CYCLE_MS * 3)) {
        return SPIPPY_PREFLIGHT_EXCURSION_DEG -
               (SPIPPY_PREFLIGHT_EXCURSION_DEG * ((float)(phase_ms - SPIPPY_PREFLIGHT_QUARTER_CYCLE_MS) / quarter));
    }
    return -SPIPPY_PREFLIGHT_EXCURSION_DEG +
           (SPIPPY_PREFLIGHT_EXCURSION_DEG *
            ((float)(phase_ms - (SPIPPY_PREFLIGHT_QUARTER_CYCLE_MS * 3)) / quarter));
}

static bool calibration_service_low_power_latched(void)
{
    spippy_power_telemetry_t power;
    return (power_manager_get_telemetry(&power) == ESP_OK) && power.low_power_latched;
}

static esp_err_t calibration_service_guard_low_power_freeze_locked(const char *operation)
{
    if (!s_preview_mode_enabled || !calibration_service_low_power_latched()) {
        return ESP_OK;
    }

    ESP_LOGW(TAG, "%s blocked while low-power latch holds calibration outputs", operation);
    return ESP_ERR_INVALID_STATE;
}

static esp_err_t calibration_service_apply_pose_locked(spippy_pose_mode_t pose)
{
    if (s_preview_mode_enabled && s_session_baseline_valid) {
        ESP_RETURN_ON_ERROR(servo_manager_set_calibration(s_session_baseline,
                                                          SPIPPY_ACTIVE_SERVO_COUNT,
                                                          false),
                            TAG, "failed to restore calibration session baseline");
    }
    ESP_RETURN_ON_ERROR(motion_engine_clear_manual_override(), TAG, "failed to clear manual override");
    ESP_RETURN_ON_ERROR(servo_manager_clear_calibration_direct_output(), TAG, "failed to clear calibration direct output");
    ESP_RETURN_ON_ERROR(motion_engine_set_pose_mode(pose), TAG, "failed to set pose");
    s_preview_mode_enabled = false;
    s_session_baseline_valid = false;
    ESP_LOGI(TAG, "pose set to %d preview=%d", pose, s_preview_mode_enabled);
    return ESP_OK;
}

esp_err_t calibration_service_init(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(s_lock != NULL, ESP_ERR_NO_MEM, TAG, "failed to create service lock");
    }

    s_preview_mode_enabled = false;
    s_preflight_running = false;
    s_preflight_started_us = 0;
    s_session_baseline_valid = false;
    ESP_LOGI(TAG, "initialized");
    return ESP_OK;
}

esp_err_t calibration_service_start(void)
{
    ESP_LOGI(TAG, "ready for web calibration requests");
    return ESP_OK;
}

esp_err_t calibration_service_tick(void)
{
    esp_err_t ret = ESP_OK;
    float outputs[SPIPPY_ACTIVE_SERVO_COUNT];

    ESP_RETURN_ON_ERROR(calibration_service_lock(), TAG, "lock failed");
    if (!s_preflight_running) {
        calibration_service_unlock();
        return ESP_OK;
    }
    ESP_GOTO_ON_FALSE(!calibration_service_low_power_latched(),
                      ESP_ERR_INVALID_STATE,
                      exit,
                      TAG,
                      "preflight sweep blocked while low-power latch is active");

    const int64_t now_us = esp_timer_get_time();
    const int64_t elapsed_ms = (now_us - s_preflight_started_us) / 1000;
    const float excursion_deg = calibration_service_preflight_excursion_deg(elapsed_ms);
    for (size_t i = 0; i < SPIPPY_ACTIVE_SERVO_COUNT; ++i) {
        outputs[i] = s_preflight_center_angles_deg[i] + excursion_deg;
    }
    ESP_GOTO_ON_ERROR(servo_manager_set_calibration_direct_output(outputs,
                                                                  SPIPPY_ACTIVE_SERVO_COUNT),
                      exit,
                      TAG,
                      "failed to update preflight sweep");

exit:
    calibration_service_unlock();
    return ret;
}

esp_err_t calibration_service_get_status(spippy_calibration_status_t *status)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(status != NULL, ESP_ERR_INVALID_ARG, TAG, "status is null");
    ESP_RETURN_ON_ERROR(calibration_service_lock(), TAG, "lock failed");

    memset(status, 0, sizeof(*status));
    status->preview_mode_enabled = s_preview_mode_enabled;
    status->preflight_running = s_preflight_running;
    ESP_GOTO_ON_ERROR(motion_engine_get_pose_mode(&status->active_pose), exit, TAG, "failed to get pose");
    ESP_GOTO_ON_ERROR(motion_engine_get_manual_override_active(&status->manual_override_active),
                      exit, TAG, "failed to get manual override state");
    ESP_GOTO_ON_ERROR(servo_manager_get_runtime_status(&status->servo_status), exit, TAG, "failed to get servo status");
    ESP_GOTO_ON_ERROR(power_manager_get_telemetry(&status->power), exit, TAG, "failed to get power telemetry");

exit:
    calibration_service_unlock();
    return ret;
}

esp_err_t calibration_service_get_channel_snapshots(spippy_calibration_channel_snapshot_t *channels, size_t count)
{
    esp_err_t ret = ESP_OK;
    spippy_servo_channel_info_t info[SPIPPY_ACTIVE_SERVO_COUNT];
    spippy_servo_calibration_t calibration[SPIPPY_ACTIVE_SERVO_COUNT];
    spippy_servo_command_telemetry_t telemetry[SPIPPY_ACTIVE_SERVO_COUNT];

    ESP_RETURN_ON_FALSE((channels != NULL) && (count >= SPIPPY_ACTIVE_SERVO_COUNT),
                        ESP_ERR_INVALID_ARG, TAG, "channels buffer too small");
    ESP_RETURN_ON_ERROR(calibration_service_lock(), TAG, "lock failed");

    ESP_GOTO_ON_ERROR(calibration_service_load_calibration_table(calibration, info), exit, TAG, "failed to read table");
    ESP_GOTO_ON_ERROR(servo_manager_get_last_output_telemetry(telemetry, SPIPPY_ACTIVE_SERVO_COUNT),
                      exit, TAG, "failed to read telemetry");

    for (size_t i = 0; i < SPIPPY_ACTIVE_SERVO_COUNT; ++i) {
        channels[i].servo_index = i;
        channels[i].info = info[i];
        channels[i].calibration = calibration[i];
        channels[i].telemetry = telemetry[i];
    }

exit:
    calibration_service_unlock();
    return ret;
}

esp_err_t calibration_service_set_pose(spippy_pose_mode_t pose)
{
    ESP_RETURN_ON_ERROR(calibration_service_lock(), TAG, "lock failed");
    esp_err_t err = s_preflight_running ? ESP_ERR_INVALID_STATE :
                                          calibration_service_apply_pose_locked(pose);
    calibration_service_unlock();
    return err;
}

esp_err_t calibration_service_set_preview_mode(bool enabled)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_ERROR(calibration_service_lock(), TAG, "lock failed");
    if (enabled) {
        spippy_servo_calibration_t calibration[SPIPPY_ACTIVE_SERVO_COUNT];

        if (s_preview_mode_enabled) {
            calibration_service_unlock();
            return ESP_OK;
        }
        ESP_GOTO_ON_FALSE(!s_preflight_running,
                          ESP_ERR_INVALID_STATE,
                          exit,
                          TAG,
                          "calibration preview blocked while preflight is running");

        ESP_GOTO_ON_FALSE(!calibration_service_low_power_latched(),
                          ESP_ERR_INVALID_STATE,
                          exit,
                          TAG,
                          "preview mode blocked while low-power latch is active");
        ESP_GOTO_ON_ERROR(motion_engine_clear_manual_override(), exit, TAG, "failed to clear manual override");
        ESP_GOTO_ON_ERROR(motion_engine_set_pose_mode(SPIPPY_POSE_SAFE), exit, TAG, "failed to enter safe pose");
        ESP_GOTO_ON_ERROR(servo_manager_get_calibration(calibration, SPIPPY_ACTIVE_SERVO_COUNT),
                          exit, TAG, "failed to read calibration table");
        memcpy(s_session_baseline, calibration, sizeof(s_session_baseline));
        s_session_baseline_valid = true;
        ESP_GOTO_ON_ERROR(calibration_service_apply_center_offsets_locked(calibration),
                          exit, TAG, "failed to apply 90 degree calibration baseline");
        s_preview_mode_enabled = true;
        ESP_LOGI(TAG, "preview mode enabled via 90 degree center baseline");
    } else {
        ESP_GOTO_ON_FALSE(!s_preflight_running,
                          ESP_ERR_INVALID_STATE,
                          exit,
                          TAG,
                          "calibration preview cannot be cancelled while preflight is running");
        ESP_GOTO_ON_ERROR(calibration_service_apply_pose_locked(SPIPPY_POSE_STAND),
                          exit, TAG, "failed to cancel calibration session");
        ESP_LOGI(TAG, "preview mode disabled and unsaved changes discarded");
    }

exit:
    calibration_service_unlock();
    return ret;
}

esp_err_t calibration_service_abort_preview(void)
{
    esp_err_t first_error = ESP_OK;
    ESP_RETURN_ON_ERROR(calibration_service_lock(), TAG, "lock failed");

    if (s_preview_mode_enabled && s_session_baseline_valid) {
        esp_err_t err = servo_manager_set_calibration(s_session_baseline,
                                                      SPIPPY_ACTIVE_SERVO_COUNT,
                                                      false);
        if (first_error == ESP_OK && err != ESP_OK) {
            first_error = err;
        }
    }
    esp_err_t err = servo_manager_clear_calibration_direct_output();
    if (first_error == ESP_OK && err != ESP_OK) {
        first_error = err;
    }
    err = motion_engine_clear_manual_override();
    if (first_error == ESP_OK && err != ESP_OK) {
        first_error = err;
    }
    err = motion_engine_set_pose_mode(SPIPPY_POSE_SAFE);
    if (first_error == ESP_OK && err != ESP_OK) {
        first_error = err;
    }
    s_preview_mode_enabled = false;
    s_preflight_running = false;
    s_preflight_started_us = 0;
    s_session_baseline_valid = false;
    calibration_service_unlock();
    return first_error;
}

esp_err_t calibration_service_get_preview_mode(bool *enabled)
{
    ESP_RETURN_ON_FALSE(enabled != NULL, ESP_ERR_INVALID_ARG, TAG, "enabled is null");
    ESP_RETURN_ON_ERROR(calibration_service_lock(), TAG, "lock failed");
    *enabled = s_preview_mode_enabled;
    calibration_service_unlock();
    return ESP_OK;
}

esp_err_t calibration_service_set_preflight_mode(bool enabled)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_ERROR(calibration_service_lock(), TAG, "lock failed");
    if (enabled) {
        if (s_preflight_running) {
            calibration_service_unlock();
            return ESP_OK;
        }
        ESP_GOTO_ON_FALSE(!s_preview_mode_enabled,
                          ESP_ERR_INVALID_STATE,
                          exit,
                          TAG,
                          "preflight blocked while calibration preview is active");
        ESP_GOTO_ON_FALSE(!calibration_service_low_power_latched(),
                          ESP_ERR_INVALID_STATE,
                          exit,
                          TAG,
                          "preflight blocked while low-power latch is active");
        ESP_GOTO_ON_ERROR(motion_engine_clear_manual_override(),
                          exit,
                          TAG,
                          "failed to clear manual override before preflight");
        ESP_GOTO_ON_ERROR(motion_engine_set_pose_mode(SPIPPY_POSE_SAFE),
                          exit,
                          TAG,
                          "failed to enter safe pose before preflight");
        ESP_GOTO_ON_ERROR(calibration_service_apply_preflight_center_locked(),
                          exit,
                          TAG,
                          "failed to establish preflight center");
        s_preflight_started_us = esp_timer_get_time();
        s_preflight_running = true;
        ESP_LOGI(TAG, "preflight sweep started around compensated 90 degree centers");
    } else {
        if (!s_preflight_running) {
            calibration_service_unlock();
            return ESP_OK;
        }
        s_preflight_running = false;
        s_preflight_started_us = 0;
        ESP_GOTO_ON_ERROR(servo_manager_clear_calibration_direct_output(),
                          exit,
                          TAG,
                          "failed to release preflight direct outputs");
        ESP_GOTO_ON_ERROR(motion_engine_clear_manual_override(),
                          exit,
                          TAG,
                          "failed to clear manual override after preflight");
        ESP_GOTO_ON_ERROR(motion_engine_set_pose_mode(SPIPPY_POSE_STAND),
                          exit,
                          TAG,
                          "failed to restore standby pose after preflight");
        ESP_LOGI(TAG, "preflight stopped and normal standby pose restored");
    }

exit:
    calibration_service_unlock();
    return ret;
}

esp_err_t calibration_service_get_preflight_mode(bool *enabled)
{
    ESP_RETURN_ON_FALSE(enabled != NULL, ESP_ERR_INVALID_ARG, TAG, "enabled is null");
    ESP_RETURN_ON_ERROR(calibration_service_lock(), TAG, "lock failed");
    *enabled = s_preflight_running;
    calibration_service_unlock();
    return ESP_OK;
}

esp_err_t calibration_service_preview_servo(size_t servo_index, float angle_deg)
{
    esp_err_t ret = ESP_OK;
    spippy_servo_channel_info_t info[SPIPPY_ACTIVE_SERVO_COUNT];
    spippy_servo_calibration_t calibration[SPIPPY_ACTIVE_SERVO_COUNT];
    spippy_pose_mode_t pose = SPIPPY_POSE_SAFE;

    ESP_RETURN_ON_FALSE(servo_index < SPIPPY_ACTIVE_SERVO_COUNT, ESP_ERR_INVALID_ARG, TAG, "invalid servo index");
    ESP_RETURN_ON_FALSE(isfinite(angle_deg), ESP_ERR_INVALID_ARG, TAG, "angle is invalid");

    ESP_RETURN_ON_ERROR(calibration_service_lock(), TAG, "lock failed");
    ESP_GOTO_ON_FALSE(s_preview_mode_enabled, ESP_ERR_INVALID_STATE, exit, TAG, "preview mode not enabled");
    ESP_GOTO_ON_ERROR(calibration_service_guard_low_power_freeze_locked("servo preview"),
                      exit,
                      TAG,
                      "low-power calibration freeze active");
    ESP_GOTO_ON_ERROR(motion_engine_get_pose_mode(&pose), exit, TAG, "failed to read current pose");
    ESP_GOTO_ON_FALSE(pose == SPIPPY_POSE_SAFE, ESP_ERR_INVALID_STATE, exit, TAG, "preview requires SAFE pose");
    ESP_GOTO_ON_ERROR(calibration_service_load_calibration_table(calibration, info), exit, TAG, "failed to read table");
    for (size_t i = 0; i < SPIPPY_ACTIVE_SERVO_COUNT; ++i) {
        s_preview_installed_angles_deg[i] = SPIPPY_CALIBRATION_BASELINE_PWM_DEG +
                                            (calibration[i].zero_offset_deg_x10 / 10.0f);
    }
    s_preview_installed_angles_deg[servo_index] = SPIPPY_CALIBRATION_BASELINE_PWM_DEG + angle_deg;
    ESP_GOTO_ON_ERROR(servo_manager_set_calibration_direct_output(s_preview_installed_angles_deg,
                                                                  SPIPPY_ACTIVE_SERVO_COUNT),
                      exit, TAG, "failed to set direct calibration preview");
    ESP_LOGI(TAG, "preview servo[%u] %s center_offset=%.1fdeg installed=%.1fdeg",
             (unsigned)servo_index,
             info[servo_index].label,
             angle_deg,
             s_preview_installed_angles_deg[servo_index]);

exit:
    calibration_service_unlock();
    return ret;
}

esp_err_t calibration_service_clear_preview(void)
{
    ESP_RETURN_ON_ERROR(calibration_service_lock(), TAG, "lock failed");
    esp_err_t err = ESP_OK;
    if (s_preview_mode_enabled) {
        spippy_servo_calibration_t calibration[SPIPPY_ACTIVE_SERVO_COUNT];
        err = calibration_service_guard_low_power_freeze_locked("preview clear");
        if (err == ESP_OK) {
            err = servo_manager_get_calibration(calibration, SPIPPY_ACTIVE_SERVO_COUNT);
        }
        if (err == ESP_OK) {
            err = calibration_service_apply_center_offsets_locked(calibration);
        }
    } else {
        err = motion_engine_clear_manual_override();
    }
    calibration_service_unlock();
    return err;
}

esp_err_t calibration_service_update_servo_calibration(size_t servo_index,
                                                       const spippy_servo_calibration_t *entry)
{
    esp_err_t ret = ESP_OK;
    spippy_servo_calibration_t calibration[SPIPPY_ACTIVE_SERVO_COUNT];
    spippy_servo_channel_info_t info[SPIPPY_ACTIVE_SERVO_COUNT];

    ESP_RETURN_ON_FALSE((servo_index < SPIPPY_ACTIVE_SERVO_COUNT) && (entry != NULL),
                        ESP_ERR_INVALID_ARG, TAG, "invalid update request");
    ESP_RETURN_ON_ERROR(calibration_service_lock(), TAG, "lock failed");
    ESP_GOTO_ON_FALSE(s_preview_mode_enabled, ESP_ERR_INVALID_STATE, exit,
                      TAG, "calibration session not active");
    ESP_GOTO_ON_ERROR(calibration_service_guard_low_power_freeze_locked("calibration update"),
                      exit,
                      TAG,
                      "low-power calibration freeze active");
    ESP_GOTO_ON_ERROR(calibration_service_load_calibration_table(calibration, info), exit, TAG, "failed to read table");

    ESP_GOTO_ON_FALSE((entry->leg_id == info[servo_index].leg_id) && (entry->joint_id == info[servo_index].joint_id),
                      ESP_ERR_INVALID_ARG, exit, TAG, "web calibration cannot remap servo ownership");
    ESP_GOTO_ON_FALSE(entry->min_angle_deg_x10 <= entry->max_angle_deg_x10,
                      ESP_ERR_INVALID_ARG, exit, TAG, "invalid logical min/max");
    ESP_GOTO_ON_FALSE(calibration_service_angle_x10_valid(entry->zero_offset_deg_x10) &&
                          calibration_service_angle_x10_valid(entry->min_angle_deg_x10) &&
                          calibration_service_angle_x10_valid(entry->max_angle_deg_x10) &&
                          calibration_service_angle_x10_valid(entry->safe_angle_deg_x10) &&
                          calibration_service_angle_x10_valid(entry->stand_angle_deg_x10),
                      ESP_ERR_INVALID_ARG, exit, TAG, "calibration angle out of allowed range");

    calibration[servo_index] = *entry;
    ESP_GOTO_ON_ERROR(servo_manager_set_calibration(calibration, SPIPPY_ACTIVE_SERVO_COUNT, false),
                      exit, TAG, "failed to apply updated calibration");
    if (s_preview_mode_enabled) {
        ESP_GOTO_ON_ERROR(calibration_service_apply_center_offsets_locked(calibration),
                          exit, TAG, "failed to refresh calibration center output");
    }
    ESP_LOGI(TAG, "updated calibration[%u] %s inv=%d zero=%.1f range=%.1f..%.1f safe=%.1f stand=%.1f",
             (unsigned)servo_index,
             info[servo_index].label,
             entry->invert,
             entry->zero_offset_deg_x10 / 10.0f,
             entry->min_angle_deg_x10 / 10.0f,
             entry->max_angle_deg_x10 / 10.0f,
             entry->safe_angle_deg_x10 / 10.0f,
             entry->stand_angle_deg_x10 / 10.0f);

exit:
    calibration_service_unlock();
    return ret;
}

esp_err_t calibration_service_save_calibration(void)
{
    esp_err_t ret = ESP_OK;
    spippy_servo_calibration_t calibration[SPIPPY_ACTIVE_SERVO_COUNT];

    ESP_RETURN_ON_ERROR(calibration_service_lock(), TAG, "lock failed");
    ESP_GOTO_ON_FALSE(s_preview_mode_enabled, ESP_ERR_INVALID_STATE, exit,
                      TAG, "calibration session not active");
    ESP_GOTO_ON_ERROR(calibration_service_guard_low_power_freeze_locked("calibration save"),
                      exit,
                      TAG,
                      "low-power calibration freeze active");
    ESP_GOTO_ON_ERROR(servo_manager_get_calibration(calibration, SPIPPY_ACTIVE_SERVO_COUNT),
                      exit, TAG, "failed to read calibration");
    ESP_GOTO_ON_ERROR(servo_manager_set_calibration(calibration, SPIPPY_ACTIVE_SERVO_COUNT, true),
                      exit, TAG, "failed to persist calibration");
    ESP_GOTO_ON_ERROR(motion_engine_clear_manual_override(), exit, TAG, "failed to clear manual override");
    ESP_GOTO_ON_ERROR(servo_manager_clear_calibration_direct_output(), exit, TAG, "failed to clear calibration direct output");
    ESP_GOTO_ON_ERROR(motion_engine_set_pose_mode(SPIPPY_POSE_STAND), exit, TAG, "failed to return to stand pose");
    s_preview_mode_enabled = false;
    s_session_baseline_valid = false;

exit:
    calibration_service_unlock();
    return ret;
}

esp_err_t calibration_service_reload_calibration(void)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_ERROR(calibration_service_lock(), TAG, "lock failed");
    ESP_GOTO_ON_ERROR(calibration_service_guard_low_power_freeze_locked("calibration reload"),
                      exit,
                      TAG,
                      "low-power calibration freeze active");
    ESP_GOTO_ON_ERROR(motion_engine_clear_manual_override(), exit, TAG, "failed to clear manual override");
    ESP_GOTO_ON_ERROR(servo_manager_clear_calibration_direct_output(), exit, TAG, "failed to clear calibration direct output");
    s_preview_mode_enabled = false;
    ESP_GOTO_ON_ERROR(servo_manager_reload_calibration_from_storage(), exit, TAG, "failed to reload calibration");

exit:
    calibration_service_unlock();
    return ret;
}

esp_err_t calibration_service_restore_defaults(void)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_ERROR(calibration_service_lock(), TAG, "lock failed");
    ESP_GOTO_ON_ERROR(calibration_service_guard_low_power_freeze_locked("restore defaults"),
                      exit,
                      TAG,
                      "low-power calibration freeze active");
    ESP_GOTO_ON_ERROR(motion_engine_clear_manual_override(), exit, TAG, "failed to clear manual override");
    ESP_GOTO_ON_ERROR(servo_manager_clear_calibration_direct_output(), exit, TAG, "failed to clear calibration direct output");
    s_preview_mode_enabled = false;
    ESP_GOTO_ON_ERROR(servo_manager_reset_calibration_to_defaults(false), exit, TAG, "failed to restore defaults");

exit:
    calibration_service_unlock();
    return ret;
}

esp_err_t calibration_service_factory_reset(void)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_ERROR(calibration_service_lock(), TAG, "lock failed");
    ESP_GOTO_ON_ERROR(calibration_service_guard_low_power_freeze_locked("factory reset"),
                      exit,
                      TAG,
                      "low-power calibration freeze active");
    ESP_GOTO_ON_ERROR(motion_engine_clear_manual_override(), exit, TAG, "failed to clear manual override");
    ESP_GOTO_ON_ERROR(servo_manager_clear_calibration_direct_output(), exit, TAG, "failed to clear calibration direct output");
    s_preview_mode_enabled = false;
    ESP_GOTO_ON_ERROR(servo_manager_reset_calibration_to_defaults(true), exit, TAG, "failed to clear persisted calibration");

exit:
    calibration_service_unlock();
    return ret;
}
