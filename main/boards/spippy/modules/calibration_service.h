#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"
#include "spippy_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    spippy_pose_mode_t active_pose;
    bool manual_override_active;
    bool preview_mode_enabled;
    bool preflight_running;
    spippy_servo_runtime_status_t servo_status;
    spippy_power_telemetry_t power;
} spippy_calibration_status_t;

typedef struct {
    size_t servo_index;
    spippy_servo_channel_info_t info;
    spippy_servo_calibration_t calibration;
    spippy_servo_command_telemetry_t telemetry;
} spippy_calibration_channel_snapshot_t;

esp_err_t calibration_service_init(void);
esp_err_t calibration_service_start(void);
esp_err_t calibration_service_tick(void);
esp_err_t calibration_service_get_status(spippy_calibration_status_t *status);
esp_err_t calibration_service_get_channel_snapshots(spippy_calibration_channel_snapshot_t *channels, size_t count);
esp_err_t calibration_service_set_pose(spippy_pose_mode_t pose);
esp_err_t calibration_service_set_preview_mode(bool enabled);
esp_err_t calibration_service_abort_preview(void);
esp_err_t calibration_service_get_preview_mode(bool *enabled);
esp_err_t calibration_service_set_preflight_mode(bool enabled);
esp_err_t calibration_service_get_preflight_mode(bool *enabled);
esp_err_t calibration_service_preview_servo(size_t servo_index, float angle_deg);
esp_err_t calibration_service_clear_preview(void);
esp_err_t calibration_service_update_servo_calibration(size_t servo_index,
                                                       const spippy_servo_calibration_t *entry);
esp_err_t calibration_service_save_calibration(void);
esp_err_t calibration_service_reload_calibration(void);
esp_err_t calibration_service_restore_defaults(void);
esp_err_t calibration_service_factory_reset(void);

#ifdef __cplusplus
}
#endif
