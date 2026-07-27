#pragma once

#include "esp_err.h"
#include <stddef.h>
#include "spippy_types.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t servo_manager_init(void);
esp_err_t servo_manager_start(void);
esp_err_t servo_manager_apply_frame(const spippy_leg_frame_t *frame);
esp_err_t servo_manager_get_calibration(spippy_servo_calibration_t *calibration, size_t count);
esp_err_t servo_manager_set_calibration(const spippy_servo_calibration_t *calibration, size_t count, bool persist);
esp_err_t servo_manager_reload_calibration_from_storage(void);
esp_err_t servo_manager_reset_calibration_to_defaults(bool clear_persisted);
esp_err_t servo_manager_get_runtime_status(spippy_servo_runtime_status_t *status);
esp_err_t servo_manager_get_channel_info(spippy_servo_channel_info_t *info, size_t count);
esp_err_t servo_manager_fill_pose_frame(spippy_pose_mode_t pose, spippy_leg_frame_t *frame);
esp_err_t servo_manager_get_last_output_telemetry(spippy_servo_command_telemetry_t *telemetry, size_t count);
esp_err_t servo_manager_set_calibration_direct_output(const float *installed_angle_offsets_deg, size_t count);
esp_err_t servo_manager_clear_calibration_direct_output(void);
esp_err_t servo_manager_tick(void);

#ifdef __cplusplus
}
#endif
