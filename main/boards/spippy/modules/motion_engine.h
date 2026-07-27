#pragma once

#include "esp_err.h"
#include "spippy_types.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t motion_engine_init(void);
esp_err_t motion_engine_start(void);
esp_err_t motion_engine_set_walk_params(const walk_gait_params_t *params);
esp_err_t motion_engine_set_action(spippy_action_t action);
esp_err_t motion_engine_set_pose_mode(spippy_pose_mode_t pose);
esp_err_t motion_engine_hold_current_frame(spippy_action_t action, spippy_pose_mode_t pose);
esp_err_t motion_engine_get_pose_mode(spippy_pose_mode_t *pose);
esp_err_t motion_engine_set_manual_override(const spippy_leg_frame_t *frame);
esp_err_t motion_engine_clear_manual_override(void);
esp_err_t motion_engine_get_manual_override_active(bool *active);
esp_err_t motion_engine_tick(void);
esp_err_t motion_engine_consume_cycle_complete(bool *completed);
esp_err_t motion_engine_get_latest_frame(spippy_leg_frame_t *frame);

#ifdef __cplusplus
}
#endif
