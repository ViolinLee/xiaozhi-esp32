#include "modules/motion_engine.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"

#include "generated/spippy_motion_groups.h"
#include "modules/servo_manager.h"
#include "sdkconfig.h"

static const char *TAG = "MotionEngine";

#define SPIPPY_MOTION_PI                3.14159265358979323846f
#define SPIPPY_TRANSITION_TIME_MS       240.0f
#define SPIPPY_DEMO_DEFAULT_CYCLE_MS    1400.0f
#define SPIPPY_SIM_THETA2_CENTER_DEG    110.0f
#define SPIPPY_SIM_THETA1_CENTER_DEG    50.0f
#define SPIPPY_SIM_THETA2_MIN_DEG       70.0f
#define SPIPPY_SIM_THETA2_MAX_DEG       130.0f
#define SPIPPY_SIM_THETA1_MIN_DEG       -70.0f
#define SPIPPY_SIM_THETA1_MAX_DEG       70.0f
#define SPIPPY_SIM_LINK1_MM             35.0f
#define SPIPPY_SIM_LINK_MM              30.0f
#define SPIPPY_SIM_SIDE_LINK_MM         27.0f
#define SPIPPY_SIM_SIDE_ANGLE_DEG       57.0f
#define SPIPPY_SIM_FOOT_EXTENSION_MM    25.0f
/*
 * The simulation's body-frame gait direction is the reference convention. On the
 * current hardware installation, applying that logical theta1 travel directly
 * produces the opposite real fore/aft foot motion, so compensate once at the
 * robot-intent -> leg-local travel boundary instead of swapping public actions.
 */
#define SPIPPY_REAL_GAIT_BODY_FORWARD_SIGN (-1.0f)

typedef struct {
    float forward_mm;
    float z_mm;
} spippy_foot_target_t;

typedef struct {
    float theta1_deg;
    float theta2_deg;
} spippy_sim_leg_pose_t;

typedef struct {
    float stand_forward_mm;
    float stand_z_mm;
    bool initialized;
} spippy_motion_workspace_t;

typedef struct {
    bool active;
    float elapsed_ms;
    spippy_leg_frame_t start_frame;
    spippy_leg_frame_t target_frame;
} spippy_transition_state_t;

static walk_gait_params_t s_active_gait = {
    .gait = SPIPPY_GAIT_TROT,
    .speed_level = SPIPPY_SPEED_LEVEL_LOW,
};
static spippy_action_t s_active_action = SPIPPY_ACTION_IDLE;
static spippy_pose_mode_t s_active_pose = SPIPPY_POSE_STAND;
static spippy_leg_frame_t s_latest_frame;
static spippy_leg_frame_t s_manual_override_frame;
static spippy_leg_frame_t s_stop_hold_frame;
static spippy_leg_frame_t s_terminal_hold_frame;
static bool s_stop_hold_valid;
static bool s_terminal_hold_active;
static bool s_manual_override_active;
static bool s_cycle_complete_pending;
static float s_action_elapsed_ms;
static spippy_transition_state_t s_transition;
static spippy_motion_workspace_t s_workspace;

static float motion_engine_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float motion_engine_wrap_phase(float phase)
{
    while (phase >= 1.0f) {
        phase -= 1.0f;
    }
    while (phase < 0.0f) {
        phase += 1.0f;
    }
    return phase;
}

static float motion_engine_lerp(float from, float to, float t)
{
    return from + ((to - from) * t);
}

static float motion_engine_ease_in_out(float t)
{
    float clamped = motion_engine_clampf(t, 0.0f, 1.0f);
    return 0.5f - (0.5f * cosf(clamped * SPIPPY_MOTION_PI));
}

static float motion_engine_deg_to_rad(float degrees)
{
    return degrees * (SPIPPY_MOTION_PI / 180.0f);
}

static float motion_engine_rad_to_deg(float radians)
{
    return radians * (180.0f / SPIPPY_MOTION_PI);
}

static float motion_engine_fixed_side_angle_rad(void)
{
    return motion_engine_deg_to_rad(180.0f - SPIPPY_SIM_SIDE_ANGLE_DEG);
}

static float motion_engine_leg_radial_mm(float theta2_deg)
{
    float beta = motion_engine_fixed_side_angle_rad() + motion_engine_deg_to_rad(theta2_deg);
    float side_angle = motion_engine_fixed_side_angle_rad();
    return SPIPPY_SIM_LINK1_MM +
           (SPIPPY_SIM_SIDE_LINK_MM * cosf(side_angle)) -
           (SPIPPY_SIM_LINK_MM * cosf(beta));
}

static float motion_engine_leg_z_mm(float theta2_deg)
{
    float beta = motion_engine_fixed_side_angle_rad() + motion_engine_deg_to_rad(theta2_deg);
    float side_angle = motion_engine_fixed_side_angle_rad();
    return (-SPIPPY_SIM_SIDE_LINK_MM * sinf(side_angle)) +
           (SPIPPY_SIM_LINK_MM * sinf(beta)) -
           SPIPPY_SIM_FOOT_EXTENSION_MM;
}

static void motion_engine_workspace_init(void)
{
    float stand_theta1 = motion_engine_deg_to_rad(SPIPPY_SIM_THETA1_CENTER_DEG);
    s_workspace.stand_forward_mm = motion_engine_leg_radial_mm(SPIPPY_SIM_THETA2_CENTER_DEG) * cosf(stand_theta1);
    s_workspace.stand_z_mm = motion_engine_leg_z_mm(SPIPPY_SIM_THETA2_CENTER_DEG);
    s_workspace.initialized = true;
}

static bool motion_engine_is_walk_action(spippy_action_t action)
{
    switch (action) {
        case SPIPPY_ACTION_FORWARD:
        case SPIPPY_ACTION_BACKWARD:
        case SPIPPY_ACTION_TURN_LEFT:
        case SPIPPY_ACTION_TURN_RIGHT:
            return true;
        default:
            return false;
    }
}

static const spippy_motion_foot_clip_t *motion_engine_foot_clip_for_speed(spippy_speed_level_t speed_level)
{
    switch (speed_level) {
        case SPIPPY_SPEED_LEVEL_HIGH:
            return &s_spippy_motion_foot_clip_high;
        case SPIPPY_SPEED_LEVEL_MID:
            return &s_spippy_motion_foot_clip_mid;
        case SPIPPY_SPEED_LEVEL_LOW:
        default:
            return &s_spippy_motion_foot_clip_low;
    }
}

static const spippy_motion_pose_clip_t *motion_engine_pose_clip_for_action(spippy_action_t action)
{
    /* CONGRATS keeps its own action/audio/face while reusing the bow motion. */
    if (action == SPIPPY_ACTION_CONGRATS) {
        action = SPIPPY_ACTION_BOW;
    }

    for (size_t i = 0; i < (sizeof(s_spippy_motion_pose_clips) / sizeof(s_spippy_motion_pose_clips[0])); ++i) {
        if ((s_spippy_motion_pose_clips[i] != NULL) && (s_spippy_motion_pose_clips[i]->action == action)) {
            return s_spippy_motion_pose_clips[i];
        }
    }
    return NULL;
}

static float motion_engine_lerp_q10(int16_t from_x10, int16_t to_x10, float t)
{
    return ((float)from_x10 + (((float)to_x10 - (float)from_x10) * t)) / 10.0f;
}

static void motion_engine_clip_indices(uint16_t sample_count,
                                       float phase,
                                       size_t *index0,
                                       size_t *index1,
                                       float *mix)
{
    float wrapped = motion_engine_wrap_phase(phase);
    float position = wrapped * (float)sample_count;
    size_t i0 = (size_t)position;
    if (i0 >= sample_count) {
        i0 = 0U;
    }
    size_t i1 = (i0 + 1U) % sample_count;

    if (index0 != NULL) {
        *index0 = i0;
    }
    if (index1 != NULL) {
        *index1 = i1;
    }
    if (mix != NULL) {
        *mix = position - (float)i0;
    }
}

static spippy_foot_target_t motion_engine_sample_foot_target(const spippy_motion_foot_clip_t *clip,
                                                            spippy_leg_id_t leg_id,
                                                            float phase,
                                                            float travel_sign)
{
    size_t i0 = 0U;
    size_t i1 = 0U;
    float mix = 0.0f;
    motion_engine_clip_indices(clip->sample_count, phase, &i0, &i1, &mix);

    const spippy_motion_foot_sample_t *a = &clip->samples[i0][leg_id];
    const spippy_motion_foot_sample_t *b = &clip->samples[i1][leg_id];
    float forward_delta = motion_engine_lerp_q10(a->forward_delta_mm_x10, b->forward_delta_mm_x10, mix);
    float z_delta = motion_engine_lerp_q10(a->z_delta_mm_x10, b->z_delta_mm_x10, mix);

    return (spippy_foot_target_t) {
        .forward_mm = s_workspace.stand_forward_mm + (travel_sign * forward_delta),
        .z_mm = s_workspace.stand_z_mm + z_delta,
    };
}

static spippy_sim_leg_pose_t motion_engine_sample_pose_delta(const spippy_motion_pose_clip_t *clip,
                                                            spippy_leg_id_t leg_id,
                                                            float phase)
{
    size_t i0 = 0U;
    size_t i1 = 0U;
    float mix = 0.0f;
    motion_engine_clip_indices(clip->sample_count, phase, &i0, &i1, &mix);

    const spippy_motion_pose_sample_t *a = &clip->samples[i0][leg_id];
    const spippy_motion_pose_sample_t *b = &clip->samples[i1][leg_id];
    return (spippy_sim_leg_pose_t) {
        .theta1_deg = SPIPPY_SIM_THETA1_CENTER_DEG +
            motion_engine_lerp_q10(a->theta1_delta_deg_x10, b->theta1_delta_deg_x10, mix),
        .theta2_deg = SPIPPY_SIM_THETA2_CENTER_DEG +
            motion_engine_lerp_q10(a->theta2_delta_deg_x10, b->theta2_delta_deg_x10, mix),
    };
}

static spippy_pose_mode_t motion_engine_pose_for_action(spippy_action_t action)
{
    switch (action) {
        case SPIPPY_ACTION_SIT:
            return SPIPPY_POSE_SIT;
        case SPIPPY_ACTION_BOW:
        case SPIPPY_ACTION_CONGRATS:
        case SPIPPY_ACTION_PRONE:
            return SPIPPY_POSE_CROUCH;
        case SPIPPY_ACTION_IDLE:
        case SPIPPY_ACTION_STOP:
        case SPIPPY_ACTION_FORWARD:
        case SPIPPY_ACTION_BACKWARD:
        case SPIPPY_ACTION_TURN_LEFT:
        case SPIPPY_ACTION_TURN_RIGHT:
        case SPIPPY_ACTION_WAVE:
        case SPIPPY_ACTION_SHAKE:
        case SPIPPY_ACTION_PUSHUP:
        case SPIPPY_ACTION_COQUETRY:
        case SPIPPY_ACTION_DANCE_1:
        case SPIPPY_ACTION_DANCE_2:
        case SPIPPY_ACTION_POSE_1:
        case SPIPPY_ACTION_ANGRY:
        case SPIPPY_ACTION_VOICE_PERFORM:
        default:
            return SPIPPY_POSE_STAND;
    }
}

static float motion_engine_cycle_ms_for_action(spippy_action_t action)
{
    if (motion_engine_is_walk_action(action)) {
        return (float)motion_engine_foot_clip_for_speed(s_active_gait.speed_level)->cycle_ms;
    }
    const spippy_motion_pose_clip_t *clip = motion_engine_pose_clip_for_action(action);
    return (clip != NULL) ? (float)clip->cycle_ms : SPIPPY_DEMO_DEFAULT_CYCLE_MS;
}

static void motion_engine_interpolate_frame(const spippy_leg_frame_t *from,
                                            const spippy_leg_frame_t *to,
                                            float t,
                                            spippy_leg_frame_t *out)
{
    float mix = motion_engine_ease_in_out(t);

    memset(out, 0, sizeof(*out));
    for (size_t i = 0; i < SPIPPY_LEG_COUNT; ++i) {
        out->legs[i].theta2_deg = motion_engine_lerp(from->legs[i].theta2_deg, to->legs[i].theta2_deg, mix);
        out->legs[i].theta1_deg = motion_engine_lerp(from->legs[i].theta1_deg, to->legs[i].theta1_deg, mix);
    }
}

static void motion_engine_apply_sim_pose(spippy_leg_frame_t *frame,
                                         const spippy_leg_frame_t *stand_frame,
                                         spippy_leg_id_t leg_id,
                                         spippy_sim_leg_pose_t pose)
{
    float theta2_delta = pose.theta2_deg - SPIPPY_SIM_THETA2_CENTER_DEG;
    float theta1_delta = pose.theta1_deg - SPIPPY_SIM_THETA1_CENTER_DEG;

    frame->legs[leg_id].theta2_deg = stand_frame->legs[leg_id].theta2_deg + theta2_delta;
    frame->legs[leg_id].theta1_deg = stand_frame->legs[leg_id].theta1_deg + theta1_delta;
}

static spippy_sim_leg_pose_t motion_engine_solve_sim_pose(const spippy_foot_target_t *target)
{
    float side_angle = motion_engine_fixed_side_angle_rad();
    float sin_beta = (target->z_mm +
                      (SPIPPY_SIM_SIDE_LINK_MM * sinf(side_angle)) +
                      SPIPPY_SIM_FOOT_EXTENSION_MM) /
                     SPIPPY_SIM_LINK_MM;
    float beta = SPIPPY_MOTION_PI - asinf(motion_engine_clampf(sin_beta, -1.0f, 1.0f));
    float theta2_deg = motion_engine_rad_to_deg(beta - side_angle);
    theta2_deg = motion_engine_clampf(theta2_deg, SPIPPY_SIM_THETA2_MIN_DEG, SPIPPY_SIM_THETA2_MAX_DEG);

    float radial_mm = motion_engine_leg_radial_mm(theta2_deg);
    if (radial_mm < 0.001f) {
        radial_mm = 0.001f;
    }

    float cos_theta1 = target->forward_mm / radial_mm;
    float theta1_abs_deg = motion_engine_rad_to_deg(acosf(motion_engine_clampf(cos_theta1, -1.0f, 1.0f)));
    float theta1_deg = motion_engine_clampf(theta1_abs_deg, SPIPPY_SIM_THETA1_MIN_DEG, SPIPPY_SIM_THETA1_MAX_DEG);

    return (spippy_sim_leg_pose_t) {
        .theta1_deg = theta1_deg,
        .theta2_deg = theta2_deg,
    };
}

static esp_err_t motion_engine_generate_walk_frame(spippy_action_t action,
                                                   float phase,
                                                   spippy_leg_frame_t *frame)
{
    const spippy_motion_foot_clip_t *clip = motion_engine_foot_clip_for_speed(s_active_gait.speed_level);
    spippy_leg_frame_t stand_frame;

    /* 基准姿态生成失败时不能继续使用未初始化的栈数据计算逆运动学。 */
    ESP_RETURN_ON_ERROR(servo_manager_fill_pose_frame(SPIPPY_POSE_STAND, &stand_frame),
                        TAG, "failed to seed walk frame");
    *frame = stand_frame;

    for (size_t i = 0; i < SPIPPY_LEG_COUNT; ++i) {
        bool rear_leg = (i == SPIPPY_LEG_REAR_LEFT) || (i == SPIPPY_LEG_REAR_RIGHT);
        bool left_leg = (i == SPIPPY_LEG_FRONT_LEFT) || (i == SPIPPY_LEG_REAR_LEFT);
        float body_forward_sign = 1.0f;

        if (action == SPIPPY_ACTION_BACKWARD) {
            body_forward_sign = -1.0f;
        } else if (action == SPIPPY_ACTION_TURN_LEFT) {
            body_forward_sign = left_leg ? -1.0f : 1.0f;
        } else if (action == SPIPPY_ACTION_TURN_RIGHT) {
            body_forward_sign = left_leg ? 1.0f : -1.0f;
        }

        float local_forward_sign = rear_leg ? -1.0f : 1.0f;
        float travel_sign = SPIPPY_REAL_GAIT_BODY_FORWARD_SIGN * body_forward_sign * local_forward_sign;
        spippy_foot_target_t target = motion_engine_sample_foot_target(clip, (spippy_leg_id_t)i, phase, travel_sign);
        spippy_sim_leg_pose_t pose = motion_engine_solve_sim_pose(&target);
        motion_engine_apply_sim_pose(frame, &stand_frame, (spippy_leg_id_t)i, pose);
    }
    return ESP_OK;
}

static esp_err_t motion_engine_generate_pose_clip_frame(const spippy_motion_pose_clip_t *clip,
                                                        float phase,
                                                        spippy_leg_frame_t *frame)
{
    spippy_leg_frame_t stand_frame;

    ESP_RETURN_ON_ERROR(servo_manager_fill_pose_frame(SPIPPY_POSE_STAND, &stand_frame),
                        TAG, "failed to seed pose clip frame");
    *frame = stand_frame;

    for (size_t i = 0; i < SPIPPY_LEG_COUNT; ++i) {
        spippy_sim_leg_pose_t pose = motion_engine_sample_pose_delta(clip, (spippy_leg_id_t)i, phase);
        motion_engine_apply_sim_pose(frame, &stand_frame, (spippy_leg_id_t)i, pose);
    }
    return ESP_OK;
}

static esp_err_t motion_engine_generate_action_frame(spippy_action_t action,
                                                     float phase,
                                                     spippy_leg_frame_t *frame)
{
    ESP_RETURN_ON_FALSE(frame != NULL, ESP_ERR_INVALID_ARG, TAG, "frame is null");

    if (action == SPIPPY_ACTION_STOP) {
        if (!s_stop_hold_valid) {
            ESP_RETURN_ON_ERROR(servo_manager_fill_pose_frame(SPIPPY_POSE_STAND, &s_stop_hold_frame),
                                TAG, "failed to seed stop hold frame");
            s_stop_hold_valid = true;
        }
        *frame = s_stop_hold_frame;
        return ESP_OK;
    }

    if (motion_engine_is_walk_action(action)) {
        return motion_engine_generate_walk_frame(action, phase, frame);
    }

    const spippy_motion_pose_clip_t *clip = motion_engine_pose_clip_for_action(action);
    if (clip != NULL) {
        return motion_engine_generate_pose_clip_frame(clip, phase, frame);
    }

    ESP_RETURN_ON_ERROR(servo_manager_fill_pose_frame(motion_engine_pose_for_action(action), frame),
                        TAG, "failed to generate fallback pose frame");
    return ESP_OK;
}

static esp_err_t motion_engine_start_transition_if_needed(void)
{
    spippy_leg_frame_t start_frame = s_latest_frame;
    spippy_leg_frame_t target_frame;

    ESP_RETURN_ON_ERROR(motion_engine_generate_action_frame(s_active_action, 0.0f, &target_frame),
                        TAG, "failed to generate transition target");
    target_frame.sequence_id = s_latest_frame.sequence_id;

    s_transition.active = true;
    s_transition.elapsed_ms = 0.0f;
    s_transition.start_frame = start_frame;
    s_transition.target_frame = target_frame;
    s_action_elapsed_ms = 0.0f;
    s_cycle_complete_pending = false;
    s_terminal_hold_active = false;

    return ESP_OK;
}

esp_err_t motion_engine_init(void)
{
    memset(&s_latest_frame, 0, sizeof(s_latest_frame));
    memset(&s_manual_override_frame, 0, sizeof(s_manual_override_frame));
    memset(&s_stop_hold_frame, 0, sizeof(s_stop_hold_frame));
    memset(&s_terminal_hold_frame, 0, sizeof(s_terminal_hold_frame));
    memset(&s_transition, 0, sizeof(s_transition));
    ESP_RETURN_ON_ERROR(servo_manager_fill_pose_frame(SPIPPY_POSE_SAFE, &s_latest_frame),
                        TAG, "failed to seed safe pose");
    s_active_pose = SPIPPY_POSE_STAND;
    s_manual_override_active = false;
    s_cycle_complete_pending = false;
    s_stop_hold_valid = false;
    s_terminal_hold_active = false;
    s_action_elapsed_ms = 0.0f;
    motion_engine_workspace_init();
    ESP_LOGI(TAG,
             "initialized action-group motion engine with analytical IK stand=(%.2fmm, %.2fmm)",
             s_workspace.stand_forward_mm,
             s_workspace.stand_z_mm);
    return ESP_OK;
}

esp_err_t motion_engine_start(void)
{
    ESP_LOGI(TAG, "motion engine ready");
    return ESP_OK;
}

esp_err_t motion_engine_set_walk_params(const walk_gait_params_t *params)
{
    if (params == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const float old_cycle_ms = motion_engine_cycle_ms_for_action(s_active_action);
    const float old_elapsed_ms = s_action_elapsed_ms;
    s_active_gait.gait = SPIPPY_GAIT_TROT;
    s_active_gait.speed_level = (params->speed_level > SPIPPY_SPEED_LEVEL_HIGH) ?
        SPIPPY_SPEED_LEVEL_LOW : params->speed_level;

    if (!s_manual_override_active && motion_engine_is_walk_action(s_active_action)) {
        /* 调速时保持当前步态相位，不能重新播放起步过渡，否则会产生一次明显停顿。 */
        const float new_cycle_ms = motion_engine_cycle_ms_for_action(s_active_action);
        const float phase = (old_cycle_ms > 0.0f) ?
            fmodf(old_elapsed_ms, old_cycle_ms) / old_cycle_ms : 0.0f;
        s_action_elapsed_ms = phase * new_cycle_ms;
    }
    return ESP_OK;
}

esp_err_t motion_engine_set_action(spippy_action_t action)
{
    ESP_RETURN_ON_FALSE((action >= SPIPPY_ACTION_IDLE) && (action <= SPIPPY_ACTION_PRONE),
                        ESP_ERR_INVALID_ARG, TAG, "invalid action=%d", action);
    s_active_action = action;
    s_active_pose = motion_engine_pose_for_action(action);
    s_stop_hold_valid = false;
    s_terminal_hold_active = false;
    if (action == SPIPPY_ACTION_STOP) {
        s_stop_hold_frame = s_latest_frame;
        s_stop_hold_valid = true;
    }

    if (!s_manual_override_active) {
        ESP_RETURN_ON_ERROR(motion_engine_start_transition_if_needed(),
                            TAG, "failed to start action transition");
        ESP_LOGI(TAG, "action=%d pose=%d speed=%d", action, s_active_pose, s_active_gait.speed_level);
    }
    return ESP_OK;
}

esp_err_t motion_engine_set_pose_mode(spippy_pose_mode_t pose)
{
    ESP_RETURN_ON_FALSE((pose >= SPIPPY_POSE_SAFE) && (pose <= SPIPPY_POSE_CROUCH),
                        ESP_ERR_INVALID_ARG, TAG, "invalid pose=%d", pose);
    s_active_pose = pose;
    s_active_action = (pose == SPIPPY_POSE_SIT) ? SPIPPY_ACTION_SIT : SPIPPY_ACTION_IDLE;
    s_stop_hold_valid = false;
    s_terminal_hold_active = false;
    if (!s_manual_override_active) {
        ESP_RETURN_ON_ERROR(motion_engine_start_transition_if_needed(),
                            TAG, "failed to start pose transition");
    }
    return ESP_OK;
}

esp_err_t motion_engine_hold_current_frame(spippy_action_t action, spippy_pose_mode_t pose)
{
    s_active_action = action;
    s_active_pose = pose;
    s_terminal_hold_frame = s_latest_frame;
    s_terminal_hold_active = true;
    s_transition.active = false;
    s_cycle_complete_pending = false;
    s_action_elapsed_ms = 0.0f;
    ESP_LOGI(TAG, "holding terminal frame action=%d pose=%d", action, pose);
    return ESP_OK;
}

esp_err_t motion_engine_get_pose_mode(spippy_pose_mode_t *pose)
{
    if (pose == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *pose = s_active_pose;
    return ESP_OK;
}

esp_err_t motion_engine_set_manual_override(const spippy_leg_frame_t *frame)
{
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t i = 0; i < SPIPPY_LEG_COUNT; ++i) {
        if (!isfinite(frame->legs[i].theta1_deg) || !isfinite(frame->legs[i].theta2_deg)) {
            return ESP_ERR_INVALID_ARG;
        }
    }

    s_manual_override_frame = *frame;
    s_manual_override_active = true;
    s_transition.active = false;
    s_cycle_complete_pending = false;
    ESP_LOGI(TAG, "manual override enabled seq=%" PRIu32, frame->sequence_id);
    return ESP_OK;
}

esp_err_t motion_engine_clear_manual_override(void)
{
    s_manual_override_active = false;
    ESP_RETURN_ON_ERROR(motion_engine_start_transition_if_needed(),
                        TAG, "failed to resume planned action");
    ESP_LOGI(TAG, "manual override cleared, resuming pose=%d", s_active_pose);
    return ESP_OK;
}

esp_err_t motion_engine_get_manual_override_active(bool *active)
{
    if (active == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *active = s_manual_override_active;
    return ESP_OK;
}

esp_err_t motion_engine_tick(void)
{
    spippy_leg_frame_t generated_frame;
    const float tick_ms = (float)CONFIG_SPIPPY_TASK_MOTION_PERIOD_MS;

    if (s_manual_override_active) {
        s_latest_frame = s_manual_override_frame;
        s_latest_frame.sequence_id++;
        s_manual_override_frame.sequence_id = s_latest_frame.sequence_id;
        return ESP_OK;
    }

    if (s_transition.active) {
        s_transition.elapsed_ms += tick_ms;
        motion_engine_interpolate_frame(&s_transition.start_frame,
                                        &s_transition.target_frame,
                                        s_transition.elapsed_ms / SPIPPY_TRANSITION_TIME_MS,
                                        &generated_frame);
        if (s_transition.elapsed_ms >= SPIPPY_TRANSITION_TIME_MS) {
            generated_frame = s_transition.target_frame;
            s_transition.active = false;
        }
    } else if (s_terminal_hold_active) {
        generated_frame = s_terminal_hold_frame;
    } else {
        float cycle_ms = motion_engine_cycle_ms_for_action(s_active_action);
        float phase = 0.0f;

        if (cycle_ms > 0.0f) {
            phase = s_action_elapsed_ms / cycle_ms;
        }

        ESP_RETURN_ON_ERROR(motion_engine_generate_action_frame(s_active_action, phase, &generated_frame),
                            TAG, "failed to generate action frame");

        s_action_elapsed_ms += tick_ms;
        if ((cycle_ms > 0.0f) && (s_action_elapsed_ms >= cycle_ms)) {
            s_action_elapsed_ms = fmodf(s_action_elapsed_ms, cycle_ms);
            s_cycle_complete_pending = true;
        }
    }

    generated_frame.sequence_id = s_latest_frame.sequence_id + 1U;
    s_latest_frame = generated_frame;
    return ESP_OK;
}

esp_err_t motion_engine_consume_cycle_complete(bool *completed)
{
    if (completed == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *completed = s_cycle_complete_pending;
    s_cycle_complete_pending = false;
    return ESP_OK;
}

esp_err_t motion_engine_get_latest_frame(spippy_leg_frame_t *frame)
{
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *frame = s_latest_frame;
    return ESP_OK;
}
