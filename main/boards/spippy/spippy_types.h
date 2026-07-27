#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPIPPY_LEG_COUNT 4
#define SPIPPY_JOINTS_PER_LEG 2
#define SPIPPY_ACTIVE_SERVO_COUNT (SPIPPY_LEG_COUNT * SPIPPY_JOINTS_PER_LEG)
#define SPIPPY_SERVO_CHANNEL_BUDGET 12

typedef enum {
    SPIPPY_SYSTEM_STATE_BOOTING = 0,
    SPIPPY_SYSTEM_STATE_STANDBY,
    SPIPPY_SYSTEM_STATE_ACTIVE,
    SPIPPY_SYSTEM_STATE_LOW_POWER_LOCK,
    SPIPPY_SYSTEM_STATE_FAULT,
} spippy_system_state_t;

typedef enum {
    SPIPPY_ACTION_IDLE = 0,
    SPIPPY_ACTION_FORWARD,
    SPIPPY_ACTION_BACKWARD,
    SPIPPY_ACTION_TURN_LEFT,
    SPIPPY_ACTION_TURN_RIGHT,
    SPIPPY_ACTION_STOP,
    SPIPPY_ACTION_WAVE,
    SPIPPY_ACTION_HEAD_SWAY,
    SPIPPY_ACTION_SHAKE,
    SPIPPY_ACTION_BOW,
    SPIPPY_ACTION_CONGRATS,
    SPIPPY_ACTION_SIT,
    SPIPPY_ACTION_PUSHUP,
    SPIPPY_ACTION_COQUETRY,
    SPIPPY_ACTION_FRONT_BACK_SWAY,
    SPIPPY_ACTION_DANCE_1,
    SPIPPY_ACTION_DANCE_2,
    SPIPPY_ACTION_BOUNCE,
    SPIPPY_ACTION_POSE_1,
    SPIPPY_ACTION_IDLE_BREATHE,
    SPIPPY_ACTION_NOD,
    SPIPPY_ACTION_CUTE_WIGGLE,
    SPIPPY_ACTION_STARTLE,
    SPIPPY_ACTION_CONFUSED,
    SPIPPY_ACTION_AFRAID,
    SPIPPY_ACTION_ANGRY,
    SPIPPY_ACTION_SLEEPY,
    SPIPPY_ACTION_VOICE_PERFORM,
    SPIPPY_ACTION_PEE_RIGHT_REAR,
    SPIPPY_ACTION_PRONE,
} spippy_action_t;

typedef enum {
    SPIPPY_GAIT_TROT = 0,
} spippy_gait_t;

typedef enum {
    SPIPPY_SPEED_LEVEL_LOW = 0,
    SPIPPY_SPEED_LEVEL_MID,
    SPIPPY_SPEED_LEVEL_HIGH,
} spippy_speed_level_t;

typedef enum {
    SPIPPY_LEG_FRONT_LEFT = 0,
    SPIPPY_LEG_FRONT_RIGHT,
    SPIPPY_LEG_REAR_LEFT,
    SPIPPY_LEG_REAR_RIGHT,
} spippy_leg_id_t;

typedef enum {
    SPIPPY_JOINT_THETA1 = 0,
    SPIPPY_JOINT_THETA2,
} spippy_joint_id_t;

typedef enum {
    SPIPPY_POSE_SAFE = 0,
    SPIPPY_POSE_STAND,
    SPIPPY_POSE_SIT,
    SPIPPY_POSE_CROUCH,
} spippy_pose_mode_t;

typedef enum {
    SPIPPY_CALIBRATION_SOURCE_DEFAULTS = 0,
    SPIPPY_CALIBRATION_SOURCE_NVS,
} spippy_calibration_source_t;

typedef struct {
    float theta1_deg;
    float theta2_deg;
} leg_command_t;

typedef struct {
    spippy_gait_t gait;
    spippy_speed_level_t speed_level;
} walk_gait_params_t;

typedef enum {
    SPIPPY_INTERACTION_SOURCE_SYSTEM = 0,
    SPIPPY_INTERACTION_SOURCE_WEB,
    SPIPPY_INTERACTION_SOURCE_VOICE,
    SPIPPY_INTERACTION_SOURCE_AUTONOMY,
    SPIPPY_INTERACTION_SOURCE_SCRIPT,
    SPIPPY_INTERACTION_SOURCE_PROXIMITY,
} spippy_interaction_source_t;

typedef enum {
    SPIPPY_ACTION_COMPLETION_POLICY_DEFAULT = 0,
    SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE,
    SPIPPY_ACTION_COMPLETION_POLICY_HOLD_FINAL_POSE,
} spippy_action_completion_policy_t;

typedef struct {
    spippy_action_t action;
    walk_gait_params_t gait_params;
    uint32_t repeat_count;
    spippy_action_completion_policy_t completion_policy;
    spippy_interaction_source_t source;
    uint16_t start_audio_clip;
    bool wait_for_start_audio;
} spippy_action_request_t;

typedef struct {
    spippy_action_request_t active_request;
    bool active_request_valid;
    uint32_t repeat_remaining;
    uint32_t pending_count;
} spippy_action_queue_status_t;

typedef struct {
    leg_command_t legs[SPIPPY_LEG_COUNT];
    uint32_t sequence_id;
} spippy_leg_frame_t;

typedef struct {
    spippy_leg_id_t leg_id;
    spippy_joint_id_t joint_id;
    bool invert;
    int16_t zero_offset_deg_x10;
    int16_t min_angle_deg_x10;
    int16_t max_angle_deg_x10;
    int16_t safe_angle_deg_x10;
    int16_t stand_angle_deg_x10;
    uint8_t reserved[2];
} spippy_servo_calibration_t;

typedef struct {
    spippy_calibration_source_t calibration_source;
    bool persisted_calibration_present;
} spippy_servo_runtime_status_t;

typedef struct {
    const char *label;
    int gpio_num;
    spippy_leg_id_t leg_id;
    spippy_joint_id_t joint_id;
    bool active;
} spippy_servo_channel_info_t;

typedef struct {
    float logical_angle_deg;
    float calibrated_servo_angle_deg;
    uint32_t pulse_us;
} spippy_servo_command_telemetry_t;

typedef struct {
    int raw_adc;
    int pin_voltage_mv;
    int battery_voltage_mv;
    int filtered_battery_voltage_mv;
    uint32_t low_streak;
    uint32_t recover_streak;
    bool low_power_latched;
    bool low_power_warning;
    bool recovery_ready;
    char state[12];
} spippy_power_telemetry_t;

typedef struct {
    bool measurement_valid;
    uint32_t distance_mm;
    uint32_t sample_count;
    uint32_t timeout_count;
    uint32_t error_count;
    char last_error[32];
} spippy_ultrasonic_reading_t;

typedef enum {
    SPIPPY_PROXIMITY_ZONE_LOST = 0,
    SPIPPY_PROXIMITY_ZONE_FAR,
    SPIPPY_PROXIMITY_ZONE_NEAR,
    SPIPPY_PROXIMITY_ZONE_CLOSE,
    SPIPPY_PROXIMITY_ZONE_TOO_CLOSE,
} spippy_proximity_zone_t;

typedef enum {
    SPIPPY_PROXIMITY_EVENT_NONE = 0,
    SPIPPY_PROXIMITY_EVENT_ENTER_NEAR,
    SPIPPY_PROXIMITY_EVENT_ENTER_CLOSE,
    SPIPPY_PROXIMITY_EVENT_ENTER_TOO_CLOSE,
    SPIPPY_PROXIMITY_EVENT_SUDDEN_APPROACH,
    SPIPPY_PROXIMITY_EVENT_HOLD_CLOSE,
    SPIPPY_PROXIMITY_EVENT_LEAVE,
} spippy_proximity_event_t;

typedef struct {
    bool enabled;
    bool valid;
    uint32_t raw_distance_mm;
    uint32_t filtered_distance_mm;
    spippy_proximity_zone_t zone;
    spippy_proximity_zone_t previous_zone;
    spippy_proximity_event_t last_event;
    uint32_t stable_sample_count;
    uint32_t invalid_streak;
    uint32_t event_count;
    char last_event_text[32];
} spippy_proximity_status_t;

typedef struct {
    bool ready;
    bool enabled;
    bool busy;
    char last_pattern[24];
    char last_error[64];
} spippy_buzzer_status_t;

typedef struct {
    bool enabled;
    bool armed;
    char last_event[32];
    uint32_t next_due_ms;
    char last_error[96];
} spippy_autonomy_status_t;

typedef struct {
    bool emergency_stop;
    bool low_power_latched;
    bool manual_override_active;
    bool calibration_preview_active;
    bool fault_latched;
    bool output_safe_pose;
    uint32_t rejected_count;
    char last_reject_reason[96];
} spippy_safety_status_t;

typedef struct {
    spippy_system_state_t system_state;
    spippy_action_t active_action;
    walk_gait_params_t active_gait;
    spippy_pose_mode_t active_pose;
    bool emergency_stop;
    bool low_power_latched;
    bool manual_override_active;
    spippy_calibration_source_t servo_calibration_source;
    spippy_interaction_source_t active_action_source;
} spippy_runtime_status_t;

typedef enum {
    SPIPPY_AUDIO_CLIP_NONE = 0,
    SPIPPY_AUDIO_CLIP_BOOT,
    SPIPPY_AUDIO_CLIP_ACTION_ACK,
    SPIPPY_AUDIO_CLIP_WAVE,
    SPIPPY_AUDIO_CLIP_SHAKE,
    SPIPPY_AUDIO_CLIP_BOW,
    SPIPPY_AUDIO_CLIP_SIT,
    SPIPPY_AUDIO_CLIP_STAND_UP,
    SPIPPY_AUDIO_CLIP_ROCK,
    SPIPPY_AUDIO_CLIP_DANCE,
    SPIPPY_AUDIO_CLIP_LOW_POWER,
    SPIPPY_AUDIO_CLIP_EMERGENCY_STOP,
    SPIPPY_AUDIO_CLIP_SAFE_MODE,
    SPIPPY_AUDIO_CLIP_WEB_CONFIRM,
    SPIPPY_AUDIO_CLIP_DANCE_DONE,
    SPIPPY_AUDIO_CLIP_CONGRATS,
    SPIPPY_AUDIO_CLIP_MOVE_FORWARD,
    SPIPPY_AUDIO_CLIP_MOVE_BACKWARD,
    SPIPPY_AUDIO_CLIP_MOVE_TURN_LEFT,
    SPIPPY_AUDIO_CLIP_MOVE_TURN_RIGHT,
    SPIPPY_AUDIO_CLIP_STOP,
    SPIPPY_AUDIO_CLIP_PUSHUP,
    SPIPPY_AUDIO_CLIP_DANCE_2,
    SPIPPY_AUDIO_CLIP_POSE_1,
    SPIPPY_AUDIO_CLIP_IDLE_BREATHE,
    SPIPPY_AUDIO_CLIP_NOD,
    SPIPPY_AUDIO_CLIP_CUTE_WIGGLE,
    SPIPPY_AUDIO_CLIP_STARTLE,
    SPIPPY_AUDIO_CLIP_SLEEPY,
    SPIPPY_AUDIO_CLIP_VOICE_HELLO,
    SPIPPY_AUDIO_CLIP_VOICE_DANCE,
    SPIPPY_AUDIO_CLIP_VOICE_ARE_YOU_THERE,
    SPIPPY_AUDIO_CLIP_VOICE_CALL_ME_MASTER,
    SPIPPY_AUDIO_CLIP_VOICE_WHO_ARE_YOU,
    SPIPPY_AUDIO_CLIP_VOICE_HAPPY,
    SPIPPY_AUDIO_CLIP_VOICE_ANGRY,
    SPIPPY_AUDIO_CLIP_VOICE_SCARED,
    SPIPPY_AUDIO_CLIP_VOICE_TIRED,
    SPIPPY_AUDIO_CLIP_VOICE_PRAISE_ROBOT,
    SPIPPY_AUDIO_CLIP_VOICE_SCOLD_ROBOT,
    SPIPPY_AUDIO_CLIP_VOICE_CUTE,
    SPIPPY_AUDIO_CLIP_VOICE_COQUETRY,
    SPIPPY_AUDIO_CLIP_VOICE_BLINK,
    SPIPPY_AUDIO_CLIP_VOICE_SMILE,
    SPIPPY_AUDIO_CLIP_VOICE_PITIFUL,
    SPIPPY_AUDIO_CLIP_VOICE_CUTE_MODE,
    SPIPPY_AUDIO_CLIP_VOICE_PRAISE_USER,
    SPIPPY_AUDIO_CLIP_VOICE_SING,
    SPIPPY_AUDIO_CLIP_VOICE_GREETING,
    SPIPPY_AUDIO_CLIP_VOICE_PERFORM,
    SPIPPY_AUDIO_CLIP_VOICE_DOG,
    SPIPPY_AUDIO_CLIP_VOICE_CAT,
    SPIPPY_AUDIO_CLIP_VOICE_BATTERY,
    SPIPPY_AUDIO_CLIP_VOICE_STATUS_OK,
    SPIPPY_AUDIO_CLIP_VOICE_SELF_CHECK,
    SPIPPY_AUDIO_CLIP_VOICE_STANDBY,
    SPIPPY_AUDIO_CLIP_VOICE_WAKE_UP,
    SPIPPY_AUDIO_CLIP_VOICE_REST,
    SPIPPY_AUDIO_CLIP_VOICE_WORK,
    SPIPPY_AUDIO_CLIP_VOICE_PLAY_WITH_ME,
    SPIPPY_AUDIO_CLIP_VOICE_JOKE,
    SPIPPY_AUDIO_CLIP_VOICE_COMFORT,
    SPIPPY_AUDIO_CLIP_VOICE_LIKE_ME,
    SPIPPY_AUDIO_CLIP_VOICE_BATTLE_MODE,
    SPIPPY_AUDIO_CLIP_VOICE_MOFISH_MODE,
    SPIPPY_AUDIO_CLIP_VOICE_THANKS,
    SPIPPY_AUDIO_CLIP_VOICE_SORRY,
    SPIPPY_AUDIO_CLIP_VOICE_BIRTHDAY,
    SPIPPY_AUDIO_CLIP_VOICE_BYE,
    SPIPPY_AUDIO_CLIP_ANGRY_ENTER,
    SPIPPY_AUDIO_CLIP_ANGRY_EXIT,
    SPIPPY_AUDIO_CLIP_ANGRY_ACTION_ACK,
    SPIPPY_AUDIO_CLIP_ANGRY_MOVE_FORWARD,
    SPIPPY_AUDIO_CLIP_ANGRY_MOVE_BACKWARD,
    SPIPPY_AUDIO_CLIP_ANGRY_MOVE_TURN_LEFT,
    SPIPPY_AUDIO_CLIP_ANGRY_MOVE_TURN_RIGHT,
    SPIPPY_AUDIO_CLIP_ANGRY_WAVE,
    SPIPPY_AUDIO_CLIP_ANGRY_SHAKE,
    SPIPPY_AUDIO_CLIP_ANGRY_BOW,
    SPIPPY_AUDIO_CLIP_ANGRY_SIT,
    SPIPPY_AUDIO_CLIP_ANGRY_STAND_UP,
    SPIPPY_AUDIO_CLIP_ANGRY_PRONE,
    SPIPPY_AUDIO_CLIP_ANGRY_ROCK,
    SPIPPY_AUDIO_CLIP_ANGRY_DANCE_1,
    SPIPPY_AUDIO_CLIP_ANGRY_CONGRATS,
    SPIPPY_AUDIO_CLIP_ANGRY_STOP,
    SPIPPY_AUDIO_CLIP_ANGRY_PUSHUP,
    SPIPPY_AUDIO_CLIP_ANGRY_DANCE_2,
    SPIPPY_AUDIO_CLIP_ANGRY_POSE_1,
    SPIPPY_AUDIO_CLIP_ANGRY_IDLE_BREATHE,
    SPIPPY_AUDIO_CLIP_ANGRY_NOD,
    SPIPPY_AUDIO_CLIP_ANGRY_CUTE_WIGGLE,
    SPIPPY_AUDIO_CLIP_ANGRY_STARTLE,
    SPIPPY_AUDIO_CLIP_ANGRY_SLEEPY,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_HELLO,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_DANCE,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_ARE_YOU_THERE,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_CALL_ME_MASTER,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_WHO_ARE_YOU,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_HAPPY,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_ANGRY,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_SCARED,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_TIRED,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_PRAISE_ROBOT,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_SCOLD_ROBOT,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_CUTE,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_COQUETRY,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_BLINK,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_SMILE,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_PITIFUL,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_CUTE_MODE,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_PRAISE_USER,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_SING,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_GREETING,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_PERFORM,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_DOG,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_CAT,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_BATTERY,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_STATUS_OK,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_SELF_CHECK,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_STANDBY,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_WAKE_UP,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_REST,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_WORK,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_PLAY_WITH_ME,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_JOKE,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_COMFORT,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_LIKE_ME,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_BATTLE_MODE,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_MOFISH_MODE,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_THANKS,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_SORRY,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_BIRTHDAY,
    SPIPPY_AUDIO_CLIP_ANGRY_VOICE_BYE,
    SPIPPY_AUDIO_CLIP_PRONE,
    SPIPPY_AUDIO_CLIP_BOUNCE,
    SPIPPY_AUDIO_CLIP_HEAD_SWAY,
    SPIPPY_AUDIO_CLIP_CONFUSED,
    SPIPPY_AUDIO_CLIP_AFRAID,
    SPIPPY_AUDIO_CLIP_PEE_RIGHT_REAR,
    SPIPPY_AUDIO_CLIP_ANGRY_BOUNCE,
    SPIPPY_AUDIO_CLIP_ANGRY_HEAD_SWAY,
    SPIPPY_AUDIO_CLIP_ANGRY_CONFUSED,
    SPIPPY_AUDIO_CLIP_ANGRY_AFRAID,
    SPIPPY_AUDIO_CLIP_ANGRY_PEE_RIGHT_REAR,
} spippy_audio_clip_id_t;

typedef enum {
    SPIPPY_AUDIO_INTERRUPT_NONE = 0,
    SPIPPY_AUDIO_INTERRUPT_REPLACE_CURRENT,
    SPIPPY_AUDIO_INTERRUPT_FLUSH_ALL,
    SPIPPY_AUDIO_INTERRUPT_DROP_IF_BUSY,
} spippy_audio_interrupt_mode_t;

typedef struct {
    spippy_audio_clip_id_t clip_id;
    spippy_audio_interrupt_mode_t interrupt_mode;
    uint8_t repeat_count;
    uint8_t volume_pct;
} spippy_audio_play_request_t;

typedef struct {
    bool speaker_ready;
    bool microphone_ready;
    bool busy;
    spippy_audio_clip_id_t active_clip;
    uint32_t pending_requests;
    bool assets_mounted;
    bool asset_file_available;
    bool asset_streaming_active;
    bool fallback_tone_active;
    bool mic_streaming_active;
    uint64_t mic_samples_read_total;
    int32_t mic_last_error;
    char active_asset_id[32];
    char active_asset_path[128];
} spippy_audio_runtime_status_t;

typedef enum {
    SPIPPY_VOICE_INTENT_NONE = 0,
    SPIPPY_VOICE_INTENT_HELLO,
    SPIPPY_VOICE_INTENT_DANCE,
    SPIPPY_VOICE_INTENT_STOP,
    SPIPPY_VOICE_INTENT_FORWARD,
    SPIPPY_VOICE_INTENT_BACKWARD,
    SPIPPY_VOICE_INTENT_TURN_LEFT,
    SPIPPY_VOICE_INTENT_TURN_RIGHT,
    SPIPPY_VOICE_INTENT_SIT,
    SPIPPY_VOICE_INTENT_STAND_UP,
    SPIPPY_VOICE_INTENT_BOW,
    SPIPPY_VOICE_INTENT_CONGRATS,
    SPIPPY_VOICE_INTENT_LOW_POWER_STATUS,
    SPIPPY_VOICE_INTENT_WAVE,
    SPIPPY_VOICE_INTENT_SHAKE,
    SPIPPY_VOICE_INTENT_PUSHUP,
    SPIPPY_VOICE_INTENT_ROCK,
    SPIPPY_VOICE_INTENT_DANCE_2,
    SPIPPY_VOICE_INTENT_POSE_1,
    SPIPPY_VOICE_INTENT_IDLE_BREATHE,
    SPIPPY_VOICE_INTENT_NOD,
    SPIPPY_VOICE_INTENT_CUTE_WIGGLE,
    SPIPPY_VOICE_INTENT_STARTLE,
    SPIPPY_VOICE_INTENT_SLEEPY,
    SPIPPY_VOICE_INTENT_ARE_YOU_THERE,
    SPIPPY_VOICE_INTENT_CALL_ME_MASTER,
    SPIPPY_VOICE_INTENT_WHO_ARE_YOU,
    SPIPPY_VOICE_INTENT_HAPPY,
    SPIPPY_VOICE_INTENT_ANGRY,
    SPIPPY_VOICE_INTENT_SCARED,
    SPIPPY_VOICE_INTENT_TIRED,
    SPIPPY_VOICE_INTENT_PRAISE_ROBOT,
    SPIPPY_VOICE_INTENT_SCOLD_ROBOT,
    SPIPPY_VOICE_INTENT_CUTE,
    SPIPPY_VOICE_INTENT_COQUETRY,
    SPIPPY_VOICE_INTENT_BLINK,
    SPIPPY_VOICE_INTENT_SMILE,
    SPIPPY_VOICE_INTENT_PITIFUL,
    SPIPPY_VOICE_INTENT_CUTE_MODE,
    SPIPPY_VOICE_INTENT_PRAISE_USER,
    SPIPPY_VOICE_INTENT_SING,
    SPIPPY_VOICE_INTENT_GREETING,
    SPIPPY_VOICE_INTENT_PERFORM,
    SPIPPY_VOICE_INTENT_DOG,
    SPIPPY_VOICE_INTENT_CAT,
    SPIPPY_VOICE_INTENT_STATUS_OK,
    SPIPPY_VOICE_INTENT_SELF_CHECK,
    SPIPPY_VOICE_INTENT_STANDBY,
    SPIPPY_VOICE_INTENT_WAKE_UP,
    SPIPPY_VOICE_INTENT_REST,
    SPIPPY_VOICE_INTENT_WORK,
    SPIPPY_VOICE_INTENT_PLAY_WITH_ME,
    SPIPPY_VOICE_INTENT_JOKE,
    SPIPPY_VOICE_INTENT_COMFORT,
    SPIPPY_VOICE_INTENT_LIKE_ME,
    SPIPPY_VOICE_INTENT_BATTLE_MODE,
    SPIPPY_VOICE_INTENT_MOFISH_MODE,
    SPIPPY_VOICE_INTENT_THANKS,
    SPIPPY_VOICE_INTENT_SORRY,
    SPIPPY_VOICE_INTENT_BIRTHDAY,
    SPIPPY_VOICE_INTENT_BYE,
    SPIPPY_VOICE_INTENT_ANGRY_ENTER,
    SPIPPY_VOICE_INTENT_ANGRY_EXIT,
    SPIPPY_VOICE_INTENT_PRONE,
    SPIPPY_VOICE_INTENT_HEAD_SWAY,
    SPIPPY_VOICE_INTENT_BOUNCE,
    SPIPPY_VOICE_INTENT_CONFUSED,
    SPIPPY_VOICE_INTENT_AFRAID,
    SPIPPY_VOICE_INTENT_PEE_RIGHT_REAR,
} spippy_voice_intent_t;

#ifdef __cplusplus
}
#endif
