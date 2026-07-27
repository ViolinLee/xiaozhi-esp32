#include "modules/servo_manager.h"

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"

#include "modules/storage.h"
#include "sdkconfig.h"
#include "spippy_pins.h"

static const char *TAG = "ServoManager";

#define SPIPPY_SERVO_LEDC_MODE           LEDC_LOW_SPEED_MODE
#define SPIPPY_SERVO_LEDC_TIMER          LEDC_TIMER_0
#define SPIPPY_SERVO_LEDC_DUTY_BITS      14
#define SPIPPY_SERVO_LEDC_DUTY_RES       LEDC_TIMER_14_BIT
#define SPIPPY_SERVO_PERIOD_US           (1000000 / CONFIG_SPIPPY_SERVO_PWM_FREQUENCY_HZ)
#define SPIPPY_SERVO_MAX_DUTY            ((1U << SPIPPY_SERVO_LEDC_DUTY_BITS) - 1U)
#define SPIPPY_SERVO_STORAGE_NS          "servo"
#define SPIPPY_SERVO_STORAGE_KEY         "cal"
#define SPIPPY_SERVO_THETA1_CENTER_X10   500
#define SPIPPY_SERVO_THETA2_CENTER_X10   1100
#define SPIPPY_SERVO_THETA1_INST_CENTER_DEG 90.0f
#define SPIPPY_SERVO_THETA1_INST_MIN_DEG 0.0f
#define SPIPPY_SERVO_THETA1_INST_MAX_DEG 180.0f
#define SPIPPY_SERVO_THETA2_INST_MIN_DEG 0.0f
#define SPIPPY_SERVO_THETA2_INST_MAX_DEG 180.0f
#define SPIPPY_SERVO_MAX_ZERO_OFFSET_X10 450

_Static_assert(CONFIG_SPIPPY_SERVO_SAFE_THETA1_DEG >= CONFIG_SPIPPY_SERVO_THETA1_MIN_DEG &&
               CONFIG_SPIPPY_SERVO_SAFE_THETA1_DEG <= CONFIG_SPIPPY_SERVO_THETA1_MAX_DEG,
               "theta1 safe angle must be inside logical limits");
_Static_assert(CONFIG_SPIPPY_SERVO_SAFE_THETA2_DEG >= CONFIG_SPIPPY_SERVO_THETA2_MIN_DEG &&
               CONFIG_SPIPPY_SERVO_SAFE_THETA2_DEG <= CONFIG_SPIPPY_SERVO_THETA2_MAX_DEG,
               "theta2 safe angle must be inside logical limits");

typedef struct {
    spippy_servo_calibration_t channels[SPIPPY_ACTIVE_SERVO_COUNT];
} spippy_servo_calibration_blob_t;

typedef struct {
    const char *label;
    int gpio_num;
    bool active;
    ledc_channel_t ledc_channel;
    float pulse_map_min_angle_deg;
    float pulse_map_max_angle_deg;
    spippy_servo_calibration_t default_calibration;
} spippy_servo_output_t;

static spippy_leg_frame_t s_target_frame;
static spippy_servo_calibration_t s_calibration[SPIPPY_ACTIVE_SERVO_COUNT];
static spippy_servo_command_telemetry_t s_last_output[SPIPPY_ACTIVE_SERVO_COUNT];
static float s_direct_installed_angle_offsets_deg[SPIPPY_ACTIVE_SERVO_COUNT];
static spippy_servo_runtime_status_t s_runtime_status = {
    .calibration_source = SPIPPY_CALIBRATION_SOURCE_DEFAULTS,
    .persisted_calibration_present = false,
};
static bool s_hw_ready;
static bool s_frame_dirty;
static bool s_direct_output_active;
static bool s_direct_output_dirty;

static esp_err_t servo_manager_save_calibration_to_storage(void);

/*
 * Firmware keeps the simulation convention as the logical command frame:
 * theta1 is the shared leg-local root swing and theta2 grows as the
 * parallelogram included angle opens. For theta2, mirrored installations use
 * the supplementary servo angle: installed = 180deg - logical.
 */
static const spippy_servo_output_t s_servo_outputs[SPIPPY_SERVO_CHANNEL_BUDGET] = {
    {
        .label = "front_left.theta2",
        .gpio_num = SPIPPY_PIN_SERVO_FL2_PWM,
        .active = true,
        .ledc_channel = LEDC_CHANNEL_0,
        .pulse_map_min_angle_deg = SPIPPY_SERVO_THETA2_INST_MIN_DEG,
        .pulse_map_max_angle_deg = SPIPPY_SERVO_THETA2_INST_MAX_DEG,
        .default_calibration = {
            .leg_id = SPIPPY_LEG_FRONT_LEFT,
            .joint_id = SPIPPY_JOINT_THETA2,
            .invert = true,
            .zero_offset_deg_x10 = 0,
            .min_angle_deg_x10 = CONFIG_SPIPPY_SERVO_THETA2_MIN_DEG * 10,
            .max_angle_deg_x10 = CONFIG_SPIPPY_SERVO_THETA2_MAX_DEG * 10,
            .safe_angle_deg_x10 = CONFIG_SPIPPY_SERVO_SAFE_THETA2_DEG * 10,
            .stand_angle_deg_x10 = SPIPPY_SERVO_THETA2_CENTER_X10,
        },
    },
    {
        .label = "front_left.theta1",
        .gpio_num = SPIPPY_PIN_SERVO_FL1_PWM,
        .active = true,
        .ledc_channel = LEDC_CHANNEL_1,
        .pulse_map_min_angle_deg = SPIPPY_SERVO_THETA1_INST_MIN_DEG,
        .pulse_map_max_angle_deg = SPIPPY_SERVO_THETA1_INST_MAX_DEG,
        .default_calibration = {
            .leg_id = SPIPPY_LEG_FRONT_LEFT,
            .joint_id = SPIPPY_JOINT_THETA1,
            .invert = true,
            .zero_offset_deg_x10 = 0,
            .min_angle_deg_x10 = CONFIG_SPIPPY_SERVO_THETA1_MIN_DEG * 10,
            .max_angle_deg_x10 = CONFIG_SPIPPY_SERVO_THETA1_MAX_DEG * 10,
            .safe_angle_deg_x10 = CONFIG_SPIPPY_SERVO_SAFE_THETA1_DEG * 10,
            .stand_angle_deg_x10 = SPIPPY_SERVO_THETA1_CENTER_X10,
        },
    },
    {
        .label = "front_right.theta2",
        .gpio_num = SPIPPY_PIN_SERVO_FR2_PWM,
        .active = true,
        .ledc_channel = LEDC_CHANNEL_2,
        .pulse_map_min_angle_deg = SPIPPY_SERVO_THETA2_INST_MIN_DEG,
        .pulse_map_max_angle_deg = SPIPPY_SERVO_THETA2_INST_MAX_DEG,
        .default_calibration = {
            .leg_id = SPIPPY_LEG_FRONT_RIGHT,
            .joint_id = SPIPPY_JOINT_THETA2,
            .invert = false,
            .zero_offset_deg_x10 = 0,
            .min_angle_deg_x10 = CONFIG_SPIPPY_SERVO_THETA2_MIN_DEG * 10,
            .max_angle_deg_x10 = CONFIG_SPIPPY_SERVO_THETA2_MAX_DEG * 10,
            .safe_angle_deg_x10 = CONFIG_SPIPPY_SERVO_SAFE_THETA2_DEG * 10,
            .stand_angle_deg_x10 = SPIPPY_SERVO_THETA2_CENTER_X10,
        },
    },
    {
        .label = "front_right.theta1",
        .gpio_num = SPIPPY_PIN_SERVO_FR1_PWM,
        .active = true,
        .ledc_channel = LEDC_CHANNEL_3,
        .pulse_map_min_angle_deg = SPIPPY_SERVO_THETA1_INST_MIN_DEG,
        .pulse_map_max_angle_deg = SPIPPY_SERVO_THETA1_INST_MAX_DEG,
        .default_calibration = {
            .leg_id = SPIPPY_LEG_FRONT_RIGHT,
            .joint_id = SPIPPY_JOINT_THETA1,
            .invert = false,
            .zero_offset_deg_x10 = 0,
            .min_angle_deg_x10 = CONFIG_SPIPPY_SERVO_THETA1_MIN_DEG * 10,
            .max_angle_deg_x10 = CONFIG_SPIPPY_SERVO_THETA1_MAX_DEG * 10,
            .safe_angle_deg_x10 = CONFIG_SPIPPY_SERVO_SAFE_THETA1_DEG * 10,
            .stand_angle_deg_x10 = SPIPPY_SERVO_THETA1_CENTER_X10,
        },
    },
    {
        .label = "rear_left.theta2",
        .gpio_num = SPIPPY_PIN_SERVO_BL2_PWM,
        .active = true,
        .ledc_channel = LEDC_CHANNEL_4,
        .pulse_map_min_angle_deg = SPIPPY_SERVO_THETA2_INST_MIN_DEG,
        .pulse_map_max_angle_deg = SPIPPY_SERVO_THETA2_INST_MAX_DEG,
        .default_calibration = {
            .leg_id = SPIPPY_LEG_REAR_LEFT,
            .joint_id = SPIPPY_JOINT_THETA2,
            .invert = false,
            .zero_offset_deg_x10 = 0,
            .min_angle_deg_x10 = CONFIG_SPIPPY_SERVO_THETA2_MIN_DEG * 10,
            .max_angle_deg_x10 = CONFIG_SPIPPY_SERVO_THETA2_MAX_DEG * 10,
            .safe_angle_deg_x10 = CONFIG_SPIPPY_SERVO_SAFE_THETA2_DEG * 10,
            .stand_angle_deg_x10 = SPIPPY_SERVO_THETA2_CENTER_X10,
        },
    },
    {
        .label = "rear_left.theta1",
        .gpio_num = SPIPPY_PIN_SERVO_BL1_PWM,
        .active = true,
        .ledc_channel = LEDC_CHANNEL_5,
        .pulse_map_min_angle_deg = SPIPPY_SERVO_THETA1_INST_MIN_DEG,
        .pulse_map_max_angle_deg = SPIPPY_SERVO_THETA1_INST_MAX_DEG,
        .default_calibration = {
            .leg_id = SPIPPY_LEG_REAR_LEFT,
            .joint_id = SPIPPY_JOINT_THETA1,
            .invert = false,
            .zero_offset_deg_x10 = 0,
            .min_angle_deg_x10 = CONFIG_SPIPPY_SERVO_THETA1_MIN_DEG * 10,
            .max_angle_deg_x10 = CONFIG_SPIPPY_SERVO_THETA1_MAX_DEG * 10,
            .safe_angle_deg_x10 = CONFIG_SPIPPY_SERVO_SAFE_THETA1_DEG * 10,
            .stand_angle_deg_x10 = SPIPPY_SERVO_THETA1_CENTER_X10,
        },
    },
    {
        .label = "rear_right.theta2",
        .gpio_num = SPIPPY_PIN_SERVO_BR2_PWM,
        .active = true,
        .ledc_channel = LEDC_CHANNEL_6,
        .pulse_map_min_angle_deg = SPIPPY_SERVO_THETA2_INST_MIN_DEG,
        .pulse_map_max_angle_deg = SPIPPY_SERVO_THETA2_INST_MAX_DEG,
        .default_calibration = {
            .leg_id = SPIPPY_LEG_REAR_RIGHT,
            .joint_id = SPIPPY_JOINT_THETA2,
            .invert = true,
            .zero_offset_deg_x10 = 0,
            .min_angle_deg_x10 = CONFIG_SPIPPY_SERVO_THETA2_MIN_DEG * 10,
            .max_angle_deg_x10 = CONFIG_SPIPPY_SERVO_THETA2_MAX_DEG * 10,
            .safe_angle_deg_x10 = CONFIG_SPIPPY_SERVO_SAFE_THETA2_DEG * 10,
            .stand_angle_deg_x10 = SPIPPY_SERVO_THETA2_CENTER_X10,
        },
    },
    {
        .label = "rear_right.theta1",
        .gpio_num = SPIPPY_PIN_SERVO_BR1_PWM,
        .active = true,
        .ledc_channel = LEDC_CHANNEL_7,
        .pulse_map_min_angle_deg = SPIPPY_SERVO_THETA1_INST_MIN_DEG,
        .pulse_map_max_angle_deg = SPIPPY_SERVO_THETA1_INST_MAX_DEG,
        .default_calibration = {
            .leg_id = SPIPPY_LEG_REAR_RIGHT,
            .joint_id = SPIPPY_JOINT_THETA1,
            .invert = true,
            .zero_offset_deg_x10 = 0,
            .min_angle_deg_x10 = CONFIG_SPIPPY_SERVO_THETA1_MIN_DEG * 10,
            .max_angle_deg_x10 = CONFIG_SPIPPY_SERVO_THETA1_MAX_DEG * 10,
            .safe_angle_deg_x10 = CONFIG_SPIPPY_SERVO_SAFE_THETA1_DEG * 10,
            .stand_angle_deg_x10 = SPIPPY_SERVO_THETA1_CENTER_X10,
        },
    },
    {
        .label = "ext.s4",
        .gpio_num = SPIPPY_PIN_SERVO_EXT4_PWM,
        .active = false,
        .ledc_channel = LEDC_CHANNEL_0,
        .pulse_map_min_angle_deg = SPIPPY_SERVO_THETA2_INST_MIN_DEG,
        .pulse_map_max_angle_deg = SPIPPY_SERVO_THETA2_INST_MAX_DEG,
    },
    {
        .label = "ext.s3",
        .gpio_num = SPIPPY_PIN_SERVO_EXT3_PWM,
        .active = false,
        .ledc_channel = LEDC_CHANNEL_0,
        .pulse_map_min_angle_deg = SPIPPY_SERVO_THETA1_INST_MIN_DEG,
        .pulse_map_max_angle_deg = SPIPPY_SERVO_THETA1_INST_MAX_DEG,
    },
    {
        .label = "ext.s2",
        .gpio_num = SPIPPY_PIN_SERVO_EXT2_PWM,
        .active = false,
        .ledc_channel = LEDC_CHANNEL_0,
        .pulse_map_min_angle_deg = SPIPPY_SERVO_THETA2_INST_MIN_DEG,
        .pulse_map_max_angle_deg = SPIPPY_SERVO_THETA2_INST_MAX_DEG,
    },
    {
        .label = "ext.s1",
        .gpio_num = SPIPPY_PIN_SERVO_EXT1_PWM,
        .active = false,
        .ledc_channel = LEDC_CHANNEL_0,
        .pulse_map_min_angle_deg = SPIPPY_SERVO_THETA1_INST_MIN_DEG,
        .pulse_map_max_angle_deg = SPIPPY_SERVO_THETA1_INST_MAX_DEG,
    },
};

static float servo_manager_clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static bool servo_manager_calibration_entry_valid(size_t channel_index,
                                                   const spippy_servo_calibration_t *calibration)
{
    if ((calibration == NULL) || (channel_index >= SPIPPY_ACTIVE_SERVO_COUNT)) {
        return false;
    }

    const spippy_servo_calibration_t *expected = &s_servo_outputs[channel_index].default_calibration;
    /*
     * NVS 中的腿/关节归属不可被校准数据重映射。这个坑会造成两个通道驱动同一关节，
     * 另一个真实关节却拿到全 0 命令；网页层虽已限制，启动加载时仍必须再次防守。
     */
    if ((calibration->leg_id != expected->leg_id) || (calibration->joint_id != expected->joint_id)) {
        return false;
    }
    if ((calibration->zero_offset_deg_x10 < -SPIPPY_SERVO_MAX_ZERO_OFFSET_X10) ||
        (calibration->zero_offset_deg_x10 > SPIPPY_SERVO_MAX_ZERO_OFFSET_X10)) {
        return false;
    }
    if (calibration->min_angle_deg_x10 > calibration->max_angle_deg_x10) {
        return false;
    }
    if ((calibration->safe_angle_deg_x10 < calibration->min_angle_deg_x10) ||
        (calibration->safe_angle_deg_x10 > calibration->max_angle_deg_x10) ||
        (calibration->stand_angle_deg_x10 < calibration->min_angle_deg_x10) ||
        (calibration->stand_angle_deg_x10 > calibration->max_angle_deg_x10)) {
        return false;
    }
    if (calibration->joint_id == SPIPPY_JOINT_THETA1) {
        if ((calibration->min_angle_deg_x10 < -900) || (calibration->max_angle_deg_x10 > 900)) {
            return false;
        }
    } else if ((calibration->min_angle_deg_x10 < 0) || (calibration->max_angle_deg_x10 > 1800)) {
        return false;
    }

    return true;
}

static float servo_manager_pose_angle_for_channel(const spippy_servo_calibration_t *calibration,
                                                  spippy_pose_mode_t pose)
{
    float safe_angle_deg = (float)calibration->safe_angle_deg_x10 / 10.0f;
    float stand_angle_deg = (float)calibration->stand_angle_deg_x10 / 10.0f;

    switch (pose) {
        case SPIPPY_POSE_SAFE:
            /*
             * Runtime SAFE is intentionally the calibrated neutral stance. At
             * boot, low battery, or fault, moving to a familiar support pose is
             * safer than folding through a separate angle that may sweep legs.
             */
            return stand_angle_deg;
        case SPIPPY_POSE_STAND:
            return stand_angle_deg;
        case SPIPPY_POSE_SIT:
            return safe_angle_deg + ((stand_angle_deg - safe_angle_deg) * 0.5f);
        case SPIPPY_POSE_CROUCH:
            return safe_angle_deg + ((stand_angle_deg - safe_angle_deg) * 0.75f);
        default:
            return safe_angle_deg;
    }
}

static void servo_manager_load_default_calibration_table(spippy_servo_calibration_t *calibration)
{
    memset(calibration, 0, sizeof(s_calibration));
    for (size_t i = 0; i < SPIPPY_ACTIVE_SERVO_COUNT; ++i) {
        calibration[i] = s_servo_outputs[i].default_calibration;
    }
}

static bool servo_manager_migrate_theta2_limits(spippy_servo_calibration_t *calibration)
{
    bool migrated = false;
    for (size_t i = 0; i < SPIPPY_ACTIVE_SERVO_COUNT; ++i) {
        if (calibration[i].joint_id != SPIPPY_JOINT_THETA2) {
            continue;
        }
        int16_t min_x10 = CONFIG_SPIPPY_SERVO_THETA2_MIN_DEG * 10;
        int16_t max_x10 = CONFIG_SPIPPY_SERVO_THETA2_MAX_DEG * 10;
        if ((calibration[i].min_angle_deg_x10 != min_x10) ||
            (calibration[i].max_angle_deg_x10 != max_x10)) {
            calibration[i].min_angle_deg_x10 = min_x10;
            calibration[i].max_angle_deg_x10 = max_x10;
            migrated = true;
        }
    }
    return migrated;
}

static esp_err_t servo_manager_load_calibration_from_storage(void)
{
    spippy_servo_calibration_blob_t blob = { 0 };
    size_t blob_size = sizeof(blob);

    servo_manager_load_default_calibration_table(s_calibration);
    s_runtime_status.calibration_source = SPIPPY_CALIBRATION_SOURCE_DEFAULTS;
    s_runtime_status.persisted_calibration_present = false;

    esp_err_t err = storage_load_blob(SPIPPY_SERVO_STORAGE_NS, SPIPPY_SERVO_STORAGE_KEY, &blob, &blob_size);
    if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGI(TAG, "no persisted servo calibration found, using installation defaults");
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "failed to load servo calibration: %s; using defaults", esp_err_to_name(err));
        return ESP_OK;
    }

    if (blob_size != sizeof(blob)) {
        ESP_LOGW(TAG, "servo calibration blob mismatch size=%u, using defaults", (unsigned)blob_size);
        return ESP_OK;
    }

    s_runtime_status.persisted_calibration_present = true;
    memcpy(s_calibration, blob.channels, sizeof(s_calibration));
    for (size_t i = 0; i < SPIPPY_ACTIVE_SERVO_COUNT; ++i) {
        if (!servo_manager_calibration_entry_valid(i, &s_calibration[i])) {
            ESP_LOGW(TAG, "invalid calibration entry for %s, reverting that channel to defaults", s_servo_outputs[i].label);
            s_calibration[i] = s_servo_outputs[i].default_calibration;
        }
    }
    if (servo_manager_migrate_theta2_limits(s_calibration)) {
        ESP_LOGI(TAG,
                 "migrated persisted theta2 limits to %d..%ddeg",
                 CONFIG_SPIPPY_SERVO_THETA2_MIN_DEG,
                 CONFIG_SPIPPY_SERVO_THETA2_MAX_DEG);
        esp_err_t save_err = servo_manager_save_calibration_to_storage();
        if (save_err != ESP_OK) {
            ESP_LOGW(TAG, "failed to persist migrated theta2 limits: %s", esp_err_to_name(save_err));
        }
    }
    s_runtime_status.calibration_source = SPIPPY_CALIBRATION_SOURCE_NVS;
    ESP_LOGI(TAG, "loaded persisted servo calibration");
    return ESP_OK;
}

static esp_err_t servo_manager_save_calibration_to_storage(void)
{
    spippy_servo_calibration_blob_t blob = { 0 };

    memcpy(blob.channels, s_calibration, sizeof(s_calibration));
    return storage_save_blob(SPIPPY_SERVO_STORAGE_NS, SPIPPY_SERVO_STORAGE_KEY, &blob, sizeof(blob));
}

static float servo_manager_clamped_logical_angle(const spippy_servo_calibration_t *calibration, float angle_deg)
{
    float min_angle_deg = (float)calibration->min_angle_deg_x10 / 10.0f;
    float max_angle_deg = (float)calibration->max_angle_deg_x10 / 10.0f;
    return servo_manager_clamp(angle_deg, min_angle_deg, max_angle_deg);
}

static float servo_manager_logical_to_installed_angle(const spippy_servo_output_t *servo,
                                                      const spippy_servo_calibration_t *calibration,
                                                      float logical_angle_deg)
{
    float clamped_logical_angle_deg = servo_manager_clamped_logical_angle(calibration, logical_angle_deg);
    float zero_offset_deg = (float)calibration->zero_offset_deg_x10 / 10.0f;
    float installed_angle_deg = clamped_logical_angle_deg;

    if (calibration->joint_id == SPIPPY_JOINT_THETA1) {
        installed_angle_deg = SPIPPY_SERVO_THETA1_INST_CENTER_DEG +
                              (calibration->invert ? -clamped_logical_angle_deg : clamped_logical_angle_deg);
    } else if ((calibration->joint_id == SPIPPY_JOINT_THETA2) && calibration->invert) {
        installed_angle_deg = 180.0f - clamped_logical_angle_deg;
    } else if (calibration->invert) {
        installed_angle_deg = -clamped_logical_angle_deg;
    }

    installed_angle_deg += zero_offset_deg;

    return servo_manager_clamp(installed_angle_deg,
                               servo->pulse_map_min_angle_deg,
                               servo->pulse_map_max_angle_deg);
}

static float servo_manager_installed_to_logical_angle(const spippy_servo_calibration_t *calibration,
                                                      float installed_angle_deg)
{
    const float zero_offset_deg = (float)calibration->zero_offset_deg_x10 / 10.0f;
    const float offset_removed_deg = installed_angle_deg - zero_offset_deg;

    if (calibration->joint_id == SPIPPY_JOINT_THETA1) {
        const float centered_deg = offset_removed_deg - SPIPPY_SERVO_THETA1_INST_CENTER_DEG;
        return calibration->invert ? -centered_deg : centered_deg;
    }
    if ((calibration->joint_id == SPIPPY_JOINT_THETA2) && calibration->invert) {
        return 180.0f - offset_removed_deg;
    }
    return calibration->invert ? -offset_removed_deg : offset_removed_deg;
}

static uint32_t servo_manager_installed_angle_to_pulse_us(const spippy_servo_output_t *servo,
                                                          float installed_angle_deg)
{
    float span_deg = servo->pulse_map_max_angle_deg - servo->pulse_map_min_angle_deg;
    float normalized = 0.5f;

    if (span_deg > 0.0f) {
        normalized = (installed_angle_deg - servo->pulse_map_min_angle_deg) / span_deg;
    }

    normalized = servo_manager_clamp(normalized, 0.0f, 1.0f);

    uint32_t pulse_us = CONFIG_SPIPPY_SERVO_MIN_PULSE_US +
                        (uint32_t)((CONFIG_SPIPPY_SERVO_MAX_PULSE_US - CONFIG_SPIPPY_SERVO_MIN_PULSE_US) * normalized);
    if (pulse_us > SPIPPY_SERVO_PERIOD_US) {
        pulse_us = SPIPPY_SERVO_PERIOD_US;
    }

    return pulse_us;
}

static esp_err_t servo_manager_write_frame(const spippy_leg_frame_t *frame)
{
    ESP_RETURN_ON_FALSE(frame != NULL, ESP_ERR_INVALID_ARG, TAG, "frame is null");
    ESP_RETURN_ON_FALSE(s_hw_ready, ESP_ERR_INVALID_STATE, TAG, "servo pwm not ready");

    /*
     * 四足步态依赖对角腿严格同相。过去每个 tick 只轮询两路舵机，导致同一帧
     * 最多跨 60 ms 才全部落地；这里统一刷新 8 路，保证 20 ms 控制帧同步生效。
     */
    for (size_t i = 0; i < SPIPPY_ACTIVE_SERVO_COUNT; ++i) {
        const spippy_servo_output_t *servo = &s_servo_outputs[i];
        const spippy_servo_calibration_t *calibration = &s_calibration[i];
        const leg_command_t *leg = &frame->legs[calibration->leg_id];
        float requested_angle_deg = (calibration->joint_id == SPIPPY_JOINT_THETA1) ? leg->theta1_deg : leg->theta2_deg;
        /*
         * 先限幅目标、再做步进和到位判断。若反过来，超限目标会让遥测声称已到位，
         * 但 PWM 实际一直卡在机械限位，后续动作的步进基准也会被污染。
         */
        float logical_angle_deg = servo_manager_clamped_logical_angle(calibration, requested_angle_deg);
        if (s_last_output[i].pulse_us != 0U) {
            float delta = logical_angle_deg - s_last_output[i].logical_angle_deg;
            if (delta > CONFIG_SPIPPY_SERVO_MAX_STEP_DEG_PER_TICK) {
                logical_angle_deg = s_last_output[i].logical_angle_deg + CONFIG_SPIPPY_SERVO_MAX_STEP_DEG_PER_TICK;
            } else if (delta < -CONFIG_SPIPPY_SERVO_MAX_STEP_DEG_PER_TICK) {
                logical_angle_deg = s_last_output[i].logical_angle_deg - CONFIG_SPIPPY_SERVO_MAX_STEP_DEG_PER_TICK;
            }
        }
        float installed_angle_deg = servo_manager_logical_to_installed_angle(servo, calibration, logical_angle_deg);
        uint32_t pulse_us = servo_manager_installed_angle_to_pulse_us(servo, installed_angle_deg);
        uint32_t duty = (uint32_t)(((uint64_t)pulse_us * SPIPPY_SERVO_MAX_DUTY) / SPIPPY_SERVO_PERIOD_US);

        ESP_RETURN_ON_ERROR(ledc_set_duty(SPIPPY_SERVO_LEDC_MODE, servo->ledc_channel, duty),
                            TAG, "set duty failed on gpio=%d", servo->gpio_num);
        ESP_RETURN_ON_ERROR(ledc_update_duty(SPIPPY_SERVO_LEDC_MODE, servo->ledc_channel),
                            TAG, "update duty failed on gpio=%d", servo->gpio_num);

        s_last_output[i].logical_angle_deg = logical_angle_deg;
        s_last_output[i].calibrated_servo_angle_deg = installed_angle_deg;
        s_last_output[i].pulse_us = pulse_us;
    }
    return ESP_OK;
}

static esp_err_t servo_manager_write_direct_outputs(void)
{
    ESP_RETURN_ON_FALSE(s_hw_ready, ESP_ERR_INVALID_STATE, TAG, "servo pwm not ready");

    for (size_t i = 0; i < SPIPPY_ACTIVE_SERVO_COUNT; ++i) {
        const spippy_servo_output_t *servo = &s_servo_outputs[i];
        const spippy_servo_calibration_t *calibration = &s_calibration[i];
        float installed_angle_deg = servo_manager_clamp(s_direct_installed_angle_offsets_deg[i],
                                                        servo->pulse_map_min_angle_deg,
                                                        servo->pulse_map_max_angle_deg);
        uint32_t pulse_us = servo_manager_installed_angle_to_pulse_us(servo, installed_angle_deg);
        uint32_t duty = (uint32_t)(((uint64_t)pulse_us * SPIPPY_SERVO_MAX_DUTY) / SPIPPY_SERVO_PERIOD_US);

        ESP_RETURN_ON_ERROR(ledc_set_duty(SPIPPY_SERVO_LEDC_MODE, servo->ledc_channel, duty),
                            TAG, "set direct duty failed on gpio=%d", servo->gpio_num);
        ESP_RETURN_ON_ERROR(ledc_update_duty(SPIPPY_SERVO_LEDC_MODE, servo->ledc_channel),
                            TAG, "update direct duty failed on gpio=%d", servo->gpio_num);

        /*
         * Direct calibration/preflight output bypasses logical joint commands,
         * but the next normal frame still uses logical_angle_deg as its slew
         * starting point. Store the inverse-mapped logical angle here so leaving
         * direct mode cannot drive mirrored joints toward a limit first.
         */
        s_last_output[i].logical_angle_deg =
            servo_manager_installed_to_logical_angle(calibration, installed_angle_deg);
        s_last_output[i].calibrated_servo_angle_deg = installed_angle_deg;
        s_last_output[i].pulse_us = pulse_us;
    }

    return ESP_OK;
}

static bool servo_manager_target_reached(const spippy_leg_frame_t *frame)
{
    for (size_t i = 0; i < SPIPPY_ACTIVE_SERVO_COUNT; ++i) {
        const spippy_servo_calibration_t *calibration = &s_calibration[i];
        const leg_command_t *leg = &frame->legs[calibration->leg_id];
        float requested_target = (calibration->joint_id == SPIPPY_JOINT_THETA1) ?
            leg->theta1_deg : leg->theta2_deg;
        float target = servo_manager_clamped_logical_angle(calibration, requested_target);
        float delta = target - s_last_output[i].logical_angle_deg;
        if ((delta > 0.25f) || (delta < -0.25f)) {
            return false;
        }
    }
    return true;
}

esp_err_t servo_manager_fill_pose_frame(spippy_pose_mode_t pose, spippy_leg_frame_t *frame)
{
    ESP_RETURN_ON_FALSE(frame != NULL, ESP_ERR_INVALID_ARG, TAG, "frame is null");

    memset(frame, 0, sizeof(*frame));
    for (size_t i = 0; i < SPIPPY_ACTIVE_SERVO_COUNT; ++i) {
        const spippy_servo_calibration_t *calibration = &s_calibration[i];
        leg_command_t *leg = &frame->legs[calibration->leg_id];
        float angle_deg = servo_manager_pose_angle_for_channel(calibration, pose);

        if (calibration->joint_id == SPIPPY_JOINT_THETA1) {
            leg->theta1_deg = angle_deg;
        } else {
            leg->theta2_deg = angle_deg;
        }
    }

    return ESP_OK;
}

esp_err_t servo_manager_init(void)
{
    ESP_RETURN_ON_ERROR(servo_manager_load_calibration_from_storage(), TAG, "failed to load calibration");

    ledc_timer_config_t ledc_timer = {
        .speed_mode = SPIPPY_SERVO_LEDC_MODE,
        .duty_resolution = SPIPPY_SERVO_LEDC_DUTY_RES,
        .timer_num = SPIPPY_SERVO_LEDC_TIMER,
        .freq_hz = CONFIG_SPIPPY_SERVO_PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ESP_RETURN_ON_ERROR(ledc_timer_config(&ledc_timer), TAG, "failed to init ledc timer");

    for (size_t i = 0; i < SPIPPY_SERVO_CHANNEL_BUDGET; ++i) {
        const spippy_servo_output_t *servo = &s_servo_outputs[i];
        if (!servo->active) {
            ESP_LOGI(TAG, "reserved pwm budget slot gpio=%d for future expansion", servo->gpio_num);
            continue;
        }

        ledc_channel_config_t channel_config = {
            .gpio_num = servo->gpio_num,
            .speed_mode = SPIPPY_SERVO_LEDC_MODE,
            .channel = servo->ledc_channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = SPIPPY_SERVO_LEDC_TIMER,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_config), TAG, "failed to init gpio=%d", servo->gpio_num);

        ESP_LOGI(TAG,
                 "map %s gpio=%d invert=%d zero=%d.%ddeg range=%d.%d..%d.%ddeg stand=%d.%ddeg",
                 servo->label,
                 servo->gpio_num,
                 s_calibration[i].invert,
                 s_calibration[i].zero_offset_deg_x10 / 10,
                 s_calibration[i].zero_offset_deg_x10 >= 0 ? s_calibration[i].zero_offset_deg_x10 % 10 : -s_calibration[i].zero_offset_deg_x10 % 10,
                 s_calibration[i].min_angle_deg_x10 / 10,
                 s_calibration[i].min_angle_deg_x10 >= 0 ? s_calibration[i].min_angle_deg_x10 % 10 : -s_calibration[i].min_angle_deg_x10 % 10,
                 s_calibration[i].max_angle_deg_x10 / 10,
                 s_calibration[i].max_angle_deg_x10 >= 0 ? s_calibration[i].max_angle_deg_x10 % 10 : -s_calibration[i].max_angle_deg_x10 % 10,
                 s_calibration[i].stand_angle_deg_x10 / 10,
                 s_calibration[i].stand_angle_deg_x10 >= 0 ? s_calibration[i].stand_angle_deg_x10 % 10 : -s_calibration[i].stand_angle_deg_x10 % 10);
    }

    ESP_RETURN_ON_ERROR(servo_manager_fill_pose_frame(SPIPPY_POSE_SAFE, &s_target_frame),
                        TAG, "failed to build safe frame");
    s_hw_ready = true;
    s_frame_dirty = true;
    s_direct_output_active = false;
    s_direct_output_dirty = false;

    ESP_LOGI(TAG,
             "initialized %d active servo outputs with %d-slot layout, pwm=%dHz pulse=%d..%dus source=%s",
             SPIPPY_ACTIVE_SERVO_COUNT,
             SPIPPY_SERVO_CHANNEL_BUDGET,
             CONFIG_SPIPPY_SERVO_PWM_FREQUENCY_HZ,
             CONFIG_SPIPPY_SERVO_MIN_PULSE_US,
             CONFIG_SPIPPY_SERVO_MAX_PULSE_US,
             (s_runtime_status.calibration_source == SPIPPY_CALIBRATION_SOURCE_NVS) ? "nvs" : "defaults");
    return ESP_OK;
}

esp_err_t servo_manager_start(void)
{
    ESP_RETURN_ON_FALSE(s_hw_ready, ESP_ERR_INVALID_STATE, TAG, "servo pwm not ready");
    ESP_RETURN_ON_ERROR(servo_manager_write_frame(&s_target_frame), TAG, "failed to apply safe frame");
    s_frame_dirty = false;

    ESP_LOGI(TAG, "safe servo output armed");
    return ESP_OK;
}

esp_err_t servo_manager_apply_frame(const spippy_leg_frame_t *frame)
{
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < SPIPPY_LEG_COUNT; ++i) {
        if (!isfinite(frame->legs[i].theta1_deg) || !isfinite(frame->legs[i].theta2_deg)) {
            ESP_LOGE(TAG, "reject non-finite servo frame at leg=%u", (unsigned)i);
            return ESP_ERR_INVALID_ARG;
        }
    }

    s_target_frame = *frame;
    s_frame_dirty = true;
    return ESP_OK;
}

esp_err_t servo_manager_get_calibration(spippy_servo_calibration_t *calibration, size_t count)
{
    if ((calibration == NULL) || (count < SPIPPY_ACTIVE_SERVO_COUNT)) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(calibration, s_calibration, sizeof(s_calibration));
    return ESP_OK;
}

esp_err_t servo_manager_set_calibration(const spippy_servo_calibration_t *calibration, size_t count, bool persist)
{
    if ((calibration == NULL) || (count < SPIPPY_ACTIVE_SERVO_COUNT)) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < SPIPPY_ACTIVE_SERVO_COUNT; ++i) {
        if (!servo_manager_calibration_entry_valid(i, &calibration[i])) {
            ESP_LOGE(TAG, "reject invalid calibration entry for channel %u", (unsigned)i);
            return ESP_ERR_INVALID_ARG;
        }
    }

    memcpy(s_calibration, calibration, sizeof(s_calibration));
    s_frame_dirty = true;

    if (persist) {
        ESP_RETURN_ON_ERROR(servo_manager_save_calibration_to_storage(), TAG, "failed to persist calibration");
        s_runtime_status.calibration_source = SPIPPY_CALIBRATION_SOURCE_NVS;
        s_runtime_status.persisted_calibration_present = true;
        ESP_LOGI(TAG, "persisted servo calibration to nvs");
    } else {
        ESP_LOGI(TAG, "applied servo calibration in ram only");
    }

    return ESP_OK;
}

esp_err_t servo_manager_reload_calibration_from_storage(void)
{
    ESP_RETURN_ON_ERROR(servo_manager_load_calibration_from_storage(), TAG, "failed to reload calibration");
    s_frame_dirty = true;
    ESP_LOGI(TAG, "reloaded servo calibration source=%s",
             (s_runtime_status.calibration_source == SPIPPY_CALIBRATION_SOURCE_NVS) ? "nvs" : "defaults");
    return ESP_OK;
}

esp_err_t servo_manager_reset_calibration_to_defaults(bool clear_persisted)
{
    servo_manager_load_default_calibration_table(s_calibration);
    s_runtime_status.calibration_source = SPIPPY_CALIBRATION_SOURCE_DEFAULTS;
    s_runtime_status.persisted_calibration_present = false;
    s_frame_dirty = true;

    if (clear_persisted) {
        esp_err_t err = storage_delete_key(SPIPPY_SERVO_STORAGE_NS, SPIPPY_SERVO_STORAGE_KEY);
        if ((err != ESP_OK) && (err != ESP_ERR_NOT_FOUND)) {
            return err;
        }
    }

    ESP_LOGI(TAG, "reverted servo calibration to installation defaults");
    return ESP_OK;
}

esp_err_t servo_manager_get_runtime_status(spippy_servo_runtime_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *status = s_runtime_status;
    return ESP_OK;
}

esp_err_t servo_manager_get_channel_info(spippy_servo_channel_info_t *info, size_t count)
{
    if ((info == NULL) || (count < SPIPPY_ACTIVE_SERVO_COUNT)) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < SPIPPY_ACTIVE_SERVO_COUNT; ++i) {
        info[i].label = s_servo_outputs[i].label;
        info[i].gpio_num = s_servo_outputs[i].gpio_num;
        info[i].leg_id = s_calibration[i].leg_id;
        info[i].joint_id = s_calibration[i].joint_id;
        info[i].active = s_servo_outputs[i].active;
    }

    return ESP_OK;
}

esp_err_t servo_manager_get_last_output_telemetry(spippy_servo_command_telemetry_t *telemetry, size_t count)
{
    if ((telemetry == NULL) || (count < SPIPPY_ACTIVE_SERVO_COUNT)) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(telemetry, s_last_output, sizeof(s_last_output));
    return ESP_OK;
}

esp_err_t servo_manager_set_calibration_direct_output(const float *installed_angle_offsets_deg, size_t count)
{
    if ((installed_angle_offsets_deg == NULL) || (count < SPIPPY_ACTIVE_SERVO_COUNT)) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < SPIPPY_ACTIVE_SERVO_COUNT; ++i) {
        if (!isfinite(installed_angle_offsets_deg[i])) {
            return ESP_ERR_INVALID_ARG;
        }
    }

    memcpy(s_direct_installed_angle_offsets_deg,
           installed_angle_offsets_deg,
           sizeof(s_direct_installed_angle_offsets_deg));
    s_direct_output_active = true;
    s_direct_output_dirty = true;
    return ESP_OK;
}

esp_err_t servo_manager_clear_calibration_direct_output(void)
{
    s_direct_output_active = false;
    s_direct_output_dirty = false;
    s_frame_dirty = true;
    return ESP_OK;
}

esp_err_t servo_manager_tick(void)
{
    if (!s_hw_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_direct_output_active) {
        if (s_direct_output_dirty) {
            ESP_RETURN_ON_ERROR(servo_manager_write_direct_outputs(), TAG, "failed to apply direct calibration outputs");
            s_direct_output_dirty = false;
        }
        return ESP_OK;
    }

    if (!s_frame_dirty) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(servo_manager_write_frame(&s_target_frame), TAG, "failed to flush frame");
    s_frame_dirty = !servo_manager_target_reached(&s_target_frame);
    ESP_LOGD(TAG, "applied servo frame seq=%" PRIu32, s_target_frame.sequence_id);
    return ESP_OK;
}
