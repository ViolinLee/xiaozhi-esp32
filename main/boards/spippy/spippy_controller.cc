#include "spippy_controller.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <cinttypes>
#include <map>

#include <esp_log.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <wifi_manager.h>

#include "application.h"
#include "assets/lang_config.h"
#include "mcp_server.h"
#include "sdkconfig.h"

#ifndef CONFIG_SPIPPY_PROXIMITY_NEAR_EXIT_MM
#define CONFIG_SPIPPY_PROXIMITY_NEAR_EXIT_MM 700
#endif
#ifndef CONFIG_SPIPPY_PROXIMITY_CLOSE_EXIT_MM
#define CONFIG_SPIPPY_PROXIMITY_CLOSE_EXIT_MM 430
#endif
#ifndef CONFIG_SPIPPY_PROXIMITY_TOO_CLOSE_EXIT_MM
#define CONFIG_SPIPPY_PROXIMITY_TOO_CLOSE_EXIT_MM 260
#endif
#ifndef CONFIG_SPIPPY_PROXIMITY_STABLE_CONFIRM_COUNT
#define CONFIG_SPIPPY_PROXIMITY_STABLE_CONFIRM_COUNT 2
#endif
#ifndef CONFIG_SPIPPY_PROXIMITY_LOST_CONFIRM_COUNT
#define CONFIG_SPIPPY_PROXIMITY_LOST_CONFIRM_COUNT 3
#endif
#ifndef CONFIG_SPIPPY_FORWARD_GUARD_RELEASE_MM
#define CONFIG_SPIPPY_FORWARD_GUARD_RELEASE_MM 350
#endif
#ifndef CONFIG_SPIPPY_FORWARD_GUARD_CLEAR_CONFIRM_COUNT
#define CONFIG_SPIPPY_FORWARD_GUARD_CLEAR_CONFIRM_COUNT 2
#endif
#ifndef CONFIG_SPIPPY_FORWARD_GUARD_NO_ECHO_CLEAR_COUNT
#define CONFIG_SPIPPY_FORWARD_GUARD_NO_ECHO_CLEAR_COUNT 4
#endif
#ifndef CONFIG_SPIPPY_FORWARD_GUARD_ERROR_TO_UNKNOWN_COUNT
#define CONFIG_SPIPPY_FORWARD_GUARD_ERROR_TO_UNKNOWN_COUNT 3
#endif
#ifndef CONFIG_SPIPPY_FORWARD_GUARD_STALE_TIMEOUT_MS
#define CONFIG_SPIPPY_FORWARD_GUARD_STALE_TIMEOUT_MS 750
#endif
#ifndef CONFIG_SPIPPY_AUTONOMY_ENABLE
#define CONFIG_SPIPPY_AUTONOMY_ENABLE 1
#endif
#ifndef CONFIG_SPIPPY_AUTONOMY_IDLE_TRIGGER_S
#define CONFIG_SPIPPY_AUTONOMY_IDLE_TRIGGER_S 45
#endif
#ifndef CONFIG_SPIPPY_AUTONOMY_MIN_INTERVAL_S
#define CONFIG_SPIPPY_AUTONOMY_MIN_INTERVAL_S 60
#endif
#ifndef CONFIG_SPIPPY_AUTONOMY_MAX_INTERVAL_S
#define CONFIG_SPIPPY_AUTONOMY_MAX_INTERVAL_S 180
#endif
#ifndef CONFIG_SPIPPY_TASK_AUTONOMY_PERIOD_MS
#define CONFIG_SPIPPY_TASK_AUTONOMY_PERIOD_MS 1000
#endif
#ifndef CONFIG_SPIPPY_CONTINUOUS_MOVE_MAX_DURATION_S
#define CONFIG_SPIPPY_CONTINUOUS_MOVE_MAX_DURATION_S 30
#endif
#ifndef CONFIG_SPIPPY_LOW_POWER_CHAT_REMINDER_COOLDOWN_MS
#define CONFIG_SPIPPY_LOW_POWER_CHAT_REMINDER_COOLDOWN_MS 180000
#endif

namespace {

constexpr const char *TAG = "SpippyController";

static_assert(CONFIG_SPIPPY_POWER_RECOVER_VOLTAGE_MV > CONFIG_SPIPPY_POWER_LOW_VOLTAGE_MV,
              "low-power recovery voltage must exceed latch voltage");
#if CONFIG_SPIPPY_PROXIMITY_ENABLE
static_assert(CONFIG_SPIPPY_PROXIMITY_TOO_CLOSE_ENTER_MM < CONFIG_SPIPPY_PROXIMITY_CLOSE_ENTER_MM &&
              CONFIG_SPIPPY_PROXIMITY_CLOSE_ENTER_MM < CONFIG_SPIPPY_PROXIMITY_NEAR_ENTER_MM,
              "proximity enter thresholds must be ordered from too-close to near");
static_assert(CONFIG_SPIPPY_PROXIMITY_TOO_CLOSE_EXIT_MM > CONFIG_SPIPPY_PROXIMITY_TOO_CLOSE_ENTER_MM &&
              CONFIG_SPIPPY_PROXIMITY_CLOSE_EXIT_MM > CONFIG_SPIPPY_PROXIMITY_CLOSE_ENTER_MM &&
              CONFIG_SPIPPY_PROXIMITY_NEAR_EXIT_MM > CONFIG_SPIPPY_PROXIMITY_NEAR_ENTER_MM,
              "each proximity exit threshold must exceed its enter threshold");
static_assert(CONFIG_SPIPPY_PROXIMITY_TOO_CLOSE_ENTER_MM < CONFIG_SPIPPY_PROXIMITY_FORWARD_LIMIT_MM &&
              CONFIG_SPIPPY_PROXIMITY_FORWARD_LIMIT_MM < CONFIG_SPIPPY_FORWARD_GUARD_RELEASE_MM,
              "forward guard thresholds must be ordered from critical to release");
static_assert(CONFIG_SPIPPY_FORWARD_GUARD_STALE_TIMEOUT_MS > CONFIG_SPIPPY_TASK_SENSOR_PERIOD_MS,
              "forward guard stale timeout must exceed the sensor period");
#endif

std::string NormalizeToken(const std::string &text) {
    std::string token;
    token.reserve(text.size());
    for (unsigned char c : text) {
        if (c == ' ' || c == '_' || c == '-' || c == '.') {
            continue;
        }
        if (c < 128) {
            token.push_back(static_cast<char>(std::tolower(c)));
        } else {
            token.push_back(static_cast<char>(c));
        }
    }
    return token;
}

void AddPowerTelemetry(cJSON *root, const spippy_power_telemetry_t &power) {
    cJSON_AddNumberToObject(root, "raw_adc", power.raw_adc);
    cJSON_AddNumberToObject(root, "pin_voltage_mv", power.pin_voltage_mv);
    cJSON_AddNumberToObject(root, "battery_voltage_mv", power.battery_voltage_mv);
    cJSON_AddNumberToObject(root, "filtered_battery_voltage_mv", power.filtered_battery_voltage_mv);
    cJSON_AddBoolToObject(root, "low_power_latched", power.low_power_latched);
    cJSON_AddBoolToObject(root, "low_power_warning", power.low_power_warning);
    cJSON_AddBoolToObject(root, "recovery_ready", power.recovery_ready);
    cJSON_AddStringToObject(root, "state", power.state);
}

cJSON *PowerTelemetryJson(const spippy_power_telemetry_t &power) {
    cJSON *root = cJSON_CreateObject();
    AddPowerTelemetry(root, power);
    return root;
}

const char *PoseName(spippy_pose_mode_t pose) {
    switch (pose) {
        case SPIPPY_POSE_SAFE:
            return "safe";
        case SPIPPY_POSE_STAND:
            return "stand";
        case SPIPPY_POSE_SIT:
            return "sit";
        case SPIPPY_POSE_CROUCH:
            return "crouch";
        default:
            return "unknown";
    }
}

uint32_t PeriodTicks(uint32_t target_ms) {
    uint32_t base_ms = std::max<uint32_t>(1, CONFIG_SPIPPY_TASK_MOTION_PERIOD_MS);
    return std::max<uint32_t>(1, (target_ms + base_ms - 1) / base_ms);
}

TickType_t DelayTicks(uint32_t delay_ms) {
    /* FreeRTOS 默认 100 Hz；5 ms 会被 pdMS_TO_TICKS 截成 0，造成任务空转占满 CPU。 */
    return std::max<TickType_t>(1, pdMS_TO_TICKS(delay_ms));
}

int64_t MsToUs(uint32_t ms) {
    return static_cast<int64_t>(ms) * 1000;
}

int64_t SecToUs(uint32_t seconds) {
    return static_cast<int64_t>(seconds) * 1000000;
}

class RecursiveLockGuard {
public:
    explicit RecursiveLockGuard(SemaphoreHandle_t lock) : lock_(lock) {
        if (lock_ != nullptr) {
            xSemaphoreTakeRecursive(lock_, portMAX_DELAY);
        }
    }

    ~RecursiveLockGuard() {
        if (lock_ != nullptr) {
            xSemaphoreGiveRecursive(lock_);
        }
    }

private:
    SemaphoreHandle_t lock_;
};

}  // namespace

SpippyController::SpippyController(SpippyDisplay *display) : display_(display) {
    lock_ = xSemaphoreCreateMutex();
    runtime_lock_ = xSemaphoreCreateRecursiveMutex();
}

SpippyController::~SpippyController() {
    if (task_ != nullptr) {
        vTaskDelete(task_);
    }
    if (lock_ != nullptr) {
        vSemaphoreDelete(lock_);
    }
    if (runtime_lock_ != nullptr) {
        vSemaphoreDelete(runtime_lock_);
    }
}

void SpippyController::Initialize() {
    ESP_ERROR_CHECK(lock_ != nullptr && runtime_lock_ != nullptr ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(storage_init());
    ESP_ERROR_CHECK(storage_start());
    ESP_ERROR_CHECK(servo_manager_init());
    ESP_ERROR_CHECK(motion_engine_init());
    ESP_ERROR_CHECK(power_manager_init());
    ESP_ERROR_CHECK(sensor_manager_init());
    spippy_low_power_reminder_init(&low_power_reminder_,
                                   CONFIG_SPIPPY_LOW_POWER_CHAT_REMINDER_COOLDOWN_MS);
#if CONFIG_SPIPPY_PROXIMITY_ENABLE
    const spippy_forward_guard_config_t forward_guard_config = {
        .block_distance_mm = CONFIG_SPIPPY_PROXIMITY_FORWARD_LIMIT_MM,
        .critical_distance_mm = CONFIG_SPIPPY_PROXIMITY_TOO_CLOSE_ENTER_MM,
        .release_distance_mm = CONFIG_SPIPPY_FORWARD_GUARD_RELEASE_MM,
        .clear_confirm_count = CONFIG_SPIPPY_FORWARD_GUARD_CLEAR_CONFIRM_COUNT,
        .no_echo_clear_count = CONFIG_SPIPPY_FORWARD_GUARD_NO_ECHO_CLEAR_COUNT,
        .error_to_unknown_count = CONFIG_SPIPPY_FORWARD_GUARD_ERROR_TO_UNKNOWN_COUNT,
        .stale_timeout_ms = CONFIG_SPIPPY_FORWARD_GUARD_STALE_TIMEOUT_MS,
    };
    spippy_forward_guard_init(&forward_guard_, &forward_guard_config);
    ESP_LOGI(TAG, "forward guard initialized block=%dmm critical=%dmm release=%dmm stale=%dms",
             CONFIG_SPIPPY_PROXIMITY_FORWARD_LIMIT_MM,
             CONFIG_SPIPPY_PROXIMITY_TOO_CLOSE_ENTER_MM,
             CONFIG_SPIPPY_FORWARD_GUARD_RELEASE_MM,
             CONFIG_SPIPPY_FORWARD_GUARD_STALE_TIMEOUT_MS);
#endif
    ESP_ERROR_CHECK(calibration_service_init());
    esp_err_t buzzer_err = buzzer_manager_init();
    if (buzzer_err != ESP_OK) {
        ESP_LOGW(TAG, "buzzer initialization failed: %s", esp_err_to_name(buzzer_err));
    }
    ESP_ERROR_CHECK(servo_manager_start());
    ESP_ERROR_CHECK(motion_engine_start());
    ESP_ERROR_CHECK(power_manager_start());
    ESP_ERROR_CHECK(sensor_manager_start());
    ESP_ERROR_CHECK(calibration_service_start());
    if (buzzer_err == ESP_OK) {
        buzzer_err = buzzer_manager_start();
        if (buzzer_err != ESP_OK) {
            ESP_LOGW(TAG, "buzzer task start failed: %s", esp_err_to_name(buzzer_err));
        }
    }
    ESP_ERROR_CHECK(power_manager_tick());
    ESP_ERROR_CHECK(power_manager_get_telemetry(&power_));
    if (display_ != nullptr) {
        int mv = power_.filtered_battery_voltage_mv > 0 ? power_.filtered_battery_voltage_mv : power_.battery_voltage_mv;
        display_->SetBatteryLevel(BatteryPercentFromVoltage(mv), power_.low_power_latched);
    }
    ESP_ERROR_CHECK(motion_engine_set_pose_mode(SPIPPY_POSE_STAND));
    uint8_t stored_autonomy = 1;
    esp_err_t autonomy_err = storage_load_u8("spippy", "autonomy", &stored_autonomy);
    autonomy_enabled_ = autonomy_err == ESP_OK ? stored_autonomy != 0 : true;
    if (autonomy_err != ESP_OK && autonomy_err != ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "failed to load autonomy setting: %s", esp_err_to_name(autonomy_err));
    }
    initialized_ = true;
    NoteUserActivity();
    BaseType_t created = xTaskCreate(TaskEntry, "spippy_ctrl", CONFIG_SPIPPY_TASK_CONTROL_STACK_SIZE,
                                     this, 4, &task_);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

bool SpippyController::GetBatteryLevel(int &level, bool &charging, bool &discharging) {
    RecursiveLockGuard runtime(runtime_lock_);
    power_manager_get_telemetry(&power_);
    int mv = power_.filtered_battery_voltage_mv > 0 ? power_.filtered_battery_voltage_mv : power_.battery_voltage_mv;
    if (mv <= 0) {
        return false;
    }
    level = BatteryPercentFromVoltage(mv);
    charging = false;
    discharging = true;
    return true;
}

void SpippyController::RegisterMcpTools() {
    auto &mcp = McpServer::GetInstance();

    mcp.AddTool("self.robot.move",
        "控制自己移动。action 支持 forward/前进、backward/后退、turn_left/左转、turn_right/右转、stop/停止。"
        "mode 支持 steps（指定步数，普通前进/后退/转向默认使用，steps 默认 3）、duration（按 duration_ms 持续）和 continuous（一直移动直到收到 stop，另有安全超时）。"
        "用户说走 N 步时必须使用 mode=steps、steps=N；说移动 N 秒时使用 mode=duration；说一直走/持续走直到停止时使用 mode=continuous。"
        "所有前进模式共用前向保护；确认有障碍或传感器状态未知时不得绕过保护。"
        "若返回 low_power_lock，必须明确告诉用户：电量低，暂时不能运动，请先充电。",
        PropertyList({
            Property("action", kPropertyTypeString),
            Property("mode", kPropertyTypeString, std::string("steps")),
            Property("steps", kPropertyTypeInteger, 3, 1, 50),
            Property("duration_ms", kPropertyTypeInteger, 3000, 500, 30000),
        }),
        [this](const PropertyList &properties) -> ReturnValue {
            return Move(properties["action"].value<std::string>(),
                        properties["mode"].value<std::string>(),
                        properties["steps"].value<int>(),
                        properties["duration_ms"].value<int>());
        });

    mcp.AddTool("self.robot.perform",
        "让自己执行一个表演动作。action 支持 wave/挥手、bow/鞠躬、shake/抖一抖、sit/坐下、pushup/俯卧撑、dance_1/跳舞、dance_2/唱歌、bounce/蹦一蹦、pose_1/摆 pose、idle_breathe/呼吸、nod/点头、cute_wiggle/撒娇、startle/吓一跳、confused/困惑、afraid/害怕、angry/生气、sleepy/困困、voice_perform/武术、pee_right_rear/撒尿、prone/趴下；repeat 为重复次数，范围 1 到 12。"
        "若返回 low_power_lock，必须明确告诉用户：电量低，暂时不能表演，请先充电。",
        PropertyList({Property("action", kPropertyTypeString), Property("repeat", kPropertyTypeInteger, 1, 1, 12)}),
        [this](const PropertyList &properties) -> ReturnValue {
            return Perform(properties["action"].value<std::string>(), properties["repeat"].value<int>());
        });

    mcp.AddTool("self.robot.expression",
        "展示不伴随运动的表情。expression 支持 idle/待机、happy/开心、sad/难过、confused/困惑、rest/休息、point/卖萌、dance/跳舞、wave/挥手、bow/鞠躬、pushup/俯卧撑、shake/惊讶、angry/生气，也可使用已注册的 sesame_* 表情资源名；duration_ms 范围 300 到 10000。",
        PropertyList({Property("expression", kPropertyTypeString), Property("duration_ms", kPropertyTypeInteger, 1800, 300, 10000)}),
        [this](const PropertyList &properties) -> ReturnValue {
            return ShowExpression(properties["expression"].value<std::string>(), properties["duration_ms"].value<int>());
        });

    mcp.AddTool("self.robot.speed_control",
        "设置自己的运动速度。speed 支持 low/slow/慢、medium/normal/正常、high/fast/快。",
        PropertyList({Property("speed", kPropertyTypeString)}),
        [this](const PropertyList &properties) -> ReturnValue {
            return SetSpeed(properties["speed"].value<std::string>());
        });

    mcp.AddTool("self.robot.showcase",
        "执行一组连续展示动作。mode 支持 cute/卖萌、dance/跳舞、skills/功夫、all/全部。"
        "若返回 low_power_lock，必须明确告诉用户：电量低，暂时不能表演，请先充电。",
        PropertyList({Property("mode", kPropertyTypeString, std::string("all"))}),
        [this](const PropertyList &properties) -> ReturnValue {
            return Showcase(properties["mode"].value<std::string>());
        });

    mcp.AddTool("self.robot.status", "获取自己的状态，包括运动、电量、距离和接近区域。",
        PropertyList(), [this](const PropertyList &) -> ReturnValue { return StatusJson(); });
    mcp.AddTool("self.robot.battery",
        "用户询问电量、电池、是否需要充电时必须调用。返回实时电压、电量百分比和低电保护状态；"
        "低电时必须明确建议用户充电。",
        PropertyList(), [this](const PropertyList &) -> ReturnValue {
            MarkLowPowerReminderHandledByResponse();
            return BatteryJson();
        });
    mcp.AddTool("self.robot.distance", "获取自己的超声波距离和接近区域。",
        PropertyList(), [this](const PropertyList &) -> ReturnValue { return DistanceJson(); });
    mcp.AddTool("self.robot.web_console",
        "当用户询问后台地址、校准网页地址、局域网访问地址、机器人 IP，或想打开网页校准时，必须调用此工具。"
        "返回当前局域网 IP 和完整校准网址；不要猜测或编造地址。",
        PropertyList(), [this](const PropertyList &) -> ReturnValue { return WebConsoleJson(); });
}

cJSON *SpippyController::WebConsoleJson() {
    const std::string ip = WifiManager::GetInstance().GetIpAddress();
    if (ip.empty()) {
        return ErrorJson("network_unavailable", "Wi-Fi is not connected; no LAN web address is available");
    }

    const std::string url = "http://" + ip + "/spippy";
    if (display_ != nullptr) {
        display_->ShowWebConsoleAddress(url, 10000);
    }

    cJSON *root = OkJson("calibration web console address");
    cJSON_AddBoolToObject(root, "connected", true);
    cJSON_AddStringToObject(root, "ip", ip.c_str());
    cJSON_AddStringToObject(root, "url", url.c_str());
    cJSON_AddStringToObject(root, "path", "/spippy");
    cJSON_AddStringToObject(root, "access_scope", "same_lan");
    return root;
}

cJSON *SpippyController::Move(const std::string &action, const std::string &mode,
                              int steps, int duration_ms) {
    RecursiveLockGuard runtime(runtime_lock_);
    NoteUserActivity();
    spippy_action_t resolved = SPIPPY_ACTION_IDLE;
    if (!ResolveAction(action, resolved, true)) {
        return ErrorJson("unsupported_action", "unsupported movement action");
    }
    if (resolved == SPIPPY_ACTION_STOP || resolved == SPIPPY_ACTION_IDLE) {
        return StartAction(SPIPPY_ACTION_STOP, 1, SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE);
    }

    CompletionMode completion_mode = CompletionMode::kCycles;
    if (!ResolveMovementMode(mode, completion_mode)) {
        return ErrorJson("unsupported_move_mode", "mode must be steps, duration or continuous");
    }
    const uint32_t safe_steps = static_cast<uint32_t>(std::clamp(steps, 1, 50));
    const uint32_t safe_duration_ms = static_cast<uint32_t>(std::clamp(duration_ms, 500, 30000));
    return StartMovement(resolved, completion_mode, safe_steps, safe_duration_ms);
}

cJSON *SpippyController::Perform(const std::string &action, int repeat) {
    RecursiveLockGuard runtime(runtime_lock_);
    NoteUserActivity();
    spippy_action_t resolved = SPIPPY_ACTION_IDLE;
    if (!ResolveAction(action, resolved, false)) {
        return ErrorJson("unsupported_action", "unsupported performance action");
    }
    PreemptLocomotionForUserCommand();
    const uint32_t safe_repeat = static_cast<uint32_t>(std::clamp(repeat, 1, 12));
    return StartAction(resolved, safe_repeat, SPIPPY_ACTION_COMPLETION_POLICY_HOLD_FINAL_POSE);
}

cJSON *SpippyController::ShowExpression(const std::string &expression, int duration_ms) {
    RecursiveLockGuard runtime(runtime_lock_);
    NoteUserActivity();
    if (display_ == nullptr) {
        return ErrorJson("display_unavailable", "expression display is unavailable");
    }

    const std::string token = NormalizeToken(expression);
    static const std::map<std::string, const char *> expressions = {
        {"idle", "idle"}, {"neutral", "idle"}, {"待机", "idle"},
        {"happy", "happy"}, {"开心", "happy"}, {"高兴", "happy"}, {"笑", "happy"},
        {"sad", "sad"}, {"难过", "sad"}, {"伤心", "sad"},
        {"confused", "confused"}, {"困惑", "confused"}, {"疑惑", "confused"},
        {"rest", "rest"}, {"sleepy", "rest"}, {"休息", "rest"}, {"困困", "rest"},
        {"point", "point"}, {"卖萌", "point"}, {"pose", "point"},
        {"dance", "dance"}, {"跳舞", "dance"}, {"唱歌", "dance"},
        {"wave", "wave"}, {"挥手", "wave"},
        {"bow", "bow"}, {"鞠躬", "bow"},
        {"pushup", "pushup"}, {"俯卧撑", "pushup"},
        {"shake", "shake"}, {"surprised", "shake"}, {"惊讶", "shake"},
        {"angry", "angry"}, {"生气", "angry"},
        {"freaky", "confused"}, {"害怕", "confused"},
        {"stand", "stand"}, {"站立", "stand"},
    };

    const char *profile = expression.c_str();
    auto it = expressions.find(token);
    if (it != expressions.end()) {
        profile = it->second;
    }
    const uint32_t duration = static_cast<uint32_t>(std::max(300, std::min(duration_ms, 10000)));
    if (!display_->ShowExpression(profile, duration)) {
        return ErrorJson("unsupported_expression", "unsupported expression or face resource name");
    }

    cJSON *root = OkJson("expression_shown");
    cJSON_AddStringToObject(root, "expression", profile);
    cJSON_AddNumberToObject(root, "duration_ms", duration);
    return root;
}

cJSON *SpippyController::SetSpeed(const std::string &speed) {
    RecursiveLockGuard runtime(runtime_lock_);
    NoteUserActivity();
    spippy_speed_level_t resolved = SPIPPY_SPEED_LEVEL_MID;
    if (!ResolveSpeed(speed, resolved)) {
        return ErrorJson("unsupported_speed", "speed must be low, medium or high");
    }
    speed_ = resolved;
    walk_gait_params_t gait = {.gait = SPIPPY_GAIT_TROT, .speed_level = speed_};
    esp_err_t err = motion_engine_set_walk_params(&gait);
    if (err != ESP_OK) {
        return ErrorJson("speed_update_failed", esp_err_to_name(err));
    }
    cJSON *root = OkJson("speed updated");
    cJSON_AddStringToObject(root, "speed", speed_ == SPIPPY_SPEED_LEVEL_LOW ? "low" : speed_ == SPIPPY_SPEED_LEVEL_HIGH ? "high" : "medium");
    return root;
}

cJSON *SpippyController::Showcase(const std::string &mode) {
    RecursiveLockGuard runtime(runtime_lock_);
    NoteUserActivity();
    const std::string token = NormalizeToken(mode);
    if (token == "cute" || token == "萌" || token == "卖萌") {
        PreemptLocomotionForUserCommand();
        return StartSequence({
            {SPIPPY_ACTION_WAVE, 1, SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE},
            {SPIPPY_ACTION_NOD, 1, SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE},
            {SPIPPY_ACTION_COQUETRY, 1, SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE},
            {SPIPPY_ACTION_CUTE_WIGGLE, 2, SPIPPY_ACTION_COMPLETION_POLICY_HOLD_FINAL_POSE},
        });
    }
    if (token == "dance" || token == "跳舞" || token == "唱歌") {
        PreemptLocomotionForUserCommand();
        return StartSequence({
            {SPIPPY_ACTION_DANCE_1, 1, SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE},
            {SPIPPY_ACTION_DANCE_2, 1, SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE},
            {SPIPPY_ACTION_BOUNCE, 1, SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE},
            {SPIPPY_ACTION_VOICE_PERFORM, 1, SPIPPY_ACTION_COMPLETION_POLICY_HOLD_FINAL_POSE},
        });
    }
    if (token == "skills" || token == "skill" || token == "功夫" || token == "武术") {
        PreemptLocomotionForUserCommand();
        return StartSequence({
            {SPIPPY_ACTION_BOW, 1, SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE},
            {SPIPPY_ACTION_PUSHUP, 2, SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE},
            {SPIPPY_ACTION_VOICE_PERFORM, 1, SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE},
            {SPIPPY_ACTION_POSE_1, 1, SPIPPY_ACTION_COMPLETION_POLICY_HOLD_FINAL_POSE},
        });
    }
    if (token == "all" || token == "全部" || token.empty()) {
        PreemptLocomotionForUserCommand();
        return StartSequence({
            {SPIPPY_ACTION_WAVE, 1, SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE},
            {SPIPPY_ACTION_BOW, 1, SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE},
            {SPIPPY_ACTION_DANCE_1, 1, SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE},
            {SPIPPY_ACTION_PUSHUP, 1, SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE},
            {SPIPPY_ACTION_COQUETRY, 1, SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE},
            {SPIPPY_ACTION_VOICE_PERFORM, 1, SPIPPY_ACTION_COMPLETION_POLICY_HOLD_FINAL_POSE},
        });
    }
    return ErrorJson("unsupported_showcase", "mode must be cute, dance, skills or all");
}

cJSON *SpippyController::StartMovement(spippy_action_t action, CompletionMode mode,
                                       uint32_t steps, uint32_t duration_ms) {
    RecursiveLockGuard runtime(runtime_lock_);
    if (!initialized_ || !IsLocomotionAction(action)) {
        return ErrorJson("invalid_movement", "movement controller is not ready or action is not locomotion");
    }
    esp_err_t power_err = power_manager_get_telemetry(&power_);
    if (power_err != ESP_OK) {
        return ErrorJson("power_state_unavailable", esp_err_to_name(power_err));
    }
    if (power_.low_power_latched) {
        return LowPowerErrorJson();
    }
    if (CalibrationPreviewActive()) {
        return ErrorJson("calibration_preview_active", "movement is locked while calibration preview is active");
    }
    if (ServoPreflightActive()) {
        return ErrorJson("servo_preflight_active", "movement is locked while servo preflight is running");
    }
    if (action == SPIPPY_ACTION_FORWARD) {
        if (cJSON *guard_error = ForwardGuardErrorJson(); guard_error != nullptr) {
            return guard_error;
        }
    }

    const int64_t now_us = esp_timer_get_time();
    Request next = {};
    next.action = action;
    next.completion_mode = mode;
    next.completion_policy = SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE;
    next.active = true;
    switch (mode) {
        case CompletionMode::kCycles:
            next.repeat_remaining = std::max<uint32_t>(1, steps);
            break;
        case CompletionMode::kDeadline:
            next.deadline_us = now_us + MsToUs(duration_ms);
            break;
        case CompletionMode::kExplicitStop:
            next.deadline_us = now_us + SecToUs(CONFIG_SPIPPY_CONTINUOUS_MOVE_MAX_DURATION_S);
            break;
    }

    xSemaphoreTake(lock_, portMAX_DELAY);
    const bool continued_without_transition = request_.active &&
        request_.action == action && IsLocomotionAction(request_.action);
    request_ = next;
    /* 用户的方向指令拥有控制权；不能排在持续运动或表演队列之后。 */
    pending_count_ = 0;
    xSemaphoreGive(lock_);

    if (!continued_without_transition) {
        walk_gait_params_t gait = {.gait = SPIPPY_GAIT_TROT, .speed_level = speed_};
        esp_err_t motion_err = motion_engine_set_walk_params(&gait);
        if (motion_err == ESP_OK) {
            motion_err = motion_engine_set_action(action);
        }
        if (motion_err != ESP_OK) {
            xSemaphoreTake(lock_, portMAX_DELAY);
            request_ = Request{};
            xSemaphoreGive(lock_);
            motion_engine_hold_current_frame(SPIPPY_ACTION_IDLE, SPIPPY_POSE_STAND);
            motion_engine_set_pose_mode(SPIPPY_POSE_STAND);
            return ErrorJson("movement_start_failed", esp_err_to_name(motion_err));
        }
        SetFaceForAction(action);
    }

    ESP_LOGI(TAG, "movement accepted action=%s mode=%s steps=%" PRIu32 " duration_ms=%" PRIu32
                  " continued=%d",
             ActionName(action), CompletionModeName(mode), steps, duration_ms, continued_without_transition);
    return MovementAcceptedJson(action, mode, steps, duration_ms, continued_without_transition);
}

cJSON *SpippyController::StartAction(spippy_action_t action, uint32_t repeat, spippy_action_completion_policy_t policy) {
    RecursiveLockGuard runtime(runtime_lock_);
    if (!initialized_) {
        return ErrorJson("not_ready", "robot controller is not initialized");
    }
    power_manager_get_telemetry(&power_);
    if (power_.low_power_latched && action != SPIPPY_ACTION_IDLE && action != SPIPPY_ACTION_STOP) {
        return LowPowerErrorJson();
    }
    if (CalibrationPreviewActive() && action != SPIPPY_ACTION_IDLE && action != SPIPPY_ACTION_STOP) {
        return ErrorJson("calibration_preview_active", "movement is locked while calibration preview is active");
    }
    if (ServoPreflightActive() && action != SPIPPY_ACTION_IDLE && action != SPIPPY_ACTION_STOP) {
        return ErrorJson("servo_preflight_active", "movement is locked while servo preflight is running");
    }
    repeat = std::max<uint32_t>(1, std::min<uint32_t>(repeat, 12));
    xSemaphoreTake(lock_, portMAX_DELAY);
    if (action == SPIPPY_ACTION_STOP) {
        pending_count_ = 0;
        request_ = Request{};
        xSemaphoreGive(lock_);
        esp_err_t err = motion_engine_hold_current_frame(SPIPPY_ACTION_IDLE, SPIPPY_POSE_STAND);
        if (err == ESP_OK) {
            err = motion_engine_set_pose_mode(SPIPPY_POSE_STAND);
        }
        if (err != ESP_OK) {
            return ErrorJson("stop_failed", esp_err_to_name(err));
        }
        if (display_ != nullptr) {
            display_->SetRobotState("idle");
        }
        return OkJson("stopped");
    }

    if (request_.active) {
        QueuedAction queued = {action, repeat, policy};
        if (!EnqueueActionLocked(queued)) {
            xSemaphoreGive(lock_);
            return ErrorJson("action_queue_full", "action queue is full; wait or stop before retrying");
        }
        const size_t queue_position = pending_count_;
        xSemaphoreGive(lock_);
        cJSON *root = ActionAcceptedJson(action, repeat);
        cJSON_AddBoolToObject(root, "queued", true);
        cJSON_AddNumberToObject(root, "queue_position", queue_position);
        return root;
    }

    request_.action = action;
    request_.repeat_remaining = repeat;
    request_.completion_mode = CompletionMode::kCycles;
    request_.deadline_us = 0;
    request_.completion_policy = policy;
    request_.active = true;
    xSemaphoreGive(lock_);

    walk_gait_params_t gait = {.gait = SPIPPY_GAIT_TROT, .speed_level = speed_};
    esp_err_t motion_err = motion_engine_set_walk_params(&gait);
    if (motion_err == ESP_OK) {
        motion_err = motion_engine_set_action(action);
    }
    if (motion_err != ESP_OK) {
        xSemaphoreTake(lock_, portMAX_DELAY);
        request_ = Request{};
        xSemaphoreGive(lock_);
        return ErrorJson("action_start_failed", esp_err_to_name(motion_err));
    }
    SetFaceForAction(action);
    cJSON *root = ActionAcceptedJson(action, repeat);
    cJSON_AddBoolToObject(root, "queued", false);
    cJSON_AddNumberToObject(root, "queue_position", 0);
    return root;
}

cJSON *SpippyController::StartSequence(std::initializer_list<QueuedAction> sequence) {
    RecursiveLockGuard runtime(runtime_lock_);
    if (!initialized_) {
        return ErrorJson("not_ready", "robot controller is not initialized");
    }
    if (sequence.size() == 0) {
        return ErrorJson("empty_sequence", "showcase sequence is empty");
    }
    if (sequence.size() > kMaxPendingActions + 1) {
        return ErrorJson("sequence_too_long", "showcase sequence is too long");
    }
    power_manager_get_telemetry(&power_);
    if (power_.low_power_latched) {
        return LowPowerErrorJson();
    }
    if (CalibrationPreviewActive()) {
        return ErrorJson("calibration_preview_active", "movement is locked while calibration preview is active");
    }
    if (ServoPreflightActive()) {
        return ErrorJson("servo_preflight_active", "movement is locked while servo preflight is running");
    }
    for (const auto &item : sequence) {
        if (item.action == SPIPPY_ACTION_FORWARD) {
            if (cJSON *guard_error = ForwardGuardErrorJson(); guard_error != nullptr) {
                return guard_error;
            }
        }
    }

    xSemaphoreTake(lock_, portMAX_DELAY);
    const bool already_active = request_.active;
    const size_t required_slots = already_active ? sequence.size() : sequence.size() - 1;
    if (required_slots > kMaxPendingActions - pending_count_) {
        xSemaphoreGive(lock_);
        return ErrorJson("action_queue_full", "showcase does not fit in the remaining action queue");
    }

    auto it = sequence.begin();
    QueuedAction first = *it;
    first.repeat = std::max<uint32_t>(1, std::min<uint32_t>(first.repeat, 12));
    if (!already_active) {
        request_.action = first.action;
        request_.repeat_remaining = first.repeat;
        request_.completion_mode = CompletionMode::kCycles;
        request_.deadline_us = 0;
        request_.completion_policy = first.completion_policy;
        request_.active = true;
        ++it;
    }
    for (; it != sequence.end(); ++it) {
        QueuedAction item = *it;
        item.repeat = std::max<uint32_t>(1, std::min<uint32_t>(item.repeat, 12));
        if (item.action != SPIPPY_ACTION_IDLE && item.action != SPIPPY_ACTION_STOP) {
            EnqueueActionLocked(item);
        }
    }
    const size_t queue_depth = pending_count_;
    xSemaphoreGive(lock_);

    if (!already_active) {
        walk_gait_params_t gait = {.gait = SPIPPY_GAIT_TROT, .speed_level = speed_};
        esp_err_t motion_err = motion_engine_set_walk_params(&gait);
        if (motion_err == ESP_OK) {
            motion_err = motion_engine_set_action(first.action);
        }
        if (motion_err != ESP_OK) {
            xSemaphoreTake(lock_, portMAX_DELAY);
            request_ = Request{};
            pending_count_ = 0;
            xSemaphoreGive(lock_);
            return ErrorJson("sequence_start_failed", esp_err_to_name(motion_err));
        }
        SetFaceForAction(first.action);
    }
    cJSON *root = SequenceAcceptedJson(sequence);
    cJSON_AddBoolToObject(root, "queued", already_active);
    cJSON_AddNumberToObject(root, "queue_depth", queue_depth);
    return root;
}

bool SpippyController::EnqueueActionLocked(const QueuedAction &action) {
    if (pending_count_ >= kMaxPendingActions) {
        return false;
    }
    pending_actions_[pending_count_++] = action;
    return true;
}

cJSON *SpippyController::StatusJson() {
    RecursiveLockGuard runtime(runtime_lock_);
    cJSON *root = OkJson("status");
    cJSON_AddStringToObject(root, "speed", speed_ == SPIPPY_SPEED_LEVEL_LOW ? "low" : speed_ == SPIPPY_SPEED_LEVEL_HIGH ? "high" : "medium");
    cJSON_AddBoolToObject(root, "forward_blocked", IsForwardBlocked());
    cJSON_AddBoolToObject(root, "forward_guard_enabled", CONFIG_SPIPPY_PROXIMITY_ENABLE);
    cJSON_AddStringToObject(root, "forward_guard_state", spippy_forward_guard_state_name(forward_guard_.state));
    cJSON_AddStringToObject(root, "forward_guard_reason", spippy_forward_guard_reason_name(forward_guard_.reason));
    cJSON_AddStringToObject(root, "proximity_zone", ZoneName(proximity_zone_));
    cJSON_AddItemToObject(root, "battery", BatteryJson());
    cJSON_AddItemToObject(root, "distance", DistanceJson());
    spippy_buzzer_status_t buzzer = {};
    if (buzzer_manager_get_status(&buzzer) == ESP_OK) {
        cJSON *buzzer_json = cJSON_CreateObject();
        cJSON_AddBoolToObject(buzzer_json, "ready", buzzer.ready);
        cJSON_AddBoolToObject(buzzer_json, "enabled", buzzer.enabled);
        cJSON_AddBoolToObject(buzzer_json, "busy", buzzer.busy);
        cJSON_AddStringToObject(buzzer_json, "last_pattern", buzzer.last_pattern);
        cJSON_AddStringToObject(buzzer_json, "last_error", buzzer.last_error);
        cJSON_AddItemToObject(root, "buzzer", buzzer_json);
    }
    cJSON_AddBoolToObject(root, "autonomy_enabled", autonomy_enabled_);
    cJSON_AddStringToObject(root, "last_autonomy_event", last_autonomy_event_.c_str());
    xSemaphoreTake(lock_, portMAX_DELAY);
    cJSON_AddStringToObject(root, "active_action", request_.active ? ActionName(request_.action) : "idle");
    cJSON_AddStringToObject(root, "completion_mode",
                            request_.active ? CompletionModeName(request_.completion_mode) : "none");
    cJSON_AddNumberToObject(root, "repeat_remaining", request_.repeat_remaining);
    int64_t remaining_ms = 0;
    if (request_.active && request_.deadline_us > 0) {
        remaining_ms = std::max<int64_t>(0, (request_.deadline_us - esp_timer_get_time() + 999) / 1000);
    }
    cJSON_AddNumberToObject(root, "deadline_remaining_ms", remaining_ms);
    cJSON_AddNumberToObject(root, "pending_count", pending_count_);
    cJSON_AddNumberToObject(root, "queue_capacity", kMaxPendingActions);
    xSemaphoreGive(lock_);
    return root;
}

cJSON *SpippyController::BatteryJson() {
    RecursiveLockGuard runtime(runtime_lock_);
    power_manager_get_telemetry(&power_);
    cJSON *root = OkJson("battery");
    AddPowerTelemetry(root, power_);
    int mv = power_.filtered_battery_voltage_mv > 0 ? power_.filtered_battery_voltage_mv : power_.battery_voltage_mv;
    cJSON_AddNumberToObject(root, "level", BatteryPercentFromVoltage(mv));
    cJSON_AddBoolToObject(root, "charging", false);
    cJSON_AddBoolToObject(root, "discharging", true);
    cJSON_AddBoolToObject(root, "conversation_reminder_pending",
                          low_power_reminder_.conversation_reminder_pending);
    cJSON_AddBoolToObject(root, "conversation_reminded",
                          low_power_reminder_.conversation_reminded);
    cJSON_AddNumberToObject(root, "conversation_reminder_cooldown_ms",
                            low_power_reminder_.conversation_cooldown_ms);
    return root;
}

cJSON *SpippyController::DistanceJson() {
    RecursiveLockGuard runtime(runtime_lock_);
    sensor_manager_get_ultrasonic_reading(&distance_);
    cJSON *root = OkJson("distance");
    cJSON_AddBoolToObject(root, "valid", distance_.measurement_valid);
    cJSON_AddNumberToObject(root, "distance_mm", distance_.distance_mm);
    cJSON_AddNumberToObject(root, "filtered_distance_mm", proximity_filtered_distance_mm_);
    cJSON_AddStringToObject(root, "zone", ZoneName(proximity_zone_));
    cJSON_AddStringToObject(root, "previous_zone", ZoneName(proximity_previous_zone_));
    cJSON_AddStringToObject(root, "last_event", ProximityEventName(proximity_last_event_));
    cJSON_AddNumberToObject(root, "event_count", proximity_event_count_);
    cJSON_AddNumberToObject(root, "stable_sample_count", proximity_stable_sample_count_);
    cJSON_AddNumberToObject(root, "invalid_streak", proximity_invalid_streak_);
    cJSON_AddBoolToObject(root, "forward_blocked", IsForwardBlocked());
    cJSON_AddBoolToObject(root, "forward_guard_enabled", CONFIG_SPIPPY_PROXIMITY_ENABLE);
    cJSON_AddStringToObject(root, "forward_guard_state", spippy_forward_guard_state_name(forward_guard_.state));
    cJSON_AddStringToObject(root, "forward_guard_reason", spippy_forward_guard_reason_name(forward_guard_.reason));
    cJSON_AddNumberToObject(root, "forward_guard_clear_streak", forward_guard_.clear_streak);
    cJSON_AddNumberToObject(root, "forward_guard_no_echo_streak", forward_guard_.no_echo_streak);
    cJSON_AddNumberToObject(root, "forward_guard_error_streak", forward_guard_.error_streak);
    int64_t guard_sample_age_ms = forward_guard_.has_sample
        ? static_cast<int64_t>(static_cast<uint32_t>(esp_timer_get_time() / 1000) - forward_guard_.last_sample_ms)
        : -1;
    cJSON_AddNumberToObject(root, "forward_guard_sample_age_ms", guard_sample_age_ms);
    cJSON_AddNumberToObject(root, "sample_count", distance_.sample_count);
    cJSON_AddNumberToObject(root, "timeout_count", distance_.timeout_count);
    cJSON_AddStringToObject(root, "last_error", distance_.last_error);
    return root;
}

cJSON *SpippyController::CalibrationStatusJson() {
    RecursiveLockGuard runtime(runtime_lock_);
    spippy_calibration_status_t status = {};
    spippy_calibration_channel_snapshot_t channels[SPIPPY_ACTIVE_SERVO_COUNT] = {};
    cJSON *root = OkJson("calibration");
    if (calibration_service_get_status(&status) != ESP_OK ||
        calibration_service_get_channel_snapshots(channels, SPIPPY_ACTIVE_SERVO_COUNT) != ESP_OK) {
        cJSON_Delete(root);
        return ErrorJson("calibration_read_failed", "failed to read calibration status");
    }
    cJSON_AddBoolToObject(root, "preview", status.preview_mode_enabled);
    cJSON_AddBoolToObject(root, "preview_mode_enabled", status.preview_mode_enabled);
    cJSON_AddBoolToObject(root, "preflight_running", status.preflight_running);
    cJSON_AddNumberToObject(root, "pose", status.active_pose);
    cJSON_AddStringToObject(root, "pose_name", PoseName(status.active_pose));
    cJSON_AddBoolToObject(root, "manual_override", status.manual_override_active);
    cJSON_AddBoolToObject(root, "manual_override_active", status.manual_override_active);
    cJSON_AddBoolToObject(root, "persisted", status.servo_status.persisted_calibration_present);
    cJSON_AddBoolToObject(root, "persisted_calibration_present", status.servo_status.persisted_calibration_present);
    cJSON_AddBoolToObject(root, "autonomy_enabled", autonomy_enabled_);
    const char *source = status.servo_status.calibration_source == SPIPPY_CALIBRATION_SOURCE_NVS ? "nvs" : "defaults";
    cJSON_AddStringToObject(root, "source", source);
    cJSON_AddStringToObject(root, "calibration_source", source);
    AddPowerTelemetry(root, status.power);
    cJSON_AddItemToObject(root, "power", PowerTelemetryJson(status.power));

    cJSON *array = cJSON_CreateArray();
    for (size_t i = 0; i < SPIPPY_ACTIVE_SERVO_COUNT; ++i) {
        cJSON *item = cJSON_CreateObject();
        cJSON_AddNumberToObject(item, "index", channels[i].servo_index);
        cJSON_AddNumberToObject(item, "servo_index", channels[i].servo_index);
        cJSON_AddStringToObject(item, "label", channels[i].info.label);
        cJSON_AddNumberToObject(item, "gpio", channels[i].info.gpio_num);
        cJSON_AddNumberToObject(item, "leg_id", channels[i].info.leg_id);
        cJSON_AddNumberToObject(item, "joint_id", channels[i].info.joint_id);
        cJSON_AddBoolToObject(item, "invert", channels[i].calibration.invert);
        cJSON_AddNumberToObject(item, "zero_offset_deg_x10", channels[i].calibration.zero_offset_deg_x10);
        cJSON_AddNumberToObject(item, "min_angle_deg_x10", channels[i].calibration.min_angle_deg_x10);
        cJSON_AddNumberToObject(item, "max_angle_deg_x10", channels[i].calibration.max_angle_deg_x10);
        cJSON_AddNumberToObject(item, "safe_angle_deg_x10", channels[i].calibration.safe_angle_deg_x10);
        cJSON_AddNumberToObject(item, "stand_angle_deg_x10", channels[i].calibration.stand_angle_deg_x10);
        cJSON_AddNumberToObject(item, "zero_offset", channels[i].calibration.zero_offset_deg_x10 / 10.0);
        cJSON_AddNumberToObject(item, "min_angle", channels[i].calibration.min_angle_deg_x10 / 10.0);
        cJSON_AddNumberToObject(item, "max_angle", channels[i].calibration.max_angle_deg_x10 / 10.0);
        cJSON_AddNumberToObject(item, "safe_angle", channels[i].calibration.safe_angle_deg_x10 / 10.0);
        cJSON_AddNumberToObject(item, "stand_angle", channels[i].calibration.stand_angle_deg_x10 / 10.0);
        cJSON_AddNumberToObject(item, "logical_angle_deg", channels[i].telemetry.logical_angle_deg);
        cJSON_AddNumberToObject(item, "servo_angle_deg", channels[i].telemetry.calibrated_servo_angle_deg);
        cJSON_AddItemToArray(array, item);
    }
    cJSON_AddItemToObject(root, "channels", array);
    return root;
}

cJSON *SpippyController::CalibrationPreview(bool enabled) {
    RecursiveLockGuard runtime(runtime_lock_);
    NoteUserActivity();
    if (enabled) {
        bool preflight_running = false;
        esp_err_t state_err = calibration_service_get_preflight_mode(&preflight_running);
        if (state_err != ESP_OK) {
            return ErrorJson("preflight_state_failed", esp_err_to_name(state_err));
        }
        if (preflight_running) {
            return ErrorJson("preflight_active", "stop servo preflight before starting calibration");
        }
        cJSON *stop_result = StartAction(SPIPPY_ACTION_STOP, 1,
                                         SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE);
        cJSON_Delete(stop_result);
    }
    esp_err_t err = calibration_service_set_preview_mode(enabled);
    if (err != ESP_OK) {
        return ErrorJson("calibration_preview_failed", esp_err_to_name(err));
    }
    if (display_ != nullptr) {
        display_->SetRobotState(enabled ? "calibration" : "idle");
    }
    return OkJson(enabled ? "preview enabled" : "preview disabled");
}

cJSON *SpippyController::CalibrationPreflight(bool enabled) {
    RecursiveLockGuard runtime(runtime_lock_);
    NoteUserActivity();
    if (enabled) {
        bool preview_mode_enabled = false;
        esp_err_t state_err = calibration_service_get_preview_mode(&preview_mode_enabled);
        if (state_err != ESP_OK) {
            return ErrorJson("calibration_state_failed", esp_err_to_name(state_err));
        }
        if (preview_mode_enabled) {
            return ErrorJson("calibration_active", "finish or cancel calibration before starting servo preflight");
        }
        cJSON *stop_result = StartAction(SPIPPY_ACTION_STOP, 1,
                                         SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE);
        cJSON_Delete(stop_result);
    }
    esp_err_t err = calibration_service_set_preflight_mode(enabled);
    if (err != ESP_OK) {
        return ErrorJson("preflight_failed", esp_err_to_name(err));
    }
    if (display_ != nullptr) {
        display_->SetRobotState(enabled ? "calibration" : "idle");
    }
    return OkJson(enabled ? "preflight started" : "preflight stopped and standby restored");
}

cJSON *SpippyController::CalibrationServo(int index, int angle_x10) {
    RecursiveLockGuard runtime(runtime_lock_);
    if (index < 0 || index >= SPIPPY_ACTIVE_SERVO_COUNT) {
        return ErrorJson("invalid_servo_index", "servo index is out of range");
    }
    if (angle_x10 < -450 || angle_x10 > 450) {
        return ErrorJson("invalid_servo_angle", "angle_x10 must be between -450 and 450");
    }
    esp_err_t err = calibration_service_preview_servo(static_cast<size_t>(index), angle_x10 / 10.0f);
    if (err != ESP_OK) {
        return ErrorJson("servo_preview_failed", esp_err_to_name(err));
    }
    cJSON *root = OkJson("servo preview updated");
    cJSON_AddNumberToObject(root, "index", index);
    cJSON_AddNumberToObject(root, "angle_x10", angle_x10);
    return root;
}

cJSON *SpippyController::CalibrationServoUpdate(int index, int zero_offset_x10) {
    RecursiveLockGuard runtime(runtime_lock_);
    if (index < 0 || index >= SPIPPY_ACTIVE_SERVO_COUNT) {
        return ErrorJson("invalid_servo_index", "servo index is out of range");
    }
    if (zero_offset_x10 < -450 || zero_offset_x10 > 450) {
        return ErrorJson("invalid_zero_offset", "zero offset must be between -45 and 45 degrees");
    }

    bool preview_mode_enabled = false;
    esp_err_t err = calibration_service_get_preview_mode(&preview_mode_enabled);
    if (err != ESP_OK) {
        return ErrorJson("calibration_state_failed", esp_err_to_name(err));
    }
    if (!preview_mode_enabled) {
        return ErrorJson("calibration_not_started", "start calibration before changing offsets");
    }

    spippy_calibration_channel_snapshot_t channels[SPIPPY_ACTIVE_SERVO_COUNT] = {};
    err = calibration_service_get_channel_snapshots(channels, SPIPPY_ACTIVE_SERVO_COUNT);
    if (err != ESP_OK) {
        return ErrorJson("read_failed", esp_err_to_name(err));
    }

    spippy_servo_calibration_t entry = channels[index].calibration;
    entry.zero_offset_deg_x10 = zero_offset_x10;
    err = calibration_service_update_servo_calibration(static_cast<size_t>(index), &entry);
    if (err != ESP_OK) {
        return ErrorJson("update_failed", esp_err_to_name(err));
    }

    cJSON *root = OkJson("servo calibration updated");
    cJSON_AddNumberToObject(root, "index", index);
    cJSON_AddNumberToObject(root, "servo_index", index);
    cJSON_AddNumberToObject(root, "zero_offset_deg_x10", zero_offset_x10);
    cJSON_AddNumberToObject(root, "zero_offset", zero_offset_x10 / 10.0);
    return root;
}

cJSON *SpippyController::CalibrationPose(const std::string &pose_text) {
    RecursiveLockGuard runtime(runtime_lock_);
    spippy_pose_mode_t pose = SPIPPY_POSE_SAFE;
    if (!ResolvePose(pose_text, pose)) {
        return ErrorJson("unsupported_pose", "pose must be safe, stand, sit or crouch");
    }
    esp_err_t err = calibration_service_set_pose(pose);
    if (err != ESP_OK) {
        return ErrorJson("pose_failed", esp_err_to_name(err));
    }
    return OkJson("pose updated");
}

cJSON *SpippyController::CalibrationSave() {
    RecursiveLockGuard runtime(runtime_lock_);
    esp_err_t err = calibration_service_save_calibration();
    if (err != ESP_OK) {
        return ErrorJson("save_failed", esp_err_to_name(err));
    }
    return OkJson("calibration saved");
}

cJSON *SpippyController::SetAutonomyEnabled(bool enabled) {
    RecursiveLockGuard runtime(runtime_lock_);
    esp_err_t err = storage_save_u8("spippy", "autonomy", enabled ? 1 : 0);
    if (err != ESP_OK) {
        return ErrorJson("autonomy_save_failed", esp_err_to_name(err));
    }
    autonomy_enabled_ = enabled;
    NoteUserActivity();
    last_autonomy_event_ = enabled ? "enabled" : "disabled";
    cJSON *root = OkJson(enabled ? "autonomy enabled" : "autonomy disabled");
    cJSON_AddBoolToObject(root, "autonomy_enabled", enabled);
    return root;
}

bool SpippyController::GetAutonomyEnabled() const {
    RecursiveLockGuard runtime(runtime_lock_);
    return autonomy_enabled_;
}

bool SpippyController::ResolveAction(const std::string &text, spippy_action_t &action, bool movement_only) const {
    const std::string t = NormalizeToken(text);
    static const std::map<std::string, spippy_action_t> actions = {
        {"idle", SPIPPY_ACTION_IDLE}, {"standby", SPIPPY_ACTION_IDLE}, {"stand", SPIPPY_ACTION_IDLE},
        {"forward", SPIPPY_ACTION_FORWARD}, {"前进", SPIPPY_ACTION_FORWARD},
        {"backward", SPIPPY_ACTION_BACKWARD}, {"back", SPIPPY_ACTION_BACKWARD}, {"后退", SPIPPY_ACTION_BACKWARD},
        {"turnleft", SPIPPY_ACTION_TURN_LEFT}, {"left", SPIPPY_ACTION_TURN_LEFT}, {"左转", SPIPPY_ACTION_TURN_LEFT},
        {"turnright", SPIPPY_ACTION_TURN_RIGHT}, {"right", SPIPPY_ACTION_TURN_RIGHT}, {"右转", SPIPPY_ACTION_TURN_RIGHT},
        {"stop", SPIPPY_ACTION_STOP}, {"停止", SPIPPY_ACTION_STOP},
        {"wave", SPIPPY_ACTION_WAVE}, {"挥手", SPIPPY_ACTION_WAVE},
        {"headsway", SPIPPY_ACTION_HEAD_SWAY}, {"摇头", SPIPPY_ACTION_HEAD_SWAY},
        {"shake", SPIPPY_ACTION_SHAKE}, {"抖一抖", SPIPPY_ACTION_SHAKE},
        {"bow", SPIPPY_ACTION_BOW}, {"鞠躬", SPIPPY_ACTION_BOW},
        {"congrats", SPIPPY_ACTION_CONGRATS}, {"恭喜", SPIPPY_ACTION_CONGRATS},
        {"sit", SPIPPY_ACTION_SIT}, {"坐下", SPIPPY_ACTION_SIT},
        {"pushup", SPIPPY_ACTION_PUSHUP}, {"俯卧撑", SPIPPY_ACTION_PUSHUP},
        {"coquetry", SPIPPY_ACTION_COQUETRY}, {"cute", SPIPPY_ACTION_COQUETRY}, {"卖萌", SPIPPY_ACTION_COQUETRY},
        {"frontbacksway", SPIPPY_ACTION_FRONT_BACK_SWAY}, {"rock", SPIPPY_ACTION_FRONT_BACK_SWAY},
        {"dance1", SPIPPY_ACTION_DANCE_1}, {"dance", SPIPPY_ACTION_DANCE_1}, {"跳舞", SPIPPY_ACTION_DANCE_1},
        {"dance2", SPIPPY_ACTION_DANCE_2}, {"sing", SPIPPY_ACTION_DANCE_2}, {"唱歌", SPIPPY_ACTION_DANCE_2},
        {"bounce", SPIPPY_ACTION_BOUNCE}, {"蹦一蹦", SPIPPY_ACTION_BOUNCE},
        {"pose1", SPIPPY_ACTION_POSE_1}, {"pose", SPIPPY_ACTION_POSE_1}, {"摆pose", SPIPPY_ACTION_POSE_1},
        {"idlebreathe", SPIPPY_ACTION_IDLE_BREATHE}, {"呼吸", SPIPPY_ACTION_IDLE_BREATHE},
        {"nod", SPIPPY_ACTION_NOD}, {"点头", SPIPPY_ACTION_NOD},
        {"cutewiggle", SPIPPY_ACTION_CUTE_WIGGLE}, {"撒娇", SPIPPY_ACTION_CUTE_WIGGLE},
        {"startle", SPIPPY_ACTION_STARTLE}, {"吓一跳", SPIPPY_ACTION_STARTLE},
        {"confused", SPIPPY_ACTION_CONFUSED}, {"困惑", SPIPPY_ACTION_CONFUSED},
        {"afraid", SPIPPY_ACTION_AFRAID}, {"害怕", SPIPPY_ACTION_AFRAID},
        {"angry", SPIPPY_ACTION_ANGRY}, {"生气", SPIPPY_ACTION_ANGRY},
        {"sleepy", SPIPPY_ACTION_SLEEPY}, {"困困", SPIPPY_ACTION_SLEEPY},
        {"voiceperform", SPIPPY_ACTION_VOICE_PERFORM}, {"kungfu", SPIPPY_ACTION_VOICE_PERFORM}, {"武术", SPIPPY_ACTION_VOICE_PERFORM},
        {"peerightrear", SPIPPY_ACTION_PEE_RIGHT_REAR}, {"pee", SPIPPY_ACTION_PEE_RIGHT_REAR}, {"撒尿", SPIPPY_ACTION_PEE_RIGHT_REAR},
        {"prone", SPIPPY_ACTION_PRONE}, {"趴下", SPIPPY_ACTION_PRONE},
    };
    auto it = actions.find(t);
    if (it == actions.end()) {
        return false;
    }
    bool movement = it->second == SPIPPY_ACTION_FORWARD || it->second == SPIPPY_ACTION_BACKWARD ||
                    it->second == SPIPPY_ACTION_TURN_LEFT || it->second == SPIPPY_ACTION_TURN_RIGHT ||
                    it->second == SPIPPY_ACTION_STOP || it->second == SPIPPY_ACTION_IDLE;
    /* move 与 perform 的动作集合必须互斥，避免错误工具调用绕过移动的安全语义。 */
    if ((movement_only && !movement) || (!movement_only && movement)) {
        return false;
    }
    action = it->second;
    return true;
}

bool SpippyController::ResolveMovementMode(const std::string &text, CompletionMode &mode) const {
    const std::string token = NormalizeToken(text);
    if (token == "steps" || token == "step" || token == "cycles" || token == "步数") {
        mode = CompletionMode::kCycles;
        return true;
    }
    if (token == "duration" || token == "time" || token == "定时") {
        mode = CompletionMode::kDeadline;
        return true;
    }
    if (token == "continuous" || token == "continue" || token == "一直" || token == "持续") {
        mode = CompletionMode::kExplicitStop;
        return true;
    }
    return false;
}

bool SpippyController::ResolveSpeed(const std::string &text, spippy_speed_level_t &speed) const {
    std::string t = NormalizeToken(text);
    if (t == "low" || t == "slow" || t == "1" || t == "慢" || t == "慢一点") {
        speed = SPIPPY_SPEED_LEVEL_LOW;
        return true;
    }
    if (t == "medium" || t == "mid" || t == "normal" || t == "2" || t == "正常" || t == "中速") {
        speed = SPIPPY_SPEED_LEVEL_MID;
        return true;
    }
    if (t == "high" || t == "fast" || t == "3" || t == "快" || t == "快一点") {
        speed = SPIPPY_SPEED_LEVEL_HIGH;
        return true;
    }
    return false;
}

bool SpippyController::ResolvePose(const std::string &text, spippy_pose_mode_t &pose) const {
    std::string t = NormalizeToken(text);
    if (t == "safe" || t == "安全") {
        pose = SPIPPY_POSE_SAFE;
        return true;
    }
    if (t == "stand" || t == "站立") {
        pose = SPIPPY_POSE_STAND;
        return true;
    }
    if (t == "sit" || t == "坐下") {
        pose = SPIPPY_POSE_SIT;
        return true;
    }
    if (t == "crouch" || t == "趴低") {
        pose = SPIPPY_POSE_CROUCH;
        return true;
    }
    return false;
}

const char *SpippyController::ActionName(spippy_action_t action) const {
    switch (action) {
        case SPIPPY_ACTION_FORWARD: return "forward";
        case SPIPPY_ACTION_BACKWARD: return "backward";
        case SPIPPY_ACTION_TURN_LEFT: return "turn_left";
        case SPIPPY_ACTION_TURN_RIGHT: return "turn_right";
        case SPIPPY_ACTION_STOP: return "stop";
        case SPIPPY_ACTION_WAVE: return "wave";
        case SPIPPY_ACTION_HEAD_SWAY: return "head_sway";
        case SPIPPY_ACTION_SHAKE: return "shake";
        case SPIPPY_ACTION_BOW: return "bow";
        case SPIPPY_ACTION_CONGRATS: return "congrats";
        case SPIPPY_ACTION_SIT: return "sit";
        case SPIPPY_ACTION_PUSHUP: return "pushup";
        case SPIPPY_ACTION_COQUETRY: return "coquetry";
        case SPIPPY_ACTION_FRONT_BACK_SWAY: return "front_back_sway";
        case SPIPPY_ACTION_DANCE_1: return "dance_1";
        case SPIPPY_ACTION_DANCE_2: return "dance_2";
        case SPIPPY_ACTION_BOUNCE: return "bounce";
        case SPIPPY_ACTION_POSE_1: return "pose_1";
        case SPIPPY_ACTION_IDLE_BREATHE: return "idle_breathe";
        case SPIPPY_ACTION_NOD: return "nod";
        case SPIPPY_ACTION_CUTE_WIGGLE: return "cute_wiggle";
        case SPIPPY_ACTION_STARTLE: return "startle";
        case SPIPPY_ACTION_CONFUSED: return "confused";
        case SPIPPY_ACTION_AFRAID: return "afraid";
        case SPIPPY_ACTION_ANGRY: return "angry";
        case SPIPPY_ACTION_SLEEPY: return "sleepy";
        case SPIPPY_ACTION_VOICE_PERFORM: return "voice_perform";
        case SPIPPY_ACTION_PEE_RIGHT_REAR: return "pee_right_rear";
        case SPIPPY_ACTION_PRONE: return "prone";
        default: return "idle";
    }
}

const char *SpippyController::CompletionModeName(CompletionMode mode) const {
    switch (mode) {
        case CompletionMode::kCycles: return "cycles";
        case CompletionMode::kDeadline: return "duration";
        case CompletionMode::kExplicitStop: return "continuous";
        default: return "unknown";
    }
}

bool SpippyController::IsLocomotionAction(spippy_action_t action) const {
    return action == SPIPPY_ACTION_FORWARD || action == SPIPPY_ACTION_BACKWARD ||
           action == SPIPPY_ACTION_TURN_LEFT || action == SPIPPY_ACTION_TURN_RIGHT;
}

const char *SpippyController::ZoneName(spippy_proximity_zone_t zone) const {
    switch (zone) {
        case SPIPPY_PROXIMITY_ZONE_FAR: return "far";
        case SPIPPY_PROXIMITY_ZONE_NEAR: return "near";
        case SPIPPY_PROXIMITY_ZONE_CLOSE: return "close";
        case SPIPPY_PROXIMITY_ZONE_TOO_CLOSE: return "too_close";
        default: return "lost";
    }
}

void SpippyController::TaskEntry(void *arg) {
    static_cast<SpippyController *>(arg)->TaskLoop();
}

void SpippyController::TaskLoop() {
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t control_error_streak = 0;
    while (true) {
        {
            RecursiveLockGuard runtime(runtime_lock_);
            esp_err_t motion_err = motion_engine_tick();
            esp_err_t calibration_err = calibration_service_tick();
            spippy_leg_frame_t frame = {};
            esp_err_t servo_err = motion_err == ESP_OK ? calibration_err : motion_err;
            if (servo_err == ESP_OK) {
                servo_err = motion_engine_get_latest_frame(&frame);
            }
            if (servo_err == ESP_OK) {
                servo_err = servo_manager_apply_frame(&frame);
            }
            if (servo_err == ESP_OK) {
                servo_err = servo_manager_tick();
            }
            if (motion_err != ESP_OK || calibration_err != ESP_OK || servo_err != ESP_OK) {
                control_error_streak++;
                if (control_error_streak == 1 || (control_error_streak % 50) == 0) {
                    ESP_LOGE(TAG, "control tick failed: motion=%s calibration=%s servo=%s streak=%" PRIu32,
                             esp_err_to_name(motion_err),
                             esp_err_to_name(calibration_err),
                             esp_err_to_name(servo_err),
                             control_error_streak);
                }
            } else {
                control_error_streak = 0;
            }

            if (++power_ticks_ >= PeriodTicks(CONFIG_SPIPPY_TASK_POWER_PERIOD_MS)) {
                power_ticks_ = 0;
                UpdatePower();
            }
            if (++sensor_ticks_ >= PeriodTicks(CONFIG_SPIPPY_TASK_SENSOR_PERIOD_MS)) {
                sensor_ticks_ = 0;
                UpdateDistance();
                UpdateForwardGuard();
                UpdateProximity();
            }
#if CONFIG_SPIPPY_AUTONOMY_ENABLE
            if (++autonomy_ticks_ >= PeriodTicks(CONFIG_SPIPPY_TASK_AUTONOMY_PERIOD_MS)) {
                autonomy_ticks_ = 0;
                UpdateAutonomy();
            }
#endif
            HandleActionCycle();
        }
        vTaskDelayUntil(&last_wake, DelayTicks(CONFIG_SPIPPY_TASK_MOTION_PERIOD_MS));
    }
}

const char *SpippyController::ProximityEventName(spippy_proximity_event_t event) const {
    switch (event) {
        case SPIPPY_PROXIMITY_EVENT_ENTER_NEAR: return "enter_near";
        case SPIPPY_PROXIMITY_EVENT_ENTER_CLOSE: return "enter_close";
        case SPIPPY_PROXIMITY_EVENT_ENTER_TOO_CLOSE: return "enter_too_close";
        case SPIPPY_PROXIMITY_EVENT_SUDDEN_APPROACH: return "sudden_approach";
        case SPIPPY_PROXIMITY_EVENT_HOLD_CLOSE: return "hold_close";
        case SPIPPY_PROXIMITY_EVENT_LEAVE: return "leave";
        default: return "none";
    }
}

void SpippyController::NoteUserActivity() {
    int64_t now = esp_timer_get_time();
    last_user_activity_us_ = now;
    ScheduleNextAutonomy(now);
}

void SpippyController::ScheduleNextAutonomy(int64_t base_us) {
    next_autonomy_due_us_ = base_us + RandomAutonomyIntervalUs();
}

int64_t SpippyController::RandomAutonomyIntervalUs() const {
    uint32_t min_s = CONFIG_SPIPPY_AUTONOMY_MIN_INTERVAL_S;
    uint32_t max_s = CONFIG_SPIPPY_AUTONOMY_MAX_INTERVAL_S;
    if (max_s < min_s) {
        max_s = min_s;
    }
    uint32_t span = max_s - min_s;
    uint32_t offset = span == 0 ? 0 : (esp_random() % (span + 1));
    uint32_t interval_s = min_s + offset;
    // Kconfig 最大允许 7200 秒，必须先提升到 64 位再换算，否则约 71 分钟处溢出。
    return static_cast<int64_t>(interval_s) * 1000000;
}

bool SpippyController::RuntimeAllowsIdleInteraction() const {
    if (power_.low_power_latched || ServoMaintenanceActive()) {
        return false;
    }
    xSemaphoreTake(lock_, portMAX_DELAY);
    bool idle = !request_.active && pending_count_ == 0;
    xSemaphoreGive(lock_);
    return idle;
}

spippy_proximity_zone_t SpippyController::ClassifyProximityDistance(uint32_t distance_mm) const {
    if (distance_mm <= CONFIG_SPIPPY_PROXIMITY_TOO_CLOSE_ENTER_MM) {
        return SPIPPY_PROXIMITY_ZONE_TOO_CLOSE;
    }
    if (proximity_zone_ == SPIPPY_PROXIMITY_ZONE_TOO_CLOSE &&
        distance_mm <= CONFIG_SPIPPY_PROXIMITY_TOO_CLOSE_EXIT_MM) {
        return SPIPPY_PROXIMITY_ZONE_TOO_CLOSE;
    }
    if (distance_mm <= CONFIG_SPIPPY_PROXIMITY_CLOSE_ENTER_MM) {
        return SPIPPY_PROXIMITY_ZONE_CLOSE;
    }
    if (proximity_zone_ == SPIPPY_PROXIMITY_ZONE_CLOSE &&
        distance_mm <= CONFIG_SPIPPY_PROXIMITY_CLOSE_EXIT_MM) {
        return SPIPPY_PROXIMITY_ZONE_CLOSE;
    }
    if (distance_mm <= CONFIG_SPIPPY_PROXIMITY_NEAR_ENTER_MM) {
        return SPIPPY_PROXIMITY_ZONE_NEAR;
    }
    if (proximity_zone_ == SPIPPY_PROXIMITY_ZONE_NEAR &&
        distance_mm <= CONFIG_SPIPPY_PROXIMITY_NEAR_EXIT_MM) {
        return SPIPPY_PROXIMITY_ZONE_NEAR;
    }
    return SPIPPY_PROXIMITY_ZONE_FAR;
}

bool SpippyController::ConfirmProximityZone(spippy_proximity_zone_t candidate) {
    if (candidate == proximity_zone_) {
        proximity_candidate_zone_ = candidate;
        if (proximity_stable_sample_count_ < UINT32_MAX) {
            proximity_stable_sample_count_++;
        }
        proximity_candidate_count_ = 0;
        return false;
    }
    if (candidate == proximity_candidate_zone_) {
        proximity_candidate_count_++;
    } else {
        proximity_candidate_zone_ = candidate;
        proximity_candidate_count_ = 1;
    }
    if (proximity_candidate_count_ < CONFIG_SPIPPY_PROXIMITY_STABLE_CONFIRM_COUNT) {
        proximity_stable_sample_count_ = proximity_candidate_count_;
        return false;
    }
    proximity_previous_zone_ = proximity_zone_;
    proximity_zone_ = candidate;
    proximity_stable_sample_count_ = proximity_candidate_count_;
    proximity_candidate_count_ = 0;
    return true;
}

spippy_proximity_event_t SpippyController::ProximityEventForZoneChange() const {
    if (proximity_zone_ == SPIPPY_PROXIMITY_ZONE_TOO_CLOSE) {
        return SPIPPY_PROXIMITY_EVENT_ENTER_TOO_CLOSE;
    }
    if (proximity_zone_ == SPIPPY_PROXIMITY_ZONE_CLOSE) {
        return SPIPPY_PROXIMITY_EVENT_ENTER_CLOSE;
    }
    if (proximity_zone_ == SPIPPY_PROXIMITY_ZONE_NEAR) {
        return SPIPPY_PROXIMITY_EVENT_ENTER_NEAR;
    }
    bool was_present = proximity_previous_zone_ == SPIPPY_PROXIMITY_ZONE_NEAR ||
                       proximity_previous_zone_ == SPIPPY_PROXIMITY_ZONE_CLOSE ||
                       proximity_previous_zone_ == SPIPPY_PROXIMITY_ZONE_TOO_CLOSE;
    if ((proximity_zone_ == SPIPPY_PROXIMITY_ZONE_FAR || proximity_zone_ == SPIPPY_PROXIMITY_ZONE_LOST) &&
        was_present) {
        return SPIPPY_PROXIMITY_EVENT_LEAVE;
    }
    return SPIPPY_PROXIMITY_EVENT_NONE;
}

void SpippyController::RecordProximityEvent(spippy_proximity_event_t event) {
    if (event == SPIPPY_PROXIMITY_EVENT_NONE) {
        return;
    }
    proximity_last_event_ = event;
    proximity_event_count_++;
}

void SpippyController::UpdatePower() {
    esp_err_t err = power_manager_tick();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "battery sampling failed: %s", esp_err_to_name(err));
        return;
    }
    if (power_manager_get_telemetry(&power_) != ESP_OK) {
        return;
    }
    if (display_ != nullptr) {
        int mv = power_.filtered_battery_voltage_mv > 0 ? power_.filtered_battery_voltage_mv : power_.battery_voltage_mv;
        display_->SetBatteryLevel(BatteryPercentFromVoltage(mv), power_.low_power_latched);
    }
    const bool reminder_state_changed =
        spippy_low_power_reminder_set_active(&low_power_reminder_, power_.low_power_latched);
    if (reminder_state_changed) {
        ESP_LOGI(TAG, "low-power reminder episode %s",
                 power_.low_power_latched ? "started" : "cleared");
        if (power_.low_power_latched) {
            esp_err_t buzzer_err = buzzer_manager_set_low_power_alert(true);
            if (buzzer_err != ESP_OK) {
                ESP_LOGW(TAG, "failed to arm low-power buzzer: %s", esp_err_to_name(buzzer_err));
            }
        }
    }
    if (power_.low_power_latched) {
        if (power_.recovery_ready && power_manager_clear_low_power_latch(false) == ESP_OK) {
            power_manager_get_telemetry(&power_);
            spippy_low_power_reminder_set_active(&low_power_reminder_, false);
            buzzer_manager_set_low_power_alert(false);
            if (display_ != nullptr) {
                int mv = power_.filtered_battery_voltage_mv > 0 ? power_.filtered_battery_voltage_mv : power_.battery_voltage_mv;
                display_->SetBatteryLevel(BatteryPercentFromVoltage(mv), power_.low_power_latched);
                display_->ClearProtectedSystemFace();
                display_->SetRobotState("idle");
            }
            return;
        }
        HandleLowPower();
    } else {
        spippy_low_power_reminder_set_active(&low_power_reminder_, false);
        buzzer_manager_set_low_power_alert(false);
    }
}

void SpippyController::UpdateDistance() {
    sensor_manager_get_ultrasonic_reading(&distance_);
}

void SpippyController::UpdateForwardGuard() {
#if CONFIG_SPIPPY_PROXIMITY_ENABLE
    const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    bool changed = false;
    if (distance_.sample_count != 0) {
        spippy_forward_guard_sample_kind_t kind = SPIPPY_FORWARD_GUARD_SAMPLE_ERROR;
        if (distance_.measurement_valid) {
            kind = SPIPPY_FORWARD_GUARD_SAMPLE_VALID;
        } else if (strcmp(distance_.last_error, "timeout") == 0) {
            kind = SPIPPY_FORWARD_GUARD_SAMPLE_NO_ECHO;
        }
        changed = spippy_forward_guard_update(&forward_guard_, kind, distance_.distance_mm,
                                              distance_.sample_count, now_ms);
    }
    changed = spippy_forward_guard_tick(&forward_guard_, now_ms) || changed;
    if (changed) {
        ESP_LOGI(TAG, "forward guard state=%s reason=%s distance=%" PRIu32
                      "mm no_echo=%" PRIu32 " errors=%" PRIu32,
                 spippy_forward_guard_state_name(forward_guard_.state),
                 spippy_forward_guard_reason_name(forward_guard_.reason),
                 forward_guard_.last_distance_mm,
                 forward_guard_.no_echo_streak,
                 forward_guard_.error_streak);
    }

    if (spippy_forward_guard_can_move(&forward_guard_)) {
        return;
    }
    xSemaphoreTake(lock_, portMAX_DELAY);
    const bool moving_forward = request_.active && request_.action == SPIPPY_ACTION_FORWARD;
    xSemaphoreGive(lock_);
    if (moving_forward) {
        ESP_LOGW(TAG, "stopping forward motion: guard state=%s reason=%s",
                 spippy_forward_guard_state_name(forward_guard_.state),
                 spippy_forward_guard_reason_name(forward_guard_.reason));
        cJSON *stop_result = StartAction(SPIPPY_ACTION_STOP, 1,
                                         SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE);
        cJSON_Delete(stop_result);
    }
#endif
}

void SpippyController::UpdateProximity() {
#if CONFIG_SPIPPY_PROXIMITY_ENABLE
    if (!distance_.measurement_valid || distance_.distance_mm < 50) {
        proximity_invalid_streak_++;
        proximity_candidate_count_ = 0;
        if (proximity_invalid_streak_ >= CONFIG_SPIPPY_PROXIMITY_LOST_CONFIRM_COUNT &&
            proximity_zone_ != SPIPPY_PROXIMITY_ZONE_LOST) {
            proximity_previous_zone_ = proximity_zone_;
            proximity_zone_ = SPIPPY_PROXIMITY_ZONE_LOST;
            proximity_stable_sample_count_ = proximity_invalid_streak_;
            proximity_sample_count_ = 0;
            proximity_sample_next_ = 0;
            proximity_previous_filtered_valid_ = false;
            proximity_previous_raw_valid_ = false;
            proximity_close_since_us_ = 0;
            proximity_hold_close_emitted_ = false;
            proximity_sudden_approach_emitted_ = false;
            spippy_proximity_event_t event = ProximityEventForZoneChange();
            RecordProximityEvent(event);
            TriggerProximityInteraction(event);
        }
        return;
    }

    proximity_invalid_streak_ = 0;
    proximity_samples_[proximity_sample_next_] = distance_.distance_mm;
    proximity_sample_next_ = (proximity_sample_next_ + 1) % kProximitySampleWindow;
    if (proximity_sample_count_ < kProximitySampleWindow) {
        proximity_sample_count_++;
    }

    std::array<uint32_t, kProximitySampleWindow> sorted = proximity_samples_;
    for (uint32_t i = 1; i < proximity_sample_count_; ++i) {
        uint32_t value = sorted[i];
        uint32_t j = i;
        while (j > 0 && sorted[j - 1] > value) {
            sorted[j] = sorted[j - 1];
            j--;
        }
        sorted[j] = value;
    }
    uint32_t median = sorted[proximity_sample_count_ / 2];
    if (!proximity_previous_filtered_valid_) {
        proximity_filtered_distance_mm_ = median;
    } else {
        proximity_filtered_distance_mm_ = ((proximity_filtered_distance_mm_ * 65) + (median * 35)) / 100;
    }

    bool sudden_approach = proximity_previous_raw_valid_ &&
        proximity_previous_raw_distance_mm_ > distance_.distance_mm + 250 &&
        distance_.distance_mm <= CONFIG_SPIPPY_PROXIMITY_CLOSE_EXIT_MM;
    proximity_previous_filtered_valid_ = true;
    proximity_previous_raw_distance_mm_ = distance_.distance_mm;
    proximity_previous_raw_valid_ = true;

    spippy_proximity_event_t event = SPIPPY_PROXIMITY_EVENT_NONE;
    if (ConfirmProximityZone(ClassifyProximityDistance(proximity_filtered_distance_mm_))) {
        if (proximity_zone_ == SPIPPY_PROXIMITY_ZONE_CLOSE) {
            proximity_close_since_us_ = esp_timer_get_time();
            proximity_hold_close_emitted_ = false;
        } else {
            proximity_close_since_us_ = 0;
            proximity_hold_close_emitted_ = false;
        }
        if (proximity_zone_ != SPIPPY_PROXIMITY_ZONE_CLOSE &&
            proximity_zone_ != SPIPPY_PROXIMITY_ZONE_TOO_CLOSE) {
            proximity_sudden_approach_emitted_ = false;
        }
        event = ProximityEventForZoneChange();
    } else if (proximity_zone_ == SPIPPY_PROXIMITY_ZONE_CLOSE &&
               proximity_close_since_us_ != 0 &&
               !proximity_hold_close_emitted_ &&
               esp_timer_get_time() - proximity_close_since_us_ >= MsToUs(3000)) {
        event = SPIPPY_PROXIMITY_EVENT_HOLD_CLOSE;
        proximity_hold_close_emitted_ = true;
    }

    if (sudden_approach &&
        (proximity_zone_ == SPIPPY_PROXIMITY_ZONE_CLOSE || proximity_zone_ == SPIPPY_PROXIMITY_ZONE_TOO_CLOSE) &&
        !proximity_sudden_approach_emitted_) {
        event = SPIPPY_PROXIMITY_EVENT_SUDDEN_APPROACH;
        proximity_sudden_approach_emitted_ = true;
    }

    RecordProximityEvent(event);
    TriggerProximityInteraction(event);
#endif
}

void SpippyController::HandleLowPower() {
    xSemaphoreTake(lock_, portMAX_DELAY);
    request_ = Request{};
    pending_count_ = 0;
    xSemaphoreGive(lock_);
    calibration_service_abort_preview();
    motion_engine_set_action(SPIPPY_ACTION_IDLE);
    motion_engine_set_pose_mode(SPIPPY_POSE_SAFE);
    if (display_ != nullptr) {
        display_->SetRobotState("low_power");
    }
    const DeviceState app_state = Application::GetInstance().GetDeviceState();
    const bool conversation_audio_active =
        app_state == kDeviceStateListening || app_state == kDeviceStateSpeaking;
    /* 对话中不抢播；保留 pending，待 tts stop 时追加到在线回答末尾。 */
    if (!conversation_audio_active &&
        spippy_low_power_reminder_take_entry(&low_power_reminder_)) {
        ScheduleLowPowerVoiceReminder("entry_idle", false);
    }
}

void SpippyController::HandleActionCycle() {
    bool completed = false;
    if (motion_engine_consume_cycle_complete(&completed) != ESP_OK || !completed) {
        return;
    }
    xSemaphoreTake(lock_, portMAX_DELAY);
    if (!request_.active) {
        xSemaphoreGive(lock_);
        return;
    }

    bool continue_current = false;
    bool continuous_safety_timeout = false;
    switch (request_.completion_mode) {
        case CompletionMode::kCycles:
            if (request_.repeat_remaining > 1) {
                request_.repeat_remaining--;
                continue_current = true;
            }
            break;
        case CompletionMode::kDeadline:
            continue_current = esp_timer_get_time() < request_.deadline_us;
            break;
        case CompletionMode::kExplicitStop:
            continue_current = esp_timer_get_time() < request_.deadline_us;
            continuous_safety_timeout = !continue_current;
            break;
    }
    if (continue_current) {
        /*
         * motion_engine 已在周期结束时把相位自然回绕到 0；这里只更新计数，
         * 不能再次 set_action，否则每一步都会重新插入 240 ms 起步过渡。
         */
        xSemaphoreGive(lock_);
        return;
    }
    spippy_action_completion_policy_t policy = request_.completion_policy;
    while (pending_count_ > 0) {
        QueuedAction next = pending_actions_[0];
        for (size_t i = 1; i < pending_count_; ++i) {
            pending_actions_[i - 1] = pending_actions_[i];
        }
        pending_count_--;

        /* 入队时的距离检查不能代替执行前检查：排队期间障碍物可能刚好靠近。 */
        if (next.action == SPIPPY_ACTION_FORWARD && IsForwardBlocked()) {
            ESP_LOGW(TAG, "skip queued forward action: guard state=%s reason=%s",
                     spippy_forward_guard_state_name(forward_guard_.state),
                     spippy_forward_guard_reason_name(forward_guard_.reason));
            continue;
        }

        request_.action = next.action;
        request_.repeat_remaining = std::max<uint32_t>(1, std::min<uint32_t>(next.repeat, 12));
        request_.completion_mode = CompletionMode::kCycles;
        request_.deadline_us = 0;
        request_.completion_policy = next.completion_policy;
        request_.active = true;
        spippy_action_t next_action = request_.action;
        xSemaphoreGive(lock_);
        esp_err_t err = motion_engine_set_action(next_action);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "failed to start queued action %s: %s",
                     ActionName(next_action), esp_err_to_name(err));
            xSemaphoreTake(lock_, portMAX_DELAY);
            request_ = Request{};
            pending_count_ = 0;
            xSemaphoreGive(lock_);
            motion_engine_hold_current_frame(SPIPPY_ACTION_IDLE, SPIPPY_POSE_STAND);
            return;
        }
        SetFaceForAction(next_action);
        return;
    }
    request_ = Request{};
    xSemaphoreGive(lock_);
    if (continuous_safety_timeout) {
        ESP_LOGW(TAG, "continuous movement reached %ds safety timeout",
                 CONFIG_SPIPPY_CONTINUOUS_MOVE_MAX_DURATION_S);
    }
    if (policy == SPIPPY_ACTION_COMPLETION_POLICY_HOLD_FINAL_POSE) {
        motion_engine_hold_current_frame(SPIPPY_ACTION_IDLE, SPIPPY_POSE_STAND);
    } else {
        esp_err_t err = motion_engine_set_pose_mode(SPIPPY_POSE_STAND);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "failed to return to stand after action: %s", esp_err_to_name(err));
        }
    }
    if (display_ != nullptr && !power_.low_power_latched) {
        display_->SetRobotState("idle");
    }
}

void SpippyController::PreemptLocomotionForUserCommand() {
    xSemaphoreTake(lock_, portMAX_DELAY);
    const bool preempted = request_.active && IsLocomotionAction(request_.action);
    if (preempted) {
        request_ = Request{};
        pending_count_ = 0;
    }
    xSemaphoreGive(lock_);
    if (preempted) {
        ESP_LOGI(TAG, "user performance command preempted active locomotion");
    }
}

void SpippyController::ScheduleLowPowerVoiceReminder(const char *source, bool append_after_tts) {
    ESP_LOGI(TAG, "scheduling low-power voice reminder source=%s",
             source != nullptr ? source : "unknown");
    Application::GetInstance().Schedule([this, append_after_tts]() {
        auto &app = Application::GetInstance();
        const DeviceState state = app.GetDeviceState();
        const bool conversation_audio_active =
            state == kDeviceStateListening || state == kDeviceStateSpeaking;
        {
            RecursiveLockGuard runtime(runtime_lock_);
            if (!low_power_reminder_.active) {
                return;
            }
            if (conversation_audio_active && !append_after_tts) {
                /* 状态在排队期间发生变化：退回 pending，不能在收音时抢播。 */
                spippy_low_power_reminder_requeue_entry(&low_power_reminder_);
                return;
            }
        }
        app.Alert(
            Lang::Strings::WARNING,
            Lang::Strings::BATTERY_NEED_CHARGE,
            "low_power",
            Lang::Sounds::OGG_LOW_BATTERY);
    });
}

void SpippyController::MarkLowPowerReminderHandledByResponse() {
    RecursiveLockGuard runtime(runtime_lock_);
    spippy_low_power_reminder_mark_explicit_response(
        &low_power_reminder_, static_cast<uint32_t>(esp_timer_get_time() / 1000));
}

cJSON *SpippyController::LowPowerErrorJson() {
    MarkLowPowerReminderHandledByResponse();
    cJSON *root = ErrorJson("low_power_lock", "电量低，暂时不能运动或表演，请先充电");
    cJSON_AddStringToObject(root, "user_message", "电量低，暂时不能运动或表演，请先充电");
    cJSON_AddBoolToObject(root, "must_inform_user", true);
    cJSON_AddBoolToObject(root, "low_power_latched", true);
    const int mv = power_.filtered_battery_voltage_mv > 0
        ? power_.filtered_battery_voltage_mv
        : power_.battery_voltage_mv;
    cJSON_AddNumberToObject(root, "battery_voltage_mv", mv);
    cJSON_AddNumberToObject(root, "level", BatteryPercentFromVoltage(mv));
    return root;
}

cJSON *SpippyController::ForwardGuardErrorJson() const {
#if CONFIG_SPIPPY_PROXIMITY_ENABLE
    if (spippy_forward_guard_can_move(&forward_guard_)) {
        return nullptr;
    }
    const bool obstacle_confirmed = forward_guard_.state == SPIPPY_FORWARD_GUARD_BLOCKED;
    cJSON *root = ErrorJson(
        obstacle_confirmed ? "obstacle_near" : "forward_guard_unknown",
        obstacle_confirmed
            ? "forward movement is blocked because an obstacle is confirmed ahead"
            : "forward movement is blocked because ultrasonic protection is not ready");
    cJSON_AddStringToObject(root, "forward_guard_state",
                            spippy_forward_guard_state_name(forward_guard_.state));
    cJSON_AddStringToObject(root, "forward_guard_reason",
                            spippy_forward_guard_reason_name(forward_guard_.reason));
    cJSON_AddNumberToObject(root, "distance_mm", forward_guard_.last_distance_mm);
    const int64_t sample_age_ms = forward_guard_.has_sample
        ? static_cast<int64_t>(static_cast<uint32_t>(esp_timer_get_time() / 1000) -
                               forward_guard_.last_sample_ms)
        : -1;
    cJSON_AddNumberToObject(root, "sample_age_ms", sample_age_ms);
    return root;
#else
    return nullptr;
#endif
}

bool SpippyController::IsForwardBlocked() const {
    RecursiveLockGuard runtime(runtime_lock_);
#if CONFIG_SPIPPY_PROXIMITY_ENABLE
    return !spippy_forward_guard_can_move(&forward_guard_);
#else
    return false;
#endif
}

void SpippyController::UpdateAutonomy() {
#if CONFIG_SPIPPY_AUTONOMY_ENABLE
    if (!autonomy_enabled_) {
        return;
    }
    int64_t now = esp_timer_get_time();
    if (now >= last_user_activity_us_ + SecToUs(CONFIG_SPIPPY_AUTONOMY_IDLE_TRIGGER_S) &&
        now >= next_autonomy_due_us_) {
        TriggerAutonomyInteraction(false);
    }
#endif
}

void SpippyController::TriggerProximityInteraction(spippy_proximity_event_t event) {
#if CONFIG_SPIPPY_PROXIMITY_ENABLE
    if (!autonomy_enabled_ || event == SPIPPY_PROXIMITY_EVENT_NONE || !RuntimeAllowsIdleInteraction()) {
        return;
    }
    int64_t now = esp_timer_get_time();
    if (now - last_proximity_reaction_us_ < MsToUs(CONFIG_SPIPPY_PROXIMITY_GLOBAL_COOLDOWN_MS)) {
        return;
    }
    uint32_t event_cooldown_ms = UINT32_MAX;
    switch (event) {
        case SPIPPY_PROXIMITY_EVENT_ENTER_NEAR: event_cooldown_ms = 8000; break;
        case SPIPPY_PROXIMITY_EVENT_ENTER_CLOSE: event_cooldown_ms = 12000; break;
        case SPIPPY_PROXIMITY_EVENT_ENTER_TOO_CLOSE: event_cooldown_ms = 15000; break;
        case SPIPPY_PROXIMITY_EVENT_SUDDEN_APPROACH:
        case SPIPPY_PROXIMITY_EVENT_HOLD_CLOSE: event_cooldown_ms = 20000; break;
        case SPIPPY_PROXIMITY_EVENT_LEAVE: event_cooldown_ms = 30000; break;
        default: break;
    }
    size_t event_index = static_cast<size_t>(event);
    if (event_index < last_proximity_event_us_.size() &&
        last_proximity_event_us_[event_index] != 0 &&
        now - last_proximity_event_us_[event_index] < MsToUs(event_cooldown_ms)) {
        return;
    }
    if (event == SPIPPY_PROXIMITY_EVENT_ENTER_NEAR && (esp_random() % 2) != 0) {
        return;
    }
    if (event == SPIPPY_PROXIMITY_EVENT_LEAVE && (esp_random() % 3) != 0) {
        return;
    }

    spippy_action_t action = SPIPPY_ACTION_IDLE;
    static const spippy_action_t close_actions[] = {
        SPIPPY_ACTION_NOD,
        SPIPPY_ACTION_WAVE,
        SPIPPY_ACTION_CUTE_WIGGLE,
    };
    switch (event) {
        case SPIPPY_PROXIMITY_EVENT_ENTER_NEAR:
            action = SPIPPY_ACTION_IDLE_BREATHE;
            break;
        case SPIPPY_PROXIMITY_EVENT_ENTER_CLOSE:
            action = close_actions[esp_random() % (sizeof(close_actions) / sizeof(close_actions[0]))];
            break;
        case SPIPPY_PROXIMITY_EVENT_ENTER_TOO_CLOSE:
        case SPIPPY_PROXIMITY_EVENT_SUDDEN_APPROACH:
            action = SPIPPY_ACTION_STARTLE;
            break;
        case SPIPPY_PROXIMITY_EVENT_HOLD_CLOSE:
            action = SPIPPY_ACTION_CUTE_WIGGLE;
            break;
        case SPIPPY_PROXIMITY_EVENT_LEAVE:
            action = SPIPPY_ACTION_WAVE;
            break;
        default:
            return;
    }

    cJSON *result = StartAction(action, 1, SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE);
    if (result != nullptr) {
        bool accepted = cJSON_IsString(cJSON_GetObjectItem(result, "status")) &&
                        strcmp(cJSON_GetObjectItem(result, "status")->valuestring, "ok") == 0;
        cJSON_Delete(result);
        if (accepted) {
            last_proximity_reaction_us_ = now;
            if (event_index < last_proximity_event_us_.size()) {
                last_proximity_event_us_[event_index] = now;
            }
        }
    }
#endif
}

void SpippyController::TriggerAutonomyInteraction(bool force) {
#if CONFIG_SPIPPY_AUTONOMY_ENABLE
    if (!autonomy_enabled_) {
        return;
    }
    int64_t now = esp_timer_get_time();
    if (!force && !RuntimeAllowsIdleInteraction()) {
        last_autonomy_event_ = "suppressed_unsafe";
        ScheduleNextAutonomy(now);
        return;
    }
    if (!force && proximity_zone_ == SPIPPY_PROXIMITY_ZONE_TOO_CLOSE) {
        last_autonomy_event_ = "suppressed_too_close";
        ScheduleNextAutonomy(now);
        return;
    }

    uint32_t pick = esp_random() % 9;
    if (pick == 0) {
        if (display_ != nullptr) {
            display_->ShowFace("blink", 1000);
        }
        last_autonomy_event_ = "blink";
        ScheduleNextAutonomy(now);
        return;
    }

    static const spippy_action_t default_actions[] = {
        SPIPPY_ACTION_IDLE_BREATHE,
        SPIPPY_ACTION_NOD,
        SPIPPY_ACTION_CUTE_WIGGLE,
        SPIPPY_ACTION_SLEEPY,
        SPIPPY_ACTION_COQUETRY,
        SPIPPY_ACTION_WAVE,
        SPIPPY_ACTION_POSE_1,
        SPIPPY_ACTION_SHAKE,
    };
    static const spippy_action_t present_actions[] = {
        SPIPPY_ACTION_IDLE_BREATHE,
        SPIPPY_ACTION_NOD,
        SPIPPY_ACTION_WAVE,
        SPIPPY_ACTION_CUTE_WIGGLE,
    };
    const spippy_action_t *actions = default_actions;
    size_t action_count = sizeof(default_actions) / sizeof(default_actions[0]);
    if (!force && (proximity_zone_ == SPIPPY_PROXIMITY_ZONE_NEAR ||
                   proximity_zone_ == SPIPPY_PROXIMITY_ZONE_CLOSE)) {
        actions = present_actions;
        action_count = sizeof(present_actions) / sizeof(present_actions[0]);
    }
    spippy_action_t action = actions[(pick - 1) % action_count];
    cJSON *result = StartAction(action, 1, SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE);
    if (result != nullptr) {
        cJSON_Delete(result);
    }
    last_autonomy_event_ = ActionName(action);
    ScheduleNextAutonomy(now);
#endif
}

bool SpippyController::CalibrationPreviewActive() const {
    bool enabled = false;
    return calibration_service_get_preview_mode(&enabled) == ESP_OK && enabled;
}

bool SpippyController::ServoPreflightActive() const {
    bool enabled = false;
    return calibration_service_get_preflight_mode(&enabled) == ESP_OK && enabled;
}

bool SpippyController::ServoMaintenanceActive() const {
    return CalibrationPreviewActive() || ServoPreflightActive();
}

int SpippyController::BatteryPercentFromVoltage(int mv) const {
    if (mv <= 0) {
        return 0;
    }
    const int empty_mv = CONFIG_SPIPPY_POWER_LOW_VOLTAGE_MV;
    const int full_mv = 8400;
    int percent = ((mv - empty_mv) * 100) / (full_mv - empty_mv);
    return std::max(0, std::min(100, percent));
}

void SpippyController::SetFaceForAction(spippy_action_t action) {
    if (display_ == nullptr) {
        return;
    }
    switch (action) {
        case SPIPPY_ACTION_FORWARD:
        case SPIPPY_ACTION_BACKWARD:
        case SPIPPY_ACTION_TURN_LEFT:
        case SPIPPY_ACTION_TURN_RIGHT:
            display_->ShowActionFace("movement");
            break;
        case SPIPPY_ACTION_WAVE:
            display_->ShowActionFace("wave");
            break;
        case SPIPPY_ACTION_HEAD_SWAY:
        case SPIPPY_ACTION_FRONT_BACK_SWAY:
            display_->ShowActionFace("sesame_shrug");
            break;
        case SPIPPY_ACTION_SHAKE:
            display_->ShowActionFace("shake");
            break;
        case SPIPPY_ACTION_BOW:
            display_->ShowActionFace("bow");
            break;
        case SPIPPY_ACTION_CONGRATS:
            display_->ShowActionFace("cute");
            break;
        case SPIPPY_ACTION_SIT:
            display_->ShowActionFace("rest");
            break;
        case SPIPPY_ACTION_PUSHUP:
            display_->ShowActionFace("pushup");
            break;
        case SPIPPY_ACTION_DANCE_1:
        case SPIPPY_ACTION_DANCE_2:
        case SPIPPY_ACTION_BOUNCE:
            display_->ShowActionFace("dance");
            break;
        case SPIPPY_ACTION_POSE_1:
            display_->ShowActionFace("point");
            break;
        case SPIPPY_ACTION_CONFUSED:
            display_->ShowActionFace("confused");
            break;
        case SPIPPY_ACTION_IDLE_BREATHE:
            display_->ShowActionFace("blink");
            break;
        case SPIPPY_ACTION_NOD:
        case SPIPPY_ACTION_CUTE_WIGGLE:
        case SPIPPY_ACTION_COQUETRY:
            display_->ShowActionFace("cute");
            break;
        case SPIPPY_ACTION_STARTLE:
        case SPIPPY_ACTION_AFRAID:
            display_->ShowActionFace("sesame_freaky");
            break;
        case SPIPPY_ACTION_VOICE_PERFORM:
            display_->ShowActionFace("angry");
            break;
        case SPIPPY_ACTION_ANGRY:
            display_->ShowActionFace("angry");
            break;
        case SPIPPY_ACTION_SLEEPY:
        case SPIPPY_ACTION_PRONE:
            display_->ShowActionFace("rest");
            break;
        case SPIPPY_ACTION_PEE_RIGHT_REAR:
            display_->ShowActionFace("cute");
            break;
        default:
            display_->ShowActionFace("idle");
            break;
    }
}

cJSON *SpippyController::ErrorJson(const char *code, const char *message) const {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "status", "error");
    cJSON_AddStringToObject(root, "code", code);
    cJSON_AddStringToObject(root, "message", message);
    return root;
}

cJSON *SpippyController::OkJson(const char *message) const {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "status", "ok");
    cJSON_AddStringToObject(root, "message", message);
    return root;
}

cJSON *SpippyController::ActionAcceptedJson(spippy_action_t action, uint32_t repeat) const {
    cJSON *root = OkJson("action accepted");
    cJSON_AddStringToObject(root, "action", ActionName(action));
    cJSON_AddNumberToObject(root, "repeat", repeat);
    cJSON_AddStringToObject(root, "speed", speed_ == SPIPPY_SPEED_LEVEL_LOW ? "low" : speed_ == SPIPPY_SPEED_LEVEL_HIGH ? "high" : "medium");
    return root;
}

cJSON *SpippyController::MovementAcceptedJson(spippy_action_t action, CompletionMode mode,
                                              uint32_t steps, uint32_t duration_ms,
                                              bool continued_without_transition) const {
    cJSON *root = OkJson("movement accepted");
    cJSON_AddStringToObject(root, "action", ActionName(action));
    cJSON_AddStringToObject(root, "mode", mode == CompletionMode::kCycles ? "steps" : CompletionModeName(mode));
    if (mode == CompletionMode::kCycles) {
        cJSON_AddNumberToObject(root, "steps", steps);
    } else if (mode == CompletionMode::kDeadline) {
        cJSON_AddNumberToObject(root, "duration_ms", duration_ms);
        cJSON_AddStringToObject(root, "stop_behavior", "complete_current_gait_cycle");
    } else {
        cJSON_AddNumberToObject(root, "safety_timeout_s", CONFIG_SPIPPY_CONTINUOUS_MOVE_MAX_DURATION_S);
    }
    cJSON_AddBoolToObject(root, "continued_without_transition", continued_without_transition);
    cJSON_AddStringToObject(root, "speed",
                            speed_ == SPIPPY_SPEED_LEVEL_LOW ? "low" :
                            speed_ == SPIPPY_SPEED_LEVEL_HIGH ? "high" : "medium");
    return root;
}

cJSON *SpippyController::SequenceAcceptedJson(std::initializer_list<QueuedAction> sequence) const {
    cJSON *root = OkJson("showcase accepted");
    cJSON *items = cJSON_CreateArray();
    uint32_t total_repeat = 0;
    for (const auto &item : sequence) {
        cJSON *entry = cJSON_CreateObject();
        uint32_t repeat = std::max<uint32_t>(1, std::min<uint32_t>(item.repeat, 12));
        cJSON_AddStringToObject(entry, "action", ActionName(item.action));
        cJSON_AddNumberToObject(entry, "repeat", repeat);
        cJSON_AddItemToArray(items, entry);
        total_repeat += repeat;
    }
    cJSON_AddItemToObject(root, "sequence", items);
    cJSON_AddNumberToObject(root, "sequence_count", sequence.size());
    cJSON_AddNumberToObject(root, "total_repeat", total_repeat);
    cJSON_AddStringToObject(root, "speed", speed_ == SPIPPY_SPEED_LEVEL_LOW ? "low" : speed_ == SPIPPY_SPEED_LEVEL_HIGH ? "high" : "medium");
    return root;
}
