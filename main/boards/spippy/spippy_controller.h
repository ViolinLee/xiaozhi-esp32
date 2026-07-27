#pragma once

#include <cJSON.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <array>
#include <initializer_list>
#include <string>

#include "spippy_display.h"

extern "C" {
#include "modules/buzzer_manager.h"
#include "modules/calibration_service.h"
#include "modules/forward_guard.h"
#include "modules/low_power_reminder_policy.h"
#include "modules/motion_engine.h"
#include "modules/power_manager.h"
#include "modules/sensor_manager.h"
#include "modules/servo_manager.h"
#include "modules/storage.h"
#include "spippy_types.h"
}

class SpippyController {
public:
    explicit SpippyController(SpippyDisplay *display);
    ~SpippyController();

    void Initialize();
    void RegisterMcpTools();
    bool GetBatteryLevel(int &level, bool &charging, bool &discharging);

    cJSON *Move(const std::string &action, const std::string &mode, int steps, int duration_ms);
    cJSON *Perform(const std::string &action, int repeat);
    cJSON *ShowExpression(const std::string &expression, int duration_ms);
    cJSON *SetSpeed(const std::string &speed);
    cJSON *Showcase(const std::string &mode);
    cJSON *StatusJson();
    cJSON *BatteryJson();
    cJSON *DistanceJson();
    cJSON *WebConsoleJson();
    cJSON *CalibrationStatusJson();
    cJSON *CalibrationPreview(bool enabled);
    cJSON *CalibrationPreflight(bool enabled);
    cJSON *CalibrationServo(int index, int angle_x10);
    cJSON *CalibrationServoUpdate(int index, int zero_offset_x10);
    cJSON *CalibrationPose(const std::string &pose);
    cJSON *CalibrationSave();
    cJSON *SetAutonomyEnabled(bool enabled);
    bool GetAutonomyEnabled() const;

    bool IsForwardBlocked() const;

private:
    static constexpr size_t kMaxPendingActions = 8;
    static constexpr size_t kProximitySampleWindow = 5;

    enum class CompletionMode : uint8_t {
        kCycles,
        kDeadline,
        kExplicitStop,
    };

    struct Request {
        spippy_action_t action = SPIPPY_ACTION_IDLE;
        uint32_t repeat_remaining = 0;
        CompletionMode completion_mode = CompletionMode::kCycles;
        int64_t deadline_us = 0;
        spippy_action_completion_policy_t completion_policy = SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE;
        bool active = false;
    };

    struct QueuedAction {
        spippy_action_t action = SPIPPY_ACTION_IDLE;
        uint32_t repeat = 1;
        spippy_action_completion_policy_t completion_policy = SPIPPY_ACTION_COMPLETION_POLICY_RELEASE_TO_IDLE;
    };

    static void TaskEntry(void *arg);
    void TaskLoop();
    cJSON *StartMovement(spippy_action_t action, CompletionMode mode, uint32_t steps, uint32_t duration_ms);
    cJSON *StartAction(spippy_action_t action, uint32_t repeat, spippy_action_completion_policy_t policy);
    cJSON *StartSequence(std::initializer_list<QueuedAction> sequence);
    cJSON *ErrorJson(const char *code, const char *message) const;
    cJSON *OkJson(const char *message) const;
    cJSON *ActionAcceptedJson(spippy_action_t action, uint32_t repeat) const;
    cJSON *MovementAcceptedJson(spippy_action_t action, CompletionMode mode, uint32_t steps,
                                uint32_t duration_ms, bool continued_without_transition) const;
    cJSON *ForwardGuardErrorJson() const;
    cJSON *LowPowerErrorJson();
    cJSON *SequenceAcceptedJson(std::initializer_list<QueuedAction> sequence) const;
    bool EnqueueActionLocked(const QueuedAction &action);
    bool ResolveAction(const std::string &text, spippy_action_t &action, bool movement_only) const;
    bool ResolveMovementMode(const std::string &text, CompletionMode &mode) const;
    bool ResolveSpeed(const std::string &text, spippy_speed_level_t &speed) const;
    bool ResolvePose(const std::string &text, spippy_pose_mode_t &pose) const;
    const char *ActionName(spippy_action_t action) const;
    const char *CompletionModeName(CompletionMode mode) const;
    bool IsLocomotionAction(spippy_action_t action) const;
    const char *ZoneName(spippy_proximity_zone_t zone) const;
    const char *ProximityEventName(spippy_proximity_event_t event) const;
    void UpdatePower();
    void UpdateDistance();
    void UpdateForwardGuard();
    void UpdateProximity();
    void UpdateAutonomy();
    void HandleLowPower();
    void ScheduleLowPowerVoiceReminder(const char *source, bool append_after_tts);
    void MarkLowPowerReminderHandledByResponse();
    void HandleActionCycle();
    void PreemptLocomotionForUserCommand();
    void NoteUserActivity();
    void ScheduleNextAutonomy(int64_t base_us);
    bool RuntimeAllowsIdleInteraction() const;
    int64_t RandomAutonomyIntervalUs() const;
    spippy_proximity_zone_t ClassifyProximityDistance(uint32_t distance_mm) const;
    bool ConfirmProximityZone(spippy_proximity_zone_t candidate);
    spippy_proximity_event_t ProximityEventForZoneChange() const;
    void RecordProximityEvent(spippy_proximity_event_t event);
    void TriggerProximityInteraction(spippy_proximity_event_t event);
    void TriggerAutonomyInteraction(bool force);
    bool CalibrationPreviewActive() const;
    bool ServoPreflightActive() const;
    bool ServoMaintenanceActive() const;
    int BatteryPercentFromVoltage(int mv) const;
    void SetFaceForAction(spippy_action_t action);

    SpippyDisplay *display_ = nullptr;
    SemaphoreHandle_t lock_ = nullptr;
    SemaphoreHandle_t runtime_lock_ = nullptr;
    TaskHandle_t task_ = nullptr;
    Request request_;
    std::array<QueuedAction, kMaxPendingActions> pending_actions_ = {};
    size_t pending_count_ = 0;
    spippy_speed_level_t speed_ = SPIPPY_SPEED_LEVEL_MID;
    spippy_power_telemetry_t power_ = {};
    spippy_low_power_reminder_policy_t low_power_reminder_ = {};
    spippy_ultrasonic_reading_t distance_ = {};
    spippy_forward_guard_t forward_guard_ = {};
    spippy_proximity_zone_t proximity_zone_ = SPIPPY_PROXIMITY_ZONE_LOST;
    uint32_t power_ticks_ = 0;
    uint32_t sensor_ticks_ = 0;
    uint32_t autonomy_ticks_ = 0;
    std::array<uint32_t, kProximitySampleWindow> proximity_samples_ = {};
    uint32_t proximity_sample_count_ = 0;
    uint32_t proximity_sample_next_ = 0;
    spippy_proximity_zone_t proximity_previous_zone_ = SPIPPY_PROXIMITY_ZONE_LOST;
    spippy_proximity_zone_t proximity_candidate_zone_ = SPIPPY_PROXIMITY_ZONE_LOST;
    uint32_t proximity_candidate_count_ = 0;
    uint32_t proximity_stable_sample_count_ = 0;
    uint32_t proximity_invalid_streak_ = 0;
    uint32_t proximity_filtered_distance_mm_ = 0;
    uint32_t proximity_previous_raw_distance_mm_ = 0;
    bool proximity_previous_filtered_valid_ = false;
    bool proximity_previous_raw_valid_ = false;
    bool proximity_hold_close_emitted_ = false;
    bool proximity_sudden_approach_emitted_ = false;
    spippy_proximity_event_t proximity_last_event_ = SPIPPY_PROXIMITY_EVENT_NONE;
    uint32_t proximity_event_count_ = 0;
    int64_t proximity_close_since_us_ = 0;
    int64_t last_proximity_reaction_us_ = 0;
    std::array<int64_t, SPIPPY_PROXIMITY_EVENT_LEAVE + 1> last_proximity_event_us_ = {};
    int64_t last_user_activity_us_ = 0;
    int64_t next_autonomy_due_us_ = 0;
    std::string last_autonomy_event_ = "NONE";
    bool autonomy_enabled_ = true;
    bool initialized_ = false;
};
