#include "spippy_display.h"

#include <algorithm>
#include <cstring>
#include <string>

#include "display/display.h"
#include "generated/spippy_faces.h"

namespace {

constexpr uint32_t kConversationHoldMs = 2500;
constexpr uint32_t kIdleBlinkIntervalMs = 5000;
constexpr uint32_t kIdleBlinkFrameMs = 140;
constexpr uint32_t kFaceTickMs = 100;
constexpr int32_t kChatViewportWidth = 124;
constexpr int32_t kChatViewportHeight = 16;
constexpr int32_t kChatScrollPixelsPerSecond = 60;
constexpr uint32_t kChatScrollMinDurationMs = 700;
constexpr uint32_t kChatScrollMaxDurationMs = 60000;
constexpr uint32_t kChatScrollStartDelayMs = 250;
constexpr uint32_t kChatShortHoldMs = 4000;

struct FaceSequence {
    const char *profile;
    const char *const *frames;
    size_t frame_count;
    uint32_t frame_interval_ms;
    bool idle_blink;
};

constexpr const char *kIdleFrames[] = {
    "sesame_idle", "sesame_idle_blink", "sesame_idle_blink_1",
    "sesame_idle_blink_2", "sesame_idle_blink_3", "sesame_idle_blink_1",
};
constexpr const char *kDanceFrames[] = {"face_dance", "sesame_dance_1"};
constexpr const char *kRestFrames[] = {"sesame_rest", "sesame_rest_1", "sesame_rest_2", "sesame_rest_1"};
constexpr const char *kPointFrames[] = {"sesame_point", "sesame_point_1", "sesame_point_2", "sesame_point_1"};
constexpr const char *kCalibrationFrames[] = {"face_calibration", "sesame_rest_1", "sesame_rest_2", "sesame_rest_1"};
constexpr const char *kLowPowerFrames[] = {"face_low_power", "sesame_dead_1", "sesame_dead_2", "sesame_dead_1"};
constexpr const char *kHappyFrames[] = {"sesame_cute", "sesame_swim", "sesame_crab", "sesame_cute"};
constexpr const char *kSadFrames[] = {"sesame_dead_1", "sesame_dead_2", "sesame_dead_1"};

constexpr FaceSequence kFaceSequences[] = {
    {"idle", kIdleFrames, sizeof(kIdleFrames) / sizeof(kIdleFrames[0]), kIdleBlinkFrameMs, true},
    {"dance", kDanceFrames, sizeof(kDanceFrames) / sizeof(kDanceFrames[0]), 1100, false},
    {"rest", kRestFrames, sizeof(kRestFrames) / sizeof(kRestFrames[0]), 1400, false},
    {"point", kPointFrames, sizeof(kPointFrames) / sizeof(kPointFrames[0]), 700, false},
    {"calibration", kCalibrationFrames, sizeof(kCalibrationFrames) / sizeof(kCalibrationFrames[0]), 1400, false},
    {"low_power", kLowPowerFrames, sizeof(kLowPowerFrames) / sizeof(kLowPowerFrames[0]), 1000, false},
    {"happy", kHappyFrames, sizeof(kHappyFrames) / sizeof(kHappyFrames[0]), 950, false},
    {"sad", kSadFrames, sizeof(kSadFrames) / sizeof(kSadFrames[0]), 900, false},
};

const FaceSequence *FindFaceSequence(const char *profile) {
    if (profile == nullptr) {
        return nullptr;
    }
    for (const auto &sequence : kFaceSequences) {
        if (strcmp(sequence.profile, profile) == 0) {
            return &sequence;
        }
    }
    return nullptr;
}

bool IsPersistentSystemMessage(const char *role, const std::string &message) {
    if (role == nullptr || strcmp(role, "system") != 0) {
        return false;
    }

    // Provisioning and activation instructions must remain readable until the
    // framework explicitly replaces or clears them. Keep this policy inside
    // the Spippy BSP instead of changing XiaoZhi's shared display behavior.
    return message.find("xiaozhi.me") != std::string::npos ||
           message.find("http://") != std::string::npos ||
           message.find("https://") != std::string::npos;
}

}  // namespace

SpippyDisplay::SpippyDisplay(esp_lcd_panel_io_handle_t panel_io,
                           esp_lcd_panel_handle_t panel,
                           int width,
                           int height,
                           bool mirror_x,
                           bool mirror_y)
    : OledDisplay(panel_io, panel, width, height, mirror_x, mirror_y) {
    esp_timer_create_args_t args = {
        .callback = IdleTimerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "spippy_face",
        .skip_unhandled_events = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&args, &idle_timer_));
}

SpippyDisplay::~SpippyDisplay() {
    if (idle_timer_ != nullptr) {
        esp_timer_stop(idle_timer_);
        esp_timer_delete(idle_timer_);
    }
    if (battery_container_ != nullptr) {
        lv_obj_del(battery_container_);
    }
    if (chat_viewport_ != nullptr) {
        lv_obj_del(chat_viewport_);
    }
}

void SpippyDisplay::SetupUI() {
    OledDisplay::SetupUI();
    DisplayLockGuard lock(this);

    // Spippy owns its battery thresholds, safety latch, warning face, buzzer,
    // and reminder timing. The generic OLED popup treats every 0-19% reading
    // as an immediate low-battery warning, which conflicts with those rules
    // and becomes exposed while the activation screen hides the face layer.
    if (low_battery_popup_ != nullptr) {
        lv_obj_del(low_battery_popup_);
        low_battery_popup_ = nullptr;
        low_battery_label_ = nullptr;
    }

    auto screen = lv_screen_active();
    face_image_ = lv_image_create(screen);
    lv_obj_set_size(face_image_, 128, 64);
    lv_obj_align(face_image_, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(face_image_, LV_OBJ_FLAG_CLICKABLE);
    lv_image_set_src(face_image_, spippy_face_find("neutral"));

    chat_viewport_ = lv_obj_create(screen);
    lv_obj_set_size(chat_viewport_, kChatViewportWidth, kChatViewportHeight);
    lv_obj_align(chat_viewport_, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_opa(chat_viewport_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(chat_viewport_, 0, 0);
    lv_obj_set_style_pad_all(chat_viewport_, 0, 0);
    lv_obj_set_scrollbar_mode(chat_viewport_, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(chat_viewport_, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(chat_viewport_, LV_OBJ_FLAG_HIDDEN);

    chat_overlay_ = lv_label_create(chat_viewport_);
    lv_obj_set_width(chat_overlay_, LV_SIZE_CONTENT);
    lv_label_set_long_mode(chat_overlay_, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_align(chat_overlay_, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_align(chat_overlay_, LV_ALIGN_LEFT_MID, 0, 0);
    lv_label_set_text(chat_overlay_, "");

    // Match XiaoZhi's native OLED marquee for persistent instructions: LVGL
    // inserts spacing between the tail and head and scrolls them continuously.
    static lv_anim_t circular_scroll_animation;
    lv_anim_init(&circular_scroll_animation);
    lv_anim_set_delay(&circular_scroll_animation, 1000);
    lv_anim_set_repeat_count(&circular_scroll_animation, LV_ANIM_REPEAT_INFINITE);
    lv_obj_set_style_anim(chat_overlay_, &circular_scroll_animation, LV_PART_MAIN);
    lv_obj_set_style_anim_duration(chat_overlay_,
                                   lv_anim_speed_clamped(60, 300, kChatScrollMaxDurationMs),
                                   LV_PART_MAIN);

    CreateBatteryIconLocked();
    UpdateBatteryIconLocked();
    MoveOverlayForegroundLocked();

    esp_timer_start_periodic(idle_timer_, kFaceTickMs * 1000);
}

void SpippyDisplay::SetEmotion(const char *emotion) {
    const char *profile = MapEmotion(emotion);
    DisplayLockGuard lock(this);
    if (strcmp(profile, "neutral") == 0 || strcmp(profile, "idle") == 0) {
        ClearLayerLocked(emotion_face_);
    } else if (strcmp(profile, "low_power") == 0 || strcmp(profile, "calibration") == 0) {
        SetLayerLocked(system_face_, profile, 0);
    } else {
        SetLayerLocked(emotion_face_, profile, kConversationHoldMs);
    }
    RenderSelectedProfileLocked(esp_timer_get_time());
}

void SpippyDisplay::SetRobotState(const char *state) {
    if (state == nullptr || state[0] == '\0') {
        state = "idle";
    }
    DisplayLockGuard lock(this);
    strlcpy(robot_state_, state, sizeof(robot_state_));
    if (strcmp(state, "low_power") == 0 || strcmp(state, "calibration") == 0) {
        SetLayerLocked(system_face_, MapEmotion(state), 0);
    } else if (strcmp(state, "idle") == 0) {
        ClearLayerLocked(action_face_);
        if (strcmp(system_face_.profile, "calibration") == 0) {
            ClearLayerLocked(system_face_);
        }
    } else {
        SetLayerLocked(action_face_, MapEmotion(state), 0);
    }
    RenderSelectedProfileLocked(esp_timer_get_time());
}

void SpippyDisplay::ClearProtectedSystemFace() {
    DisplayLockGuard lock(this);
    ClearLayerLocked(system_face_);
    RenderSelectedProfileLocked(esp_timer_get_time());
}

void SpippyDisplay::SetChatMessage(const char *role, const char *content) {
    DisplayLockGuard lock(this);
    if (chat_overlay_ == nullptr || chat_viewport_ == nullptr) {
        return;
    }
    if (content == nullptr || content[0] == '\0') {
        ClearChatLocked();
        if (activation_screen_visible_) {
            activation_screen_visible_ = false;
            lv_obj_remove_flag(face_image_, LV_OBJ_FLAG_HIDDEN);
            UpdateBatteryIconLocked();
            RenderSelectedProfileLocked(esp_timer_get_time());
        }
        return;
    }

    std::string message(content);
    std::replace(message.begin(), message.end(), '\n', ' ');
    const bool persistent_message = IsPersistentSystemMessage(role, message);
    const bool activation_message = persistent_message &&
                                    message.find("xiaozhi.me") != std::string::npos;
    activation_screen_visible_ = activation_message;
    if (activation_message) {
        lv_obj_add_flag(face_image_, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_remove_flag(face_image_, LV_OBJ_FLAG_HIDDEN);
    }
    UpdateBatteryIconLocked();

    lv_anim_delete(chat_overlay_, ChatScrollExec);
    lv_obj_set_x(chat_overlay_, 0);
    lv_obj_set_width(chat_overlay_, persistent_message ?
                                          lv_obj_get_content_width(chat_viewport_) :
                                          LV_SIZE_CONTENT);
    lv_label_set_long_mode(chat_overlay_, persistent_message ?
                                              LV_LABEL_LONG_SCROLL_CIRCULAR :
                                              LV_LABEL_LONG_CLIP);
    lv_label_set_text(chat_overlay_, message.c_str());
    lv_obj_remove_flag(chat_viewport_, LV_OBJ_FLAG_HIDDEN);
    StartChatScrollLocked(persistent_message);
    MoveOverlayForegroundLocked();
}

void SpippyDisplay::ClearChatMessages() {
    DisplayLockGuard lock(this);
    ClearChatLocked();
    if (activation_screen_visible_) {
        activation_screen_visible_ = false;
        lv_obj_remove_flag(face_image_, LV_OBJ_FLAG_HIDDEN);
        UpdateBatteryIconLocked();
        RenderSelectedProfileLocked(esp_timer_get_time());
    }
}

void SpippyDisplay::SetBatteryLevel(int percent, bool low_power) {
    if (percent < 0) {
        percent = 0;
    } else if (percent > 100) {
        percent = 100;
    }
    DisplayLockGuard lock(this);
    battery_percent_ = percent;
    battery_valid_ = true;
    battery_low_power_ = low_power;
    UpdateBatteryIconLocked();
    MoveOverlayForegroundLocked();
}

void SpippyDisplay::ShowFace(const char *name, uint32_t hold_ms) {
    DisplayLockGuard lock(this);
    SetLayerLocked(transient_face_, name, hold_ms == 0 ? kConversationHoldMs : hold_ms);
    RenderSelectedProfileLocked(esp_timer_get_time());
}

void SpippyDisplay::ShowActionFace(const char *name, uint32_t hold_ms) {
    DisplayLockGuard lock(this);
    SetLayerLocked(action_face_, name, hold_ms);
    RenderSelectedProfileLocked(esp_timer_get_time());
}

bool SpippyDisplay::ShowExpression(const char *name, uint32_t hold_ms) {
    if (!IsKnownProfile(name)) {
        return false;
    }
    ShowFace(name, hold_ms);
    return true;
}

void SpippyDisplay::ShowWebConsoleAddress(const std::string &url, uint32_t duration_ms) {
    DisplayLockGuard lock(this);
    if (chat_overlay_ == nullptr || chat_viewport_ == nullptr || url.empty()) {
        return;
    }

    // The shared LvglDisplay notification label sits below Spippy's face
    // layer. Use the BSP-owned subtitle lane so the local address is visible
    // above the face and battery overlays for the requested duration.
    lv_anim_delete(chat_overlay_, ChatScrollExec);
    lv_obj_set_x(chat_overlay_, 0);
    lv_obj_set_width(chat_overlay_, lv_obj_get_content_width(chat_viewport_));
    lv_label_set_long_mode(chat_overlay_, LV_LABEL_LONG_SCROLL_CIRCULAR);
    const std::string message = "后台：" + url;
    lv_label_set_text(chat_overlay_, message.c_str());
    lv_obj_remove_flag(chat_viewport_, LV_OBJ_FLAG_HIDDEN);
    chat_clear_at_us_ = esp_timer_get_time() + static_cast<int64_t>(duration_ms) * 1000;
    MoveOverlayForegroundLocked();
}

void SpippyDisplay::IdleTimerCallback(void *arg) {
    static_cast<SpippyDisplay *>(arg)->TickIdleFace();
}

const char *SpippyDisplay::MapEmotion(const char *emotion) const {
    if (emotion == nullptr || emotion[0] == '\0') {
        return "neutral";
    }
    if (strcmp(emotion, "listening") == 0 || strcmp(emotion, "wake") == 0) {
        return "listening";
    }
    if (strcmp(emotion, "speaking") == 0 || strcmp(emotion, "talking") == 0) {
        return "speaking";
    }
    if (strcmp(emotion, "thinking") == 0 || strcmp(emotion, "processing") == 0) {
        return "thinking";
    }
    if (strcmp(emotion, "happy") == 0 || strcmp(emotion, "laughing") == 0 ||
        strcmp(emotion, "funny") == 0 || strcmp(emotion, "loving") == 0 ||
        strcmp(emotion, "winking") == 0 || strcmp(emotion, "cool") == 0 ||
        strcmp(emotion, "delicious") == 0 || strcmp(emotion, "kissy") == 0 ||
        strcmp(emotion, "confident") == 0 || strcmp(emotion, "silly") == 0) {
        return "happy";
    }
    if (strcmp(emotion, "sad") == 0 || strcmp(emotion, "crying") == 0) {
        return "sad";
    }
    if (strcmp(emotion, "sleepy") == 0) {
        return "rest";
    }
    if (strcmp(emotion, "angry") == 0) {
        return "angry";
    }
    if (strcmp(emotion, "surprised") == 0 || strcmp(emotion, "shocked") == 0) {
        return "surprised";
    }
    if (strcmp(emotion, "confused") == 0 || strcmp(emotion, "unsure") == 0 ||
        strcmp(emotion, "embarrassed") == 0) {
        return "confused";
    }
    if (strcmp(emotion, "relaxed") == 0 || strcmp(emotion, "microchip_ai") == 0) {
        return "neutral";
    }
    if (strcmp(emotion, "calibration") == 0) {
        return "calibration";
    }
    if (strcmp(emotion, "low_power") == 0) {
        return "low_power";
    }
    if (strcmp(emotion, "movement") == 0) {
        return "movement";
    }
    if (strcmp(emotion, "action") == 0) {
        return "action";
    }
    return "neutral";
}

bool SpippyDisplay::IsKnownProfile(const char *name) const {
    return name != nullptr && (FindFaceSequence(name) != nullptr || spippy_face_exists(name));
}

void SpippyDisplay::SetLayerLocked(FaceLayer &layer, const char *profile, uint32_t hold_ms) {
    strlcpy(layer.profile, profile == nullptr ? "neutral" : profile, sizeof(layer.profile));
    layer.expires_at_us = hold_ms == 0 ? 0 : esp_timer_get_time() + static_cast<int64_t>(hold_ms) * 1000;
    layer.active = true;
}

void SpippyDisplay::ClearLayerLocked(FaceLayer &layer) {
    layer.profile[0] = '\0';
    layer.expires_at_us = 0;
    layer.active = false;
}

void SpippyDisplay::ExpireLayersLocked(int64_t now_us) {
    FaceLayer *layers[] = {&action_face_, &transient_face_, &emotion_face_};
    for (FaceLayer *layer : layers) {
        if (layer->active && layer->expires_at_us > 0 && now_us >= layer->expires_at_us) {
            ClearLayerLocked(*layer);
        }
    }
}

const char *SpippyDisplay::SelectProfileLocked() const {
    if (system_face_.active) {
        return system_face_.profile;
    }
    if (action_face_.active) {
        return action_face_.profile;
    }
    if (transient_face_.active) {
        return transient_face_.profile;
    }
    if (emotion_face_.active) {
        return emotion_face_.profile;
    }
    return "idle";
}

void SpippyDisplay::RenderSelectedProfileLocked(int64_t now_us) {
    ExpireLayersLocked(now_us);
    const char *profile = SelectProfileLocked();
    const FaceSequence *sequence = FindFaceSequence(profile);
    if (strcmp(rendered_profile_, profile) != 0) {
        strlcpy(rendered_profile_, profile, sizeof(rendered_profile_));
        idle_index_ = 0;
        next_frame_at_us_ = now_us;
        RenderProfileFrameLocked(profile, 0);
        if (sequence == nullptr || sequence->frame_count <= 1) {
            next_frame_at_us_ = 0;
        } else if (sequence->idle_blink) {
            next_frame_at_us_ = now_us + static_cast<int64_t>(kIdleBlinkIntervalMs) * 1000;
        } else {
            next_frame_at_us_ = now_us + static_cast<int64_t>(sequence->frame_interval_ms) * 1000;
        }
        return;
    }
    if (sequence == nullptr || sequence->frame_count <= 1 || next_frame_at_us_ == 0 || now_us < next_frame_at_us_) {
        return;
    }
    if (sequence->idle_blink) {
        if (idle_index_ == 0) {
            idle_index_ = 1;
        } else if (++idle_index_ >= static_cast<int>(sequence->frame_count)) {
            idle_index_ = 0;
        }
        next_frame_at_us_ = now_us + static_cast<int64_t>(idle_index_ == 0 ? kIdleBlinkIntervalMs : sequence->frame_interval_ms) * 1000;
    } else {
        idle_index_ = (idle_index_ + 1) % static_cast<int>(sequence->frame_count);
        next_frame_at_us_ = now_us + static_cast<int64_t>(sequence->frame_interval_ms) * 1000;
    }
    RenderProfileFrameLocked(profile, idle_index_);
}

void SpippyDisplay::RenderProfileFrameLocked(const char *profile, int frame_index) {
    const FaceSequence *sequence = FindFaceSequence(profile);
    if (sequence == nullptr) {
        ShowFaceLocked(profile);
        return;
    }
    const int safe_index = std::max(0, std::min(frame_index, static_cast<int>(sequence->frame_count) - 1));
    ShowFaceLocked(sequence->frames[safe_index]);
}

void SpippyDisplay::CreateBatteryIconLocked() {
    auto screen = lv_screen_active();
    battery_container_ = lv_obj_create(screen);
    lv_obj_set_size(battery_container_, 16, 8);
    lv_obj_align(battery_container_, LV_ALIGN_TOP_RIGHT, -4, 2);
    lv_obj_set_style_bg_color(battery_container_, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(battery_container_, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(battery_container_, 0, 0);
    lv_obj_set_style_radius(battery_container_, 0, 0);
    lv_obj_set_style_pad_all(battery_container_, 0, 0);
    lv_obj_set_scrollbar_mode(battery_container_, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(battery_container_, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *outline = lv_obj_create(battery_container_);
    lv_obj_set_size(outline, 14, 8);
    lv_obj_set_pos(outline, 0, 0);
    lv_obj_set_style_bg_color(outline, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(outline, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(outline, lv_color_black(), 0);
    lv_obj_set_style_border_width(outline, 1, 0);
    lv_obj_set_style_radius(outline, 0, 0);
    lv_obj_set_style_pad_all(outline, 0, 0);
    lv_obj_set_scrollbar_mode(outline, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(outline, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *cap = lv_obj_create(battery_container_);
    lv_obj_set_size(cap, 2, 4);
    lv_obj_set_pos(cap, 14, 2);
    lv_obj_set_style_bg_color(cap, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(cap, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(cap, 0, 0);
    lv_obj_set_style_radius(cap, 0, 0);
    lv_obj_set_style_pad_all(cap, 0, 0);
    lv_obj_set_scrollbar_mode(cap, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(cap, LV_OBJ_FLAG_CLICKABLE);

    for (int i = 0; i < 4; ++i) {
        battery_bars_[i] = lv_obj_create(battery_container_);
        lv_obj_set_size(battery_bars_[i], 2, 4);
        lv_obj_set_pos(battery_bars_[i], 2 + i * 3, 2);
        lv_obj_set_style_bg_color(battery_bars_[i], lv_color_black(), 0);
        lv_obj_set_style_bg_opa(battery_bars_[i], LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(battery_bars_[i], 0, 0);
        lv_obj_set_style_radius(battery_bars_[i], 0, 0);
        lv_obj_set_style_pad_all(battery_bars_[i], 0, 0);
        lv_obj_set_scrollbar_mode(battery_bars_[i], LV_SCROLLBAR_MODE_OFF);
        lv_obj_clear_flag(battery_bars_[i], LV_OBJ_FLAG_CLICKABLE);
    }

    battery_low_mark_ = lv_obj_create(battery_container_);
    lv_obj_set_size(battery_low_mark_, 1, 6);
    lv_obj_set_pos(battery_low_mark_, 7, 1);
    lv_obj_set_style_bg_color(battery_low_mark_, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(battery_low_mark_, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(battery_low_mark_, 0, 0);
    lv_obj_set_style_radius(battery_low_mark_, 0, 0);
    lv_obj_set_style_pad_all(battery_low_mark_, 0, 0);
    lv_obj_set_scrollbar_mode(battery_low_mark_, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(battery_low_mark_, LV_OBJ_FLAG_CLICKABLE);
}

void SpippyDisplay::UpdateBatteryIconLocked() {
    if (battery_container_ == nullptr) {
        return;
    }
    if (!battery_valid_ || activation_screen_visible_) {
        lv_obj_add_flag(battery_container_, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    lv_obj_remove_flag(battery_container_, LV_OBJ_FLAG_HIDDEN);

    int bars = 0;
    if (battery_percent_ >= 80) {
        bars = 4;
    } else if (battery_percent_ >= 60) {
        bars = 3;
    } else if (battery_percent_ >= 40) {
        bars = 2;
    } else if (battery_percent_ >= 20) {
        bars = 1;
    }

    for (int i = 0; i < 4; ++i) {
        if (battery_bars_[i] == nullptr) {
            continue;
        }
        if (i < bars) {
            lv_obj_remove_flag(battery_bars_[i], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(battery_bars_[i], LV_OBJ_FLAG_HIDDEN);
        }
    }

    if (battery_low_mark_ != nullptr) {
        if (battery_low_power_) {
            lv_obj_remove_flag(battery_low_mark_, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(battery_low_mark_, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

void SpippyDisplay::ChatScrollExec(void *obj, int32_t x) {
    lv_obj_set_x(static_cast<lv_obj_t *>(obj), x);
}

void SpippyDisplay::StartChatScrollLocked(bool loop) {
    chat_clear_at_us_ = 0;
    lv_obj_update_layout(chat_viewport_);
    lv_obj_update_layout(chat_overlay_);
    if (loop) {
        // LV_LABEL_LONG_SCROLL_CIRCULAR owns the seamless persistent marquee.
        return;
    }
    const int32_t text_width = lv_obj_get_width(chat_overlay_);
    const int32_t viewport_width = lv_obj_get_content_width(chat_viewport_);
    if (text_width <= viewport_width) {
        if (!loop) {
            chat_clear_at_us_ = esp_timer_get_time() +
                                static_cast<int64_t>(kChatShortHoldMs) * 1000;
        }
        return;
    }

    // Scroll the trailing characters across the entire viewport before
    // clearing. Stopping at text_width - viewport_width makes the final word
    // only just enter from the right before it disappears.
    const int32_t distance = text_width;

    const uint32_t duration = std::clamp(
        static_cast<uint32_t>((distance * 1000 + kChatScrollPixelsPerSecond - 1) / kChatScrollPixelsPerSecond),
        kChatScrollMinDurationMs, kChatScrollMaxDurationMs);
    lv_anim_t animation;
    lv_anim_init(&animation);
    lv_anim_set_var(&animation, chat_overlay_);
    lv_anim_set_values(&animation, 0, -distance);
    lv_anim_set_duration(&animation, duration);
    lv_anim_set_delay(&animation, kChatScrollStartDelayMs);
    lv_anim_set_path_cb(&animation, lv_anim_path_linear);
    lv_anim_set_exec_cb(&animation, ChatScrollExec);
    chat_clear_at_us_ = esp_timer_get_time() +
                        static_cast<int64_t>(kChatScrollStartDelayMs + duration) * 1000;
    lv_anim_start(&animation);
}

void SpippyDisplay::ClearChatLocked() {
    if (chat_overlay_ == nullptr || chat_viewport_ == nullptr) {
        return;
    }
    lv_anim_delete(chat_overlay_, ChatScrollExec);
    chat_clear_at_us_ = 0;
    lv_obj_set_x(chat_overlay_, 0);
    lv_label_set_long_mode(chat_overlay_, LV_LABEL_LONG_CLIP);
    lv_obj_set_width(chat_overlay_, LV_SIZE_CONTENT);
    lv_label_set_text(chat_overlay_, "");
    lv_obj_add_flag(chat_viewport_, LV_OBJ_FLAG_HIDDEN);
}

void SpippyDisplay::MoveOverlayForegroundLocked() {
    if (chat_viewport_ != nullptr && !lv_obj_has_flag(chat_viewport_, LV_OBJ_FLAG_HIDDEN)) {
        lv_obj_move_foreground(chat_viewport_);
    }
    if (battery_container_ != nullptr && !lv_obj_has_flag(battery_container_, LV_OBJ_FLAG_HIDDEN)) {
        lv_obj_move_foreground(battery_container_);
    }
}

void SpippyDisplay::ShowFaceLocked(const char *name) {
    if (face_image_ == nullptr) {
        return;
    }
    const lv_image_dsc_t *image = spippy_face_find(name);
    lv_image_set_src(face_image_, image);
    lv_obj_move_foreground(face_image_);
    MoveOverlayForegroundLocked();
    strlcpy(current_face_, name == nullptr ? "neutral" : name, sizeof(current_face_));
}

void SpippyDisplay::TickIdleFace() {
    if (face_image_ == nullptr) {
        return;
    }
    DisplayLockGuard lock(this);
    const int64_t now_us = esp_timer_get_time();
    if (chat_clear_at_us_ > 0 && now_us >= chat_clear_at_us_) {
        ClearChatLocked();
    }
    RenderSelectedProfileLocked(now_us);
}
