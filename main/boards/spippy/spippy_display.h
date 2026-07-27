#pragma once

#include <esp_timer.h>

#include "display/oled_display.h"

class SpippyDisplay : public OledDisplay {
public:
    SpippyDisplay(esp_lcd_panel_io_handle_t panel_io,
                 esp_lcd_panel_handle_t panel,
                 int width,
                 int height,
                 bool mirror_x,
                 bool mirror_y);
    ~SpippyDisplay() override;

    void SetupUI() override;
    void SetEmotion(const char *emotion) override;
    void SetChatMessage(const char *role, const char *content) override;
    void ClearChatMessages() override;
    // A short, non-motion expression. It never overrides safety or an active action.
    void ShowFace(const char *name, uint32_t hold_ms = 0);
    // An action-owned expression remains visible until the action completes or is replaced.
    void ShowActionFace(const char *name, uint32_t hold_ms = 0);
    bool ShowExpression(const char *name, uint32_t hold_ms);
    // Temporarily takes over the subtitle lane for a local, user-requested URL.
    void ShowWebConsoleAddress(const std::string &url, uint32_t duration_ms = 10000);
    void SetRobotState(const char *state);
    void ClearProtectedSystemFace();
    void SetBatteryLevel(int percent, bool low_power);

private:
    struct FaceLayer {
        char profile[32] = {};
        int64_t expires_at_us = 0;
        bool active = false;
    };

    static void IdleTimerCallback(void *arg);
    const char *MapEmotion(const char *emotion) const;
    bool IsKnownProfile(const char *name) const;
    void SetLayerLocked(FaceLayer &layer, const char *profile, uint32_t hold_ms);
    void ClearLayerLocked(FaceLayer &layer);
    void ExpireLayersLocked(int64_t now_us);
    const char *SelectProfileLocked() const;
    void RenderSelectedProfileLocked(int64_t now_us);
    void RenderProfileFrameLocked(const char *profile, int frame_index);
    void CreateBatteryIconLocked();
    void UpdateBatteryIconLocked();
    static void ChatScrollExec(void *obj, int32_t x);
    void StartChatScrollLocked(bool loop);
    void ClearChatLocked();
    void MoveOverlayForegroundLocked();
    void ShowFaceLocked(const char *name);
    void TickIdleFace();

    lv_obj_t *face_image_ = nullptr;
    lv_obj_t *chat_viewport_ = nullptr;
    lv_obj_t *chat_overlay_ = nullptr;
    lv_obj_t *battery_container_ = nullptr;
    lv_obj_t *battery_bars_[4] = {};
    lv_obj_t *battery_low_mark_ = nullptr;
    esp_timer_handle_t idle_timer_ = nullptr;
    int idle_index_ = 0;
    int battery_percent_ = 0;
    int64_t next_frame_at_us_ = 0;
    bool battery_valid_ = false;
    bool battery_low_power_ = false;
    bool activation_screen_visible_ = false;
    int64_t chat_clear_at_us_ = 0;
    FaceLayer system_face_;
    FaceLayer action_face_;
    FaceLayer transient_face_;
    FaceLayer emotion_face_;
    char current_face_[32] = "neutral";
    char rendered_profile_[32] = "";
    char robot_state_[32] = "idle";
};
