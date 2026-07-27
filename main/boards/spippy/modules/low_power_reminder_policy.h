#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t conversation_cooldown_ms;
    uint32_t last_conversation_reminder_ms;
    bool active;
    bool entry_reminder_pending;
    bool conversation_reminder_pending;
    bool conversation_reminded;
} spippy_low_power_reminder_policy_t;

void spippy_low_power_reminder_init(spippy_low_power_reminder_policy_t *policy,
                                    uint32_t conversation_cooldown_ms);

/*
 * 低电锁存的每次 false -> true 都是一个新的提醒周期；恢复后清空本周期状态。
 * 返回值表示 active 是否发生了变化。
 */
bool spippy_low_power_reminder_set_active(spippy_low_power_reminder_policy_t *policy,
                                          bool active);

/* 用户的一轮有效语音被识别后调用；返回值表示本轮是否新建了待播提醒。 */
bool spippy_low_power_reminder_on_user_turn(spippy_low_power_reminder_policy_t *policy,
                                            uint32_t now_ms);

/* 消费锁存瞬间的一次性提醒。 */
bool spippy_low_power_reminder_take_entry(spippy_low_power_reminder_policy_t *policy);

/*
 * 入低电提示排队后若恰好进入收音，可尝试把它退回策略。若本轮对话已经
 * 排有提醒，或明确的电量/拒绝回复已覆盖提醒，则不会重复挂起。
 */
void spippy_low_power_reminder_requeue_entry(
    spippy_low_power_reminder_policy_t *policy);

/*
 * 在在线 TTS 完成时消费对话提醒，并从此刻开始计算冷却时间。uint32_t
 * 无符号时间差可安全跨越系统毫秒计数回绕。
 */
bool spippy_low_power_reminder_take_conversation(spippy_low_power_reminder_policy_t *policy,
                                                 uint32_t now_ms);

/*
 * 电量查询或动作拒绝的在线回答已经包含低电原因时调用，避免回答结束后
 * 再追加一遍相同提示。它同时满足尚未播放的入低电提醒。
 */
void spippy_low_power_reminder_mark_explicit_response(
    spippy_low_power_reminder_policy_t *policy,
    uint32_t now_ms);

#ifdef __cplusplus
}
#endif
