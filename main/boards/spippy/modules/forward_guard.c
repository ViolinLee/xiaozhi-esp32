#include "forward_guard.h"

#include <stddef.h>
#include <string.h>

static uint32_t saturating_increment(uint32_t value)
{
    return value == UINT32_MAX ? value : value + 1U;
}

static bool set_decision(spippy_forward_guard_t *guard,
                         spippy_forward_guard_state_t state,
                         spippy_forward_guard_reason_t reason)
{
    bool changed = guard->state != state || guard->reason != reason;
    guard->state = state;
    guard->reason = reason;
    return changed;
}

void spippy_forward_guard_init(spippy_forward_guard_t *guard,
                               const spippy_forward_guard_config_t *config)
{
    if (guard == NULL || config == NULL) {
        return;
    }
    memset(guard, 0, sizeof(*guard));
    guard->config = *config;
    guard->state = SPIPPY_FORWARD_GUARD_UNKNOWN;
    guard->reason = SPIPPY_FORWARD_GUARD_REASON_STARTUP;
    guard->last_sample_kind = SPIPPY_FORWARD_GUARD_SAMPLE_ERROR;
}

static bool update_valid_sample(spippy_forward_guard_t *guard, uint32_t distance_mm, uint32_t now_ms)
{
    guard->has_valid_sample = true;
    guard->last_valid_ms = now_ms;
    guard->last_distance_mm = distance_mm;
    guard->no_echo_streak = 0;
    guard->error_streak = 0;

    /*
     * 进入阻挡采用单次有效近距样本，退出则要求连续确认。这样真实障碍
     * 可以尽快制动，而一次超时或阈值抖动都不能立即解除保护。
     */
    if (distance_mm <= guard->config.block_distance_mm) {
        guard->clear_streak = 0;
        return set_decision(
            guard,
            SPIPPY_FORWARD_GUARD_BLOCKED,
            distance_mm <= guard->config.critical_distance_mm
                ? SPIPPY_FORWARD_GUARD_REASON_OBSTACLE_CRITICAL
                : SPIPPY_FORWARD_GUARD_REASON_OBSTACLE_NEAR);
    }

    uint32_t clear_threshold = guard->state == SPIPPY_FORWARD_GUARD_BLOCKED
        ? guard->config.release_distance_mm
        : guard->config.block_distance_mm + 1U;
    if (distance_mm < clear_threshold) {
        guard->clear_streak = 0;
        return false;
    }

    guard->clear_streak = saturating_increment(guard->clear_streak);
    if (guard->clear_streak < guard->config.clear_confirm_count) {
        return false;
    }
    return set_decision(guard,
                        SPIPPY_FORWARD_GUARD_CLEAR,
                        SPIPPY_FORWARD_GUARD_REASON_VALID_CLEAR);
}

static bool update_no_echo(spippy_forward_guard_t *guard)
{
    guard->clear_streak = 0;
    guard->error_streak = 0;
    guard->no_echo_streak = saturating_increment(guard->no_echo_streak);

    /*
     * “无回波”与驱动错误不同。在开阔区域它是常态，但不能用单次无回波
     * 清除刚确认的障碍，因此仍要求连续多次确认后才判定量程内无目标。
     */
    if (guard->no_echo_streak < guard->config.no_echo_clear_count) {
        return false;
    }
    return set_decision(guard,
                        SPIPPY_FORWARD_GUARD_CLEAR,
                        SPIPPY_FORWARD_GUARD_REASON_NO_ECHO_CLEAR);
}

static bool update_sensor_error(spippy_forward_guard_t *guard)
{
    guard->clear_streak = 0;
    guard->no_echo_streak = 0;
    guard->error_streak = saturating_increment(guard->error_streak);

    /* 已确认的障碍必须锁存，传感器故障不能成为解除阻挡的理由。 */
    if (guard->state == SPIPPY_FORWARD_GUARD_BLOCKED ||
        guard->error_streak < guard->config.error_to_unknown_count) {
        return false;
    }
    return set_decision(guard,
                        SPIPPY_FORWARD_GUARD_UNKNOWN,
                        SPIPPY_FORWARD_GUARD_REASON_SENSOR_ERROR);
}

bool spippy_forward_guard_update(spippy_forward_guard_t *guard,
                                 spippy_forward_guard_sample_kind_t kind,
                                 uint32_t distance_mm,
                                 uint32_t sample_id,
                                 uint32_t now_ms)
{
    if (guard == NULL) {
        return false;
    }
    if (guard->has_sample && guard->last_sample_id == sample_id) {
        return false;
    }

    guard->has_sample = true;
    guard->last_sample_id = sample_id;
    guard->last_sample_ms = now_ms;
    guard->last_sample_kind = kind;

    switch (kind) {
        case SPIPPY_FORWARD_GUARD_SAMPLE_VALID:
            return update_valid_sample(guard, distance_mm, now_ms);
        case SPIPPY_FORWARD_GUARD_SAMPLE_NO_ECHO:
            return update_no_echo(guard);
        case SPIPPY_FORWARD_GUARD_SAMPLE_ERROR:
        default:
            return update_sensor_error(guard);
    }
}

bool spippy_forward_guard_tick(spippy_forward_guard_t *guard, uint32_t now_ms)
{
    if (guard == NULL || !guard->has_sample) {
        return false;
    }
    /* uint32_t 无符号差值在系统运行时间回绕时仍能得到正确的短时间间隔。 */
    if ((uint32_t)(now_ms - guard->last_sample_ms) <= guard->config.stale_timeout_ms) {
        return false;
    }
    /*
     * 采样恢复后必须重新积累安全确认，不能沿用停更前的计数。尤其是
     * BLOCKED 状态下，不能把停更前的无回波次数带到恢复后提前解锁。
     */
    guard->clear_streak = 0;
    guard->no_echo_streak = 0;
    guard->error_streak = 0;

    /* 已确认的障碍继续锁存；数据陈旧只能收紧保护，不能解除保护。 */
    if (guard->state == SPIPPY_FORWARD_GUARD_BLOCKED) {
        return false;
    }
    return set_decision(guard,
                        SPIPPY_FORWARD_GUARD_UNKNOWN,
                        SPIPPY_FORWARD_GUARD_REASON_STALE);
}

bool spippy_forward_guard_can_move(const spippy_forward_guard_t *guard)
{
    return guard != NULL && guard->state == SPIPPY_FORWARD_GUARD_CLEAR;
}

const char *spippy_forward_guard_state_name(spippy_forward_guard_state_t state)
{
    switch (state) {
        case SPIPPY_FORWARD_GUARD_CLEAR: return "clear";
        case SPIPPY_FORWARD_GUARD_BLOCKED: return "blocked";
        case SPIPPY_FORWARD_GUARD_UNKNOWN:
        default: return "unknown";
    }
}

const char *spippy_forward_guard_reason_name(spippy_forward_guard_reason_t reason)
{
    switch (reason) {
        case SPIPPY_FORWARD_GUARD_REASON_VALID_CLEAR: return "valid_clear";
        case SPIPPY_FORWARD_GUARD_REASON_OBSTACLE_NEAR: return "obstacle_near";
        case SPIPPY_FORWARD_GUARD_REASON_OBSTACLE_CRITICAL: return "obstacle_critical";
        case SPIPPY_FORWARD_GUARD_REASON_NO_ECHO_CLEAR: return "no_echo_clear";
        case SPIPPY_FORWARD_GUARD_REASON_SENSOR_ERROR: return "sensor_error";
        case SPIPPY_FORWARD_GUARD_REASON_STALE: return "stale";
        case SPIPPY_FORWARD_GUARD_REASON_STARTUP:
        default: return "startup";
    }
}
