#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SPIPPY_FORWARD_GUARD_UNKNOWN = 0,
    SPIPPY_FORWARD_GUARD_CLEAR,
    SPIPPY_FORWARD_GUARD_BLOCKED,
} spippy_forward_guard_state_t;

typedef enum {
    SPIPPY_FORWARD_GUARD_REASON_STARTUP = 0,
    SPIPPY_FORWARD_GUARD_REASON_VALID_CLEAR,
    SPIPPY_FORWARD_GUARD_REASON_OBSTACLE_NEAR,
    SPIPPY_FORWARD_GUARD_REASON_OBSTACLE_CRITICAL,
    SPIPPY_FORWARD_GUARD_REASON_NO_ECHO_CLEAR,
    SPIPPY_FORWARD_GUARD_REASON_SENSOR_ERROR,
    SPIPPY_FORWARD_GUARD_REASON_STALE,
} spippy_forward_guard_reason_t;

typedef enum {
    SPIPPY_FORWARD_GUARD_SAMPLE_VALID = 0,
    /* 超声波等待到上限仍无回波：通常表示量程内没有可反射目标。 */
    SPIPPY_FORWARD_GUARD_SAMPLE_NO_ECHO,
    /* 驱动、捕获或脉宽异常；不能把它当成“前方安全”。 */
    SPIPPY_FORWARD_GUARD_SAMPLE_ERROR,
} spippy_forward_guard_sample_kind_t;

typedef struct {
    uint32_t block_distance_mm;
    uint32_t critical_distance_mm;
    uint32_t release_distance_mm;
    uint32_t clear_confirm_count;
    uint32_t no_echo_clear_count;
    uint32_t error_to_unknown_count;
    uint32_t stale_timeout_ms;
} spippy_forward_guard_config_t;

typedef struct {
    spippy_forward_guard_config_t config;
    spippy_forward_guard_state_t state;
    spippy_forward_guard_reason_t reason;
    spippy_forward_guard_sample_kind_t last_sample_kind;
    uint32_t last_distance_mm;
    uint32_t last_sample_id;
    uint32_t last_sample_ms;
    uint32_t last_valid_ms;
    uint32_t clear_streak;
    uint32_t no_echo_streak;
    uint32_t error_streak;
    bool has_sample;
    bool has_valid_sample;
} spippy_forward_guard_t;

void spippy_forward_guard_init(spippy_forward_guard_t *guard,
                               const spippy_forward_guard_config_t *config);

/*
 * sample_id 必须随超声波采样递增。重复 sample_id 会被忽略，避免两个任务
 * 恰好读到同一个样本时把一次测量误算成多次确认。
 */
bool spippy_forward_guard_update(spippy_forward_guard_t *guard,
                                 spippy_forward_guard_sample_kind_t kind,
                                 uint32_t distance_mm,
                                 uint32_t sample_id,
                                 uint32_t now_ms);

/* 没有新样本时检查采样任务是否已经失去更新。 */
bool spippy_forward_guard_tick(spippy_forward_guard_t *guard, uint32_t now_ms);

bool spippy_forward_guard_can_move(const spippy_forward_guard_t *guard);
const char *spippy_forward_guard_state_name(spippy_forward_guard_state_t state);
const char *spippy_forward_guard_reason_name(spippy_forward_guard_reason_t reason);

#ifdef __cplusplus
}
#endif
