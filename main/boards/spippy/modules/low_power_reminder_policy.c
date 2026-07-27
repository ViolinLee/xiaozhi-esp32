#include "low_power_reminder_policy.h"

#include <stddef.h>
#include <string.h>

void spippy_low_power_reminder_init(spippy_low_power_reminder_policy_t *policy,
                                    uint32_t conversation_cooldown_ms)
{
    if (policy == NULL) {
        return;
    }
    memset(policy, 0, sizeof(*policy));
    policy->conversation_cooldown_ms = conversation_cooldown_ms;
}

bool spippy_low_power_reminder_set_active(spippy_low_power_reminder_policy_t *policy,
                                          bool active)
{
    if (policy == NULL || policy->active == active) {
        return false;
    }

    policy->active = active;
    policy->entry_reminder_pending = active;
    policy->conversation_reminder_pending = false;
    policy->conversation_reminded = false;
    policy->last_conversation_reminder_ms = 0;
    return true;
}

bool spippy_low_power_reminder_on_user_turn(spippy_low_power_reminder_policy_t *policy,
                                            uint32_t now_ms)
{
    if (policy == NULL || !policy->active || policy->conversation_reminder_pending) {
        return false;
    }

    const bool first_conversation = !policy->conversation_reminded;
    const bool cooldown_elapsed = policy->conversation_reminded &&
        (uint32_t)(now_ms - policy->last_conversation_reminder_ms) >=
            policy->conversation_cooldown_ms;
    if (!first_conversation && !cooldown_elapsed) {
        return false;
    }

    policy->conversation_reminder_pending = true;
    return true;
}

bool spippy_low_power_reminder_take_entry(spippy_low_power_reminder_policy_t *policy)
{
    if (policy == NULL || !policy->active || !policy->entry_reminder_pending) {
        return false;
    }
    policy->entry_reminder_pending = false;
    return true;
}

void spippy_low_power_reminder_requeue_entry(
    spippy_low_power_reminder_policy_t *policy)
{
    if (policy == NULL || !policy->active || policy->conversation_reminded ||
        policy->conversation_reminder_pending) {
        return;
    }
    policy->entry_reminder_pending = true;
}

bool spippy_low_power_reminder_take_conversation(spippy_low_power_reminder_policy_t *policy,
                                                 uint32_t now_ms)
{
    if (policy == NULL || !policy->active || !policy->conversation_reminder_pending) {
        return false;
    }
    policy->conversation_reminder_pending = false;
    policy->conversation_reminded = true;
    policy->last_conversation_reminder_ms = now_ms;
    return true;
}

void spippy_low_power_reminder_mark_explicit_response(
    spippy_low_power_reminder_policy_t *policy,
    uint32_t now_ms)
{
    if (policy == NULL || !policy->active) {
        return;
    }
    policy->entry_reminder_pending = false;
    policy->conversation_reminder_pending = false;
    policy->conversation_reminded = true;
    policy->last_conversation_reminder_ms = now_ms;
}
