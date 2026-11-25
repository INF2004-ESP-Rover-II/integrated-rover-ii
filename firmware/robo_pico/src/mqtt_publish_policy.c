/**
 * @file mqtt_publish_policy.c
 * @brief Mqtt publish policy firmware module.
 */
#include "mqtt_publish_policy.h"

void
mqtt_publish_policy_init (mqtt_publish_policy_t * p_policy)
{
    *p_policy = (mqtt_publish_policy_t) {0};
    for (unsigned index = 0; index < PUBLISH_COUNT; ++index)
    {
        p_policy->slots[index].kind = (publish_kind_t) index;
    }
}

bool
mqtt_publish_policy_begin (mqtt_publish_policy_t * p_policy,
                           publish_kind_t kind, uint32_t now_ms,
                           size_t wire_bytes, size_t free_bytes,
                           unsigned free_requests, bool b_replies_waiting)
{
    if ((unsigned) kind >= PUBLISH_COUNT)
    {
        return false;
    }
    publish_slot_t * p_slot = &p_policy->slots[kind];
    // Telemetry already has a 500 ms caller interval. New command replies may
    // arrive back-to-back; their queued retries are separately paced at 200 ms.
    if (kind != PUBLISH_STATUS && kind != PUBLISH_SENSOR)
    {
        if (p_slot->attempted && (uint32_t) (now_ms - p_slot->last_attempt_ms) <
                                     MQTT_REPORT_RETRY_MS)
        {
            return false;
        }
        p_slot->attempted       = true;
        p_slot->last_attempt_ms = now_ms;
    }
    if (p_slot->pending >= (kind == PUBLISH_STATUS ? 4u : 1u))
    {
        return false;
    }
    bool b_background = kind != PUBLISH_STATUS && kind != PUBLISH_SENSOR;
    if (b_background && b_replies_waiting)
    {
        return false;
    }
    unsigned reserve_requests = b_background ? 3u : 1u;

    size_t reserve_bytes = b_background ? MQTT_BACKGROUND_RESERVE_BYTES
                           : kind == PUBLISH_STATUS
                               ? MQTT_TELEMETRY_RESERVE_BYTES
                               : MQTT_REPLY_RESERVE_BYTES;

    if (free_requests <= reserve_requests || free_bytes < reserve_bytes ||
        wire_bytes > free_bytes - reserve_bytes)
    {
        return false;
    }
    p_slot->pending++;
    return true;
}

void
mqtt_publish_policy_release (publish_slot_t * p_slot)
{
    if (p_slot->pending)
    {
        p_slot->pending--;
    }
}

/* End of file. */
