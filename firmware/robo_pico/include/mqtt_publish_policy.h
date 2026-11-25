/**
 * @file mqtt_publish_policy.h
 * @brief Mqtt publish policy firmware module.
 */
#ifndef ROVER_MQTT_PUBLISH_POLICY_H
#define ROVER_MQTT_PUBLISH_POLICY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MQTT_REPORT_RETRY_MS 200u
#define MQTT_BACKGROUND_RESERVE_BYTES 1280u
#define MQTT_TELEMETRY_RESERVE_BYTES 1024u
#define MQTT_REPLY_RESERVE_BYTES 256u
#define MQTT_REPLY_QUEUE_CAPACITY 8u
#define MQTT_REPLY_CAPACITY 256u

typedef enum
{
    PUBLISH_STATUS,
    PUBLISH_SENSOR,
    PUBLISH_CAMERA,
    PUBLISH_MAP,
    PUBLISH_MISSION,
    PUBLISH_DFS,
    PUBLISH_TRACKING,
    PUBLISH_TARGET,
    PUBLISH_COUNT
} publish_kind_t;

typedef struct
{
    publish_kind_t kind;
    unsigned       pending;
    bool           attempted;
    uint32_t       last_attempt_ms;
} publish_slot_t;

typedef struct
{
    publish_slot_t slots[PUBLISH_COUNT];
} mqtt_publish_policy_t;

void mqtt_publish_policy_init (mqtt_publish_policy_t * p_policy);

// Reserve before calling lwIP. On immediate failure or completion, release it.
// Free resources must come from lwIP itself, including subscription requests.

bool mqtt_publish_policy_begin (mqtt_publish_policy_t * p_policy,
                                publish_kind_t kind, uint32_t now_ms,
                                size_t wire_bytes, size_t free_bytes,
                                unsigned free_requests, bool b_replies_waiting);

void mqtt_publish_policy_release (publish_slot_t * p_slot);

#endif

/* End of file. */
