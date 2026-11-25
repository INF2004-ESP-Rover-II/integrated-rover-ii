#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mqtt_publish_policy.h"
#include "rover_map.h"

int main(void)
{
    mqtt_publish_policy_t policy;
    mqtt_publish_policy_init(&policy);
    // A stalled connection may hold each routine report once, never 13 statuses.
    for (unsigned kind = PUBLISH_MISSION; kind < PUBLISH_COUNT; ++kind) {
        assert(mqtt_publish_policy_begin(&policy, (publish_kind_t)kind, 1000, 800, 8192, 16, false));
        for (uint32_t now = 1001; now < 5000; ++now)
            assert(!mqtt_publish_policy_begin(&policy, (publish_kind_t)kind, now, 800, 8192, 16, false));
        assert(policy.slots[kind].pending == 1);
    }
    // Replies and telemetry are independent of those four pending reports.
    assert(mqtt_publish_policy_begin(&policy, PUBLISH_SENSOR, 5000, 800, 2000, 4, false));
    for (unsigned i = 0; i < 4; ++i)
        assert(mqtt_publish_policy_begin(&policy, PUBLISH_STATUS, 5000, 100, 2000, 4, false));
    assert(!mqtt_publish_policy_begin(&policy, PUBLISH_STATUS, 5000, 100, 2000, 4, false));

    mqtt_publish_policy_init(&policy);
    // Admission preserves real byte/request headroom, including subscriptions.
    assert(!mqtt_publish_policy_begin(&policy, PUBLISH_MAP, 0, 500, 8192, 3, false));
    assert(!mqtt_publish_policy_begin(&policy, PUBLISH_MAP, 199, 500, 8192, 16, false));
    assert(!mqtt_publish_policy_begin(&policy, PUBLISH_MAP, 200, 500, 500 + MQTT_BACKGROUND_RESERVE_BYTES - 1, 16, false));
    assert(mqtt_publish_policy_begin(&policy, PUBLISH_MAP, 400, 500, 500 + MQTT_BACKGROUND_RESERVE_BYTES, 4, false));
    mqtt_publish_policy_release(&policy.slots[PUBLISH_MAP]);
    assert(!mqtt_publish_policy_begin(&policy, PUBLISH_MAP, 401, 500, 8192, 16, false));
    assert(mqtt_publish_policy_begin(&policy, PUBLISH_MAP, 600, 500, 8192, 16, false));
    assert(!mqtt_publish_policy_begin(&policy, PUBLISH_SENSOR, 1000, 800, 800 + MQTT_REPLY_RESERVE_BYTES - 1, 16, false));
    assert(mqtt_publish_policy_begin(&policy, PUBLISH_SENSOR, 1000, 800, 800 + MQTT_REPLY_RESERVE_BYTES, 2, false));
    assert(!mqtt_publish_policy_begin(&policy, PUBLISH_STATUS, 1000, 100, 100 + MQTT_TELEMETRY_RESERVE_BYTES - 1, 16, false));
    assert(mqtt_publish_policy_begin(&policy, PUBLISH_STATUS, 1000, 100, 100 + MQTT_TELEMETRY_RESERVE_BYTES, 2, false));

    mqtt_publish_policy_init(&policy);
    assert(!mqtt_publish_policy_begin(&policy, PUBLISH_TARGET, UINT32_MAX - 100u, 100, 8192, 16, true));
    assert(!mqtt_publish_policy_begin(&policy, PUBLISH_TARGET, 98, 100, 8192, 16, false));
    assert(mqtt_publish_policy_begin(&policy, PUBLISH_TARGET, 99, 100, 8192, 16, false));
    mqtt_publish_policy_release(&policy.slots[PUBLISH_TARGET]);
    mqtt_publish_policy_release(&policy.slots[PUBLISH_TARGET]);
    assert(!policy.slots[PUBLISH_TARGET].pending);
    assert(!mqtt_publish_policy_begin(&policy, PUBLISH_COUNT, 0, 100, 8192, 16, false));

    // The byte reservation must still allow a full map with extreme coordinates.
    rover_map_t map;
    rover_map_init(&map, UINT64_MAX);
    for (unsigned i = 0; i < ROVER_MAP_CAPACITY; ++i) {
        assert(rover_map_add(&map, (uint8_t)(255u - i), INT16_MIN + (int16_t)i, INT16_MIN, false) == MAP_OK);
        for (unsigned dir = 0; dir < 4; ++dir)
            assert(rover_map_set_exit(&map, (uint8_t)(255u-i), (map_direction_t)dir, MAP_EXIT_BLOCKED) == MAP_OK);
    }
    char payload[ROVER_MAP_JSON_CAPACITY];
    size_t length = rover_map_format(&map, "0123456789abcdef0123456789abcdef", payload, sizeof(payload));
    assert(length && length + strlen("robot/map") + 7u + MQTT_BACKGROUND_RESERVE_BYTES <= 8192u);
    assert(mqtt_publish_policy_begin(&policy, PUBLISH_MAP, 1000, length + strlen("robot/map") + 7u, 8192, 16, false));
    printf("MQTT policy tests passed; maximum-coordinate full map=%u bytes\n", (unsigned)length);
    return 0;
}
