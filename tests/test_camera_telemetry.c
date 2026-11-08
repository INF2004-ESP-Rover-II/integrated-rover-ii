#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "camera_telemetry.h"

int main(void)
{
    camera_frame_t frame = {.seq = 194, .status = VISION_STATUS_OK, .age_ms = 200, .count = 4,
        .markers = {{1, 484, 82, 0}, {36, 65535, -32768, 4}, {7, 0, 0, 1}, {9, 700, 32767, 0}}};
    char payload[768];
    size_t length = camera_telemetry_format(payload, sizeof(payload), &frame, 100, 120);
    assert(length == strlen(payload));
    assert(strstr(payload, "\"age_ms\":220") && strstr(payload, "\"bearing_cdeg\":-32768"));
    puts(payload); // Python validation also parses these actual C outputs as JSON.
    length = camera_telemetry_format(payload, sizeof(payload), &frame, 100, 1101);
    assert(length && strstr(payload, "UART_TIMEOUT") && strstr(payload, "\"markers\":[]"));
    puts(payload);
    frame.count = 0;
    for (unsigned status = VISION_STATUS_NO_MARKER; status <= VISION_STATUS_CAMERA_LOST; ++status) {
        frame.status = (uint8_t)status;
        assert(camera_telemetry_format(payload, sizeof(payload), &frame, UINT32_MAX - 10u, 10u));
        assert(strstr(payload, "\"uart_age_ms\":21"));
        puts(payload);
    }
    assert(!camera_telemetry_format(payload, 10, &frame, 1, 2));
    assert(payload[0] == '\0');
    assert(!camera_telemetry_format(NULL, 0, &frame, 1, 2));
    frame.count = 5;
    assert(!camera_telemetry_format(payload, sizeof(payload), &frame, 1, 2));
    puts("Camera telemetry tests passed");
    return 0;
}
