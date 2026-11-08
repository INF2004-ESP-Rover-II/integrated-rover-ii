#include "camera_telemetry.h"

#include <stdarg.h>
#include <stdio.h>

static bool append(char *out, size_t capacity, size_t *used, const char *format, ...)
{
    if (*used >= capacity) return false;
    va_list args;
    va_start(args, format);
    int length = vsnprintf(out + *used, capacity - *used, format, args);
    va_end(args);
    if (length < 0 || (size_t)length >= capacity - *used) return false;
    *used += (size_t)length;
    return true;
}

size_t camera_telemetry_format(char *out, size_t capacity, const camera_frame_t *frame,
                               uint32_t received_ms, uint32_t now_ms)
{
    if (!out || !capacity) return 0;
    out[0] = '\0';
    if (!frame || frame->count > VISION_MAX_MARKERS || frame->status > VISION_STATUS_CAMERA_LOST)
        return 0;
    uint32_t uart_age = now_ms - received_ms;
    uint64_t total_age = (uint64_t)frame->age_ms + uart_age;
    uint32_t age = total_age > UINT32_MAX ? UINT32_MAX : (uint32_t)total_age;
    bool expired = uart_age > CAMERA_UART_TIMEOUT_MS;
    const char *status = expired ? "UART_TIMEOUT" : camera_status_name(frame->status);
    uint8_t count = !expired && frame->status == VISION_STATUS_OK ? frame->count : 0;
    size_t used = 0;
    if (!append(out, capacity, &used,
        "{\"schema\":1,\"source\":\"robo_pico_uart\",\"seq\":%u,\"status\":\"%s\","
        "\"age_ms\":%lu,\"uart_age_ms\":%lu,\"markers\":[",
        (unsigned)frame->seq, status, (unsigned long)age, (unsigned long)uart_age)) goto full;
    for (uint8_t i = 0; i < count; ++i) {
        const camera_marker_t *marker = &frame->markers[i];
        if (!append(out, capacity, &used,
            "%s{\"id\":%u,\"range_mm\":%u,\"bearing_cdeg\":%d,\"quality\":%u}",
            i ? "," : "", (unsigned)marker->id, (unsigned)marker->range_mm,
            (int)marker->bearing_cdeg, (unsigned)marker->quality)) goto full;
    }
    if (!append(out, capacity, &used, "]}")) goto full;
    return used;
full:
    out[0] = '\0';
    return 0;
}
