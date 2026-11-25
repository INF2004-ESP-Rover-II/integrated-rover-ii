/**
 * @file camera_telemetry.c
 * @brief Camera telemetry firmware module.
 */
#include <inttypes.h>
#include "camera_telemetry.h"

#include <stdarg.h>
#include <stdio.h>

static bool
append (char * p_out, size_t capacity, size_t * p_used, const char * p_format,
        ...)
{
    if (*p_used >= capacity)
    {
        return false;
    }
    va_list args;
    va_start(args, p_format);
    int length = vsnprintf(p_out + *p_used, capacity - *p_used, p_format, args);
    va_end(args);
    if (length < 0 || (size_t) length >= capacity - *p_used)
    {
        return false;
    }
    *p_used += (size_t) length;
    return true;
}

size_t
camera_telemetry_format (char * p_out, size_t capacity,
                         const camera_frame_t * p_frame, uint32_t received_ms,
                         uint32_t now_ms)
{
    if (!p_out || !capacity)
    {
        return 0;
    }
    p_out[0] = '\0';
    if (!p_frame || p_frame->count > VISION_MAX_MARKERS ||
        p_frame->status > VISION_STATUS_CAMERA_LOST)
    {
        return 0;
    }
    uint32_t uart_age  = now_ms - received_ms;
    uint64_t total_age = (uint64_t) p_frame->age_ms + uart_age;
    uint32_t age = total_age > UINT32_MAX ? UINT32_MAX : (uint32_t) total_age;
    bool     b_expired = uart_age > CAMERA_UART_TIMEOUT_MS;

    const char * p_status =
        b_expired ? "UART_TIMEOUT" : camera_status_name(p_frame->status);


    uint8_t count =
        !b_expired && p_frame->status == VISION_STATUS_OK ? p_frame->count : 0;

    size_t used = 0;
    if (!append(p_out, capacity, &used,
                "{\"schema\":1,\"source\":\"robo_pico_uart\",\"seq\":%u,"
                "\"status\":\"%s\","
                "\"age_ms\":%" PRIu32 ",\"uart_age_ms\":%" PRIu32
                ",\"markers\":[",
                (unsigned) p_frame->seq, p_status, (uint32_t) age,
                (uint32_t) uart_age))
    {
        goto full;
    }
    for (uint8_t index = 0; index < count; ++index)
    {
        const camera_marker_t * p_marker = &p_frame->markers[index];
        if (!append(p_out, capacity, &used,
                    "%s{\"id\":%u,\"range_mm\":%u,\"bearing_cdeg\":%d,"
                    "\"quality\":%u}",
                    index ? "," : "", (unsigned) p_marker->id,
                    (unsigned) p_marker->range_mm, (int) p_marker->bearing_cdeg,
                    (unsigned) p_marker->quality))
        {
            goto full;
        }
    }
    if (!append(p_out, capacity, &used, "]}"))
    {
        goto full;
    }
    return used;
full:
    p_out[0] = '\0';
    return 0;
}

/* End of file. */
