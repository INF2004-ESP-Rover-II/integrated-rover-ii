#include "vision_frame.h"

#include <math.h>

uint8_t vision_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

// A NaN becomes 0: the cast of one to an integer is undefined.
static long round_capped(float v, long lo, long hi)
{
    if (isnan(v)) return 0;
    if (v <= (float)lo) return lo;
    if (v >= (float)hi) return hi;
    return lroundf(v);
}

static uint8_t *put_u16(uint8_t *p, uint16_t v)
{
    *p++ = (uint8_t)v;
    *p++ = (uint8_t)(v >> 8);
    return p;
}

size_t vision_frame_build(uint8_t *out, uint8_t seq, uint8_t status, int64_t age_ms,
                          const vision_marker_t *markers, int n)
{
    if (n < 0) n = 0;
    if (n > VISION_MAX_MARKERS) n = VISION_MAX_MARKERS;
    if (age_ms < 0) age_ms = 0;
    if (age_ms > 65535) age_ms = 65535;

    uint8_t *p = out;
    *p++ = VISION_SYNC;
    *p++ = seq;
    *p++ = status;
    p = put_u16(p, (uint16_t)age_ms);
    *p++ = (uint8_t)n;
    for (int i = 0; i < n; i++) {
        *p++ = markers[i].id;
        p = put_u16(p, (uint16_t)round_capped(markers[i].range_mm, 0, 65535));
        p = put_u16(p, (uint16_t)(int16_t)round_capped(markers[i].bearing_deg * 100.0f, -32768, 32767));
        *p++ = markers[i].quality;
    }
    size_t covered = (size_t)(p - (out + 1));
    uint8_t crc = vision_crc8(out + 1, covered);
    *p++ = crc;
    return (size_t)(p - out);
}
