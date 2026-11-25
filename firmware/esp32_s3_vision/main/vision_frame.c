/**
 * @file vision_frame.c
 * @brief Vision frame firmware module.
 */
#include "vision_frame.h"

#include <math.h>

/* Fixed dimensions and iteration limits used by this module. */
#define CRC_BIT_COUNT 8

uint8_t
vision_crc8 (const uint8_t * p_data, size_t len)
{
    uint8_t crc = 0;
    for (size_t index = 0; index < len; index++)
    {
        crc ^= (uint32_t) (p_data[index]);
        for (int bit = 0; bit < CRC_BIT_COUNT; bit++)
        {

            crc = ((uint32_t) (crc) & (uint32_t) (0x80))
                      ? (uint8_t) ((uint32_t) (((uint32_t) (crc)
                                                << (uint32_t) (1))) ^
                                   (uint32_t) (0x07))
                      : (uint8_t) ((uint32_t) (crc) << (uint32_t) (1));
        }
    }
    return crc;
}

// A NaN becomes 0: the cast of one to an integer is undefined.
static int32_t
round_capped (float value, int32_t lower_bound, int32_t upper_bound)
{
    if (isnan(value))
    {
        return 0;
    }
    if (value <= (float) lower_bound)
    {
        return lower_bound;
    }
    if (value >= (float) upper_bound)
    {
        return upper_bound;
    }
    return lroundf(value);
}

static uint8_t *
put_u16 (uint8_t * p_pointer, uint16_t value)
{
    *p_pointer++ = (uint8_t) value;
    *p_pointer++ = (uint8_t) ((uint32_t) (value) >> (uint32_t) (8));
    return p_pointer;
}

size_t
vision_frame_build (uint8_t * p_out, uint8_t seq, uint8_t status,
                    int64_t age_ms, const vision_marker_t * p_markers,
                    int count)
{
    if (count < 0)
    {
        count = 0;
    }
    if (count > VISION_MAX_MARKERS)
    {
        count = VISION_MAX_MARKERS;
    }
    if (age_ms < 0)
    {
        age_ms = 0;
    }
    if (age_ms > 65535)
    {
        age_ms = 65535;
    }

    uint8_t * p_pointer = p_out;
    *p_pointer++        = VISION_SYNC;
    *p_pointer++        = seq;
    *p_pointer++        = status;
    p_pointer           = put_u16(p_pointer, (uint16_t) age_ms);
    *p_pointer++        = (uint8_t) count;
    for (int index = 0; index < count; index++)
    {
        *p_pointer++ = p_markers[index].id;

        p_pointer = put_u16(
            p_pointer,
            (uint16_t) round_capped(p_markers[index].range_mm, 0, 65535));

        p_pointer =
            put_u16(p_pointer,
                    (uint16_t) (int16_t) round_capped(
                        p_markers[index].bearing_deg * 100.0f, -32768, 32767));

        *p_pointer++ = p_markers[index].quality;
    }
    size_t  covered = (size_t) (p_pointer - (p_out + 1));
    uint8_t crc     = vision_crc8(p_out + 1, covered);
    *p_pointer++    = crc;
    return (size_t) (p_pointer - p_out);
}

/* End of file. */
