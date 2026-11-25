/**
 * @file camera_protocol.c
 * @brief Camera protocol firmware module.
 */
#include "camera_protocol.h"

#include <string.h>

static uint16_t
read_u16 (const uint8_t * p_pointer)
{
    return (uint16_t) ((uint32_t) ((uint16_t) p_pointer[0]) |
                       (uint32_t) (((uint32_t) ((uint16_t) p_pointer[1])
                                    << (uint32_t) (8))));
}

static void
discard_prefix (camera_parser_t * p_parser, size_t count)
{
    p_parser->used -= count;
    memmove(p_parser->bytes, p_parser->bytes + count, p_parser->used);
}

void
camera_parser_init (camera_parser_t * p_parser)
{
    memset(p_parser, 0, sizeof(*p_parser));
}

void
camera_parser_reset (camera_parser_t * p_parser)
{
    p_parser->used     = 0;
    p_parser->has_byte = false;
}

bool
camera_parser_feed (camera_parser_t * p_parser, uint8_t byte,
                    uint32_t receive_ms, camera_frame_t * p_frame)
{
    if (p_parser->used && p_parser->has_byte &&
        (uint32_t) (receive_ms - p_parser->last_byte_ms) > CAMERA_BYTE_GAP_MS)
    {
        p_parser->used = 0;
        p_parser->gap_resets++;
    }
    p_parser->last_byte_ms = receive_ms;
    p_parser->has_byte     = true;

    // A candidate is consumed/rejected at its length, at most VISION_FRAME_MAX.
    p_parser->bytes[p_parser->used++] = byte;
    while (p_parser->used)
    {
        if (p_parser->bytes[0] != VISION_SYNC)
        {
            discard_prefix(p_parser, 1);
            continue;
        }
        if (p_parser->used < 6)
        {
            return false;
        }

        const uint8_t * p_data = p_parser->bytes;
        uint8_t         status = p_data[2];
        uint8_t         count  = p_data[5];
        if (status > VISION_STATUS_CAMERA_LOST || count > VISION_MAX_MARKERS ||
            ((status == VISION_STATUS_OK) != (count > 0)))
        {
            p_parser->format_errors++;
            discard_prefix(p_parser, 1);
            continue;
        }
        size_t length = 7u + 6u * count;
        if (p_parser->used < length)
        {
            return false;
        }
        if (vision_crc8(p_data + 1, length - 2) != p_data[length - 1])
        {
            p_parser->crc_errors++;
            // Keep possible sync bytes inside a corrupt/truncated candidate.
            discard_prefix(p_parser, 1);
            continue;
        }
        bool b_fields_ok = true;
        for (uint8_t index = 0; index < count; ++index)
        {
            const uint8_t * p_marker = p_data + 6 + 6 * index;
            if (p_marker[0] < ROVER_MARKER_ID_MIN || p_marker[5] > 4)
            {
                b_fields_ok = false;
            }
        }
        if (!b_fields_ok)
        {
            p_parser->format_errors++;
            discard_prefix(p_parser, 1);
            continue;
        }

        memset(p_frame, 0, sizeof(*p_frame));
        p_frame->seq    = p_data[1];
        p_frame->status = status;
        p_frame->age_ms = read_u16(p_data + 3);
        p_frame->count  = count;
        for (uint8_t index = 0; index < count; ++index)
        {
            const uint8_t * p_marker         = p_data + 6 + 6 * index;
            uint16_t        bearing          = read_u16(p_marker + 3);
            p_frame->markers[index].id       = p_marker[0];
            p_frame->markers[index].range_mm = read_u16(p_marker + 1);

            p_frame->markers[index].bearing_cdeg =
                (int16_t) (bearing < 0x8000u ? (int32_t) bearing
                                             : (int32_t) bearing - 65536);

            p_frame->markers[index].quality = p_marker[5];
        }
        discard_prefix(p_parser, length);
        p_parser->valid_frames++;
        return true;
    }
    return false;
}

const char *
camera_status_name (uint8_t status)
{
    switch (status)
    {
        case VISION_STATUS_OK:
            return "OK";
        case VISION_STATUS_NO_MARKER:
            return "NO_MARKER";
        case VISION_STATUS_STALLED:
            return "STALLED";
        case VISION_STATUS_CAMERA_LOST:
            return "CAMERA_LOST";
        default:
            return "UNKNOWN";
    }
}

const char *
camera_quality_name (uint8_t quality)
{
    switch (quality)
    {
        case 0:
            return "OK";
        case 1:
            return "POSE_FAILED";
        case 2:
            return "BAD_SHAPE";
        case 3:
            return "TOO_SMALL";
        case 4:
            return "TOO_FAR";
        default:
            return "UNKNOWN";
    }
}

/* End of file. */
