#include "camera_protocol.h"

#include <string.h>

static uint16_t read_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void discard_prefix(camera_parser_t *parser, size_t count)
{
    parser->used -= count;
    memmove(parser->bytes, parser->bytes + count, parser->used);
}

void camera_parser_init(camera_parser_t *parser)
{
    memset(parser, 0, sizeof(*parser));
}

void camera_parser_reset(camera_parser_t *parser)
{
    parser->used = 0;
    parser->has_byte = false;
}

bool camera_parser_feed(camera_parser_t *parser, uint8_t byte,
                        uint32_t receive_ms, camera_frame_t *frame)
{
    if (parser->used && parser->has_byte &&
        (uint32_t)(receive_ms - parser->last_byte_ms) > CAMERA_BYTE_GAP_MS) {
        parser->used = 0;
        parser->gap_resets++;
    }
    parser->last_byte_ms = receive_ms;
    parser->has_byte = true;

    // A candidate is consumed/rejected at its length, at most VISION_FRAME_MAX.
    parser->bytes[parser->used++] = byte;
    while (parser->used) {
        if (parser->bytes[0] != VISION_SYNC) {
            discard_prefix(parser, 1);
            continue;
        }
        if (parser->used < 6) return false;

        const uint8_t *data = parser->bytes;
        uint8_t status = data[2], count = data[5];
        if (status > VISION_STATUS_CAMERA_LOST || count > VISION_MAX_MARKERS ||
            ((status == VISION_STATUS_OK) != (count > 0))) {
            parser->format_errors++;
            discard_prefix(parser, 1);
            continue;
        }
        size_t length = 7u + 6u * count;
        if (parser->used < length) return false;
        if (vision_crc8(data + 1, length - 2) != data[length - 1]) {
            parser->crc_errors++;
            // Keep possible sync bytes inside a corrupt/truncated candidate.
            discard_prefix(parser, 1);
            continue;
        }
        bool fields_ok = true;
        for (uint8_t i = 0; i < count; ++i) {
            const uint8_t *marker = data + 6 + 6 * i;
            if (marker[0] < 1 || marker[0] > 36 || marker[5] > 4) fields_ok = false;
        }
        if (!fields_ok) {
            parser->format_errors++;
            discard_prefix(parser, 1);
            continue;
        }

        memset(frame, 0, sizeof(*frame));
        frame->seq = data[1];
        frame->status = status;
        frame->age_ms = read_u16(data + 3);
        frame->count = count;
        for (uint8_t i = 0; i < count; ++i) {
            const uint8_t *marker = data + 6 + 6 * i;
            uint16_t bearing = read_u16(marker + 3);
            frame->markers[i].id = marker[0];
            frame->markers[i].range_mm = read_u16(marker + 1);
            frame->markers[i].bearing_cdeg = (int16_t)(bearing < 0x8000u ?
                (int32_t)bearing : (int32_t)bearing - 65536);
            frame->markers[i].quality = marker[5];
        }
        discard_prefix(parser, length);
        parser->valid_frames++;
        return true;
    }
    return false;
}

const char *camera_status_name(uint8_t status)
{
    switch (status) {
    case VISION_STATUS_OK: return "OK";
    case VISION_STATUS_NO_MARKER: return "NO_MARKER";
    case VISION_STATUS_STALLED: return "STALLED";
    case VISION_STATUS_CAMERA_LOST: return "CAMERA_LOST";
    default: return "UNKNOWN";
    }
}

const char *camera_quality_name(uint8_t quality)
{
    switch (quality) {
    case 0: return "OK";
    case 1: return "POSE_FAILED";
    case 2: return "BAD_SHAPE";
    case 3: return "TOO_SMALL";
    case 4: return "TOO_FAR";
    default: return "UNKNOWN";
    }
}
