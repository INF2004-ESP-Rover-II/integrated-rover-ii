#include "vision_protocol.h"

#include <limits.h>
#include <string.h>

uint8_t vision_crc8(const uint8_t *bytes, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (uint8_t)((crc & 0x80u) ? ((unsigned)crc << 1u) ^ 0x07u
                                         : (unsigned)crc << 1u);
        }
    }
    return crc;
}

void vision_parser_init(vision_parser_t *parser) {
    memset(parser, 0, sizeof(*parser));
}

void vision_parser_reset(vision_parser_t *parser) {
    parser->used = 0;
}

static void discard(vision_parser_t *parser, size_t count) {
    parser->used -= count;
    memmove(parser->bytes, parser->bytes + count, parser->used);
    memmove(parser->byte_received_ms, parser->byte_received_ms + count,
            parser->used * sizeof(parser->byte_received_ms[0]));
}

static uint16_t le16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8u));
}

static bool decode(const uint8_t *p, vision_frame_t *out) {
    vision_frame_t frame = {0};
    frame.sequence = p[1];
    frame.status = (vision_status_t)p[2];
    frame.age_ms = le16(p + 3);
    frame.count = p[5];
    for (unsigned i = 0; i < frame.count; ++i) {
        const uint8_t *m = p + 6u + VISION_MARKER_BYTES * i;
        vision_marker_t *marker = &frame.markers[i];
        marker->id = m[0];
        marker->range_mm = le16(m + 1);
        uint16_t raw = le16(m + 3);
        /* Avoid relying on implementation-defined unsigned-to-signed casts. */
        int32_t bearing = (raw & 0x8000u) ? (int32_t)raw - 65536 : (int32_t)raw;
        marker->bearing_cdeg = (int16_t)bearing;
        marker->quality = m[5];
        if (marker->id < 1u || (marker->range_mm == 0u && marker->quality == VISION_QUALITY_OK) ||
            bearing < -18000 || bearing > 18000) return false;
        for (unsigned j = 0; j < i; ++j) {
            if (frame.markers[j].id == marker->id) return false;
        }
    }
    *out = frame;
    return true;
}

static bool drain(vision_parser_t *parser, vision_frame_t *out) {
    bool emitted = false;
    while (parser->used != 0u) {
        const uint8_t *p = parser->bytes;
        if (p[0] != VISION_SYNC) {
            discard(parser, 1);
            continue;
        }
        if (parser->used < 6u) break;
        uint8_t status = p[2], count = p[5];
        if (status > VISION_CAMERA_LOST || count > VISION_MAX_MARKERS ||
            ((status == VISION_OK) != (count != 0u))) {
            ++parser->stats.format_errors;
            discard(parser, 1);
            continue;
        }
        size_t length = VISION_FRAME_BASE_BYTES + VISION_MARKER_BYTES * (size_t)count;
        if (parser->used < length) break;
        if (vision_crc8(p + 1, length - 2u) != p[length - 1u]) {
            ++parser->stats.crc_errors;
            discard(parser, 1);
            continue;
        }
        if (decode(p, out)) {
            parser->frame_received_ms = parser->byte_received_ms[length - 1u];
            ++parser->stats.frames_valid;
            emitted = true;
        } else {
            ++parser->stats.format_errors;
        }
        discard(parser, length);
    }
    return emitted;
}

bool vision_parser_poll(vision_parser_t *parser, uint32_t now_ms,
                        vision_frame_t *out) {
    bool emitted = false;
    if (parser->used != 0u &&
        (uint32_t)(now_ms - parser->last_byte_ms) >= VISION_INTERBYTE_TIMEOUT_MS) {
        ++parser->stats.timeouts;
        /* An incomplete false header can hide an entire later valid packet.
         * Drop the false candidate, then salvage complete suffix packets. */
        while (parser->used != 0u) {
            discard(parser, 1);
            if (drain(parser, out)) emitted = true;
        }
    }
    return emitted;
}

bool vision_parser_feed(vision_parser_t *parser, uint8_t byte,
                        uint32_t now_ms, vision_frame_t *out) {
    bool emitted = vision_parser_poll(parser, now_ms, out);
    ++parser->stats.bytes_received;
    parser->last_byte_ms = now_ms;
    if (parser->used == 0u && byte != VISION_SYNC) return emitted;
    /* drain() consumes every complete candidate at <=31 bytes. */
    if (parser->used >= sizeof(parser->bytes)) {
        ++parser->stats.format_errors;
        discard(parser, 1);
    }
    parser->byte_received_ms[parser->used] = now_ms;
    parser->bytes[parser->used++] = byte;
    if (drain(parser, out)) emitted = true;
    return emitted;
}

bool vision_frame_is_image(const vision_frame_t *frame) {
    return frame->status == VISION_OK || frame->status == VISION_NO_MARKER;
}

uint32_t vision_effective_age_ms(const vision_frame_t *frame,
                                uint32_t received_ms, uint32_t now_ms) {
    uint32_t bytes = VISION_FRAME_BASE_BYTES + VISION_MARKER_BYTES * frame->count;
    uint32_t wire_ms = (bytes * 10000u + VISION_BAUD - 1u) / VISION_BAUD;
    uint32_t fixed = (uint32_t)frame->age_ms + wire_ms;
    uint32_t elapsed = now_ms - received_ms;
    return elapsed > UINT32_MAX - fixed ? UINT32_MAX : elapsed + fixed;
}
