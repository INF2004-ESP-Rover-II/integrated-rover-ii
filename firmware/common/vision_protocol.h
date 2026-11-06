#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Wire v1: A5 seq status age_le16 count [id range_le16 bearing_le16 quality]* CRC.
 * CRC-8/SMBUS: poly 07, init 00, refin/refout false, xorout 00; excludes A5.
 * Decode fields explicitly: sizeof(vision_frame_t) is NOT the wire size. */
#define VISION_SYNC 0xA5u
#define VISION_BAUD 115200u
#define VISION_MAX_MARKERS 4u
#define VISION_FRAME_BASE_BYTES 7u
#define VISION_MARKER_BYTES 6u
#define VISION_MAX_FRAME_BYTES (VISION_FRAME_BASE_BYTES + VISION_MARKER_BYTES * VISION_MAX_MARKERS)
#define VISION_INTERBYTE_TIMEOUT_MS 25u

typedef enum {
    VISION_OK = 0,
    VISION_NO_MARKER = 1,
    VISION_STALLED = 2,
    VISION_CAMERA_LOST = 3
} vision_status_t;

/* B3's quality reason: nonzero observations are diagnostic-only. */
typedef enum {
    VISION_QUALITY_OK = 0, VISION_POSE_FAILED = 1, VISION_BAD_SHAPE = 2,
    VISION_TOO_SMALL = 3, VISION_TOO_FAR = 4
} vision_quality_t;

typedef struct {
    uint8_t id;                 /* 1..255; zero reserved */
    uint16_t range_mm;          /* Lens range; 0 allowed only with nonzero quality. */
    int16_t bearing_cdeg;       /* -18000..18000; positive right */
    uint8_t quality;            /* 0 usable; nonzero reason/unknown is not used for motion. */
} vision_marker_t;

typedef struct {
    uint8_t sequence;
    vision_status_t status;
    uint16_t age_ms;           /* S3 receipt to TX; excludes camera/USB delay. */
    uint8_t count;
    vision_marker_t markers[VISION_MAX_MARKERS];
} vision_frame_t;

typedef struct {
    uint32_t bytes_received;
    uint32_t frames_valid;
    uint32_t crc_errors;
    uint32_t format_errors;
    uint32_t timeouts;
} vision_parser_stats_t;

typedef struct {
    uint8_t bytes[VISION_MAX_FRAME_BYTES];
    uint32_t byte_received_ms[VISION_MAX_FRAME_BYTES];
    size_t used;
    uint32_t last_byte_ms;
    uint32_t frame_received_ms; /* End of last emitted packet, even after resync. */
    vision_parser_stats_t stats;
} vision_parser_t;

uint8_t vision_crc8(const uint8_t *bytes, size_t length);
void vision_parser_init(vision_parser_t *parser);
/* Discard incomplete framing after a UART hardware error; retain statistics. */
void vision_parser_reset(vision_parser_t *parser);
/* Returns the latest complete valid packet found. Parsing does not infer
 * arrival, filter sequence numbers or establish camera/position freshness. */
bool vision_parser_feed(vision_parser_t *parser, uint8_t byte,
                        uint32_t now_ms, vision_frame_t *out);
/* Call even during silence: recover any complete packet behind a truncated
 * candidate and discard incomplete data after the inter-byte deadline. */
bool vision_parser_poll(vision_parser_t *parser, uint32_t now_ms,
                        vision_frame_t *out);
bool vision_frame_is_image(const vision_frame_t *frame);
/* Reported source age + rounded-up wire duration + elapsed local receive time.
 * now_ms/received_ms are from the SAME monotonic receiver clock. */
uint32_t vision_effective_age_ms(const vision_frame_t *frame,
                                uint32_t received_ms, uint32_t now_ms);

#endif
