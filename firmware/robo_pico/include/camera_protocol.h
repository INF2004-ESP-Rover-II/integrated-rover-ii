#ifndef ROVER_CAMERA_PROTOCOL_H
#define ROVER_CAMERA_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "vision_frame.h" // Existing ESP32 wire contract; do not duplicate it.

#define CAMERA_BYTE_GAP_MS 50u

typedef struct {
    uint8_t id;
    uint16_t range_mm;
    int16_t bearing_cdeg;
    uint8_t quality;
} camera_marker_t;

typedef struct {
    uint8_t seq;
    uint8_t status;
    uint16_t age_ms; // Age since image reached ESP32, not camera exposure.
    uint8_t count;
    camera_marker_t markers[VISION_MAX_MARKERS];
} camera_frame_t;

typedef struct {
    uint8_t bytes[VISION_FRAME_MAX];
    size_t used;
    uint32_t last_byte_ms;
    bool has_byte;
    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t format_errors;
    uint32_t gap_resets;
} camera_parser_t;

void camera_parser_init(camera_parser_t *parser);
// Discard partial input after UART data loss; preserve diagnostic counters.
void camera_parser_reset(camera_parser_t *parser);
// Feed one byte with its local UART receive time. True = one complete frame.
bool camera_parser_feed(camera_parser_t *parser, uint8_t byte,
                        uint32_t receive_ms, camera_frame_t *frame);
const char *camera_status_name(uint8_t status);
const char *camera_quality_name(uint8_t quality);

#endif
