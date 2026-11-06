#pragma once

#include <stddef.h>
#include <stdint.h>

// The vision message from the S3 to the Robo Pico, as agreed with Vivi. Layout:
// sync, seq, status, age_ms (2), N, N * [id, range_mm (2), bearing_cdeg (2), quality], CRC.
// Multi-byte fields are little-endian.
#define VISION_SYNC 0xA5
#define VISION_MAX_MARKERS 4
#define VISION_FRAME_MAX (7 + 6 * VISION_MAX_MARKERS)

typedef enum {
    VISION_STATUS_OK = 0,
    VISION_STATUS_NO_MARKER,
    VISION_STATUS_STALLED,
    VISION_STATUS_CAMERA_LOST,
} vision_status_t;

typedef struct {
    uint8_t id;
    float range_mm;     // straight line from the lens
    float bearing_deg;  // + = marker to the right
    uint8_t quality;    // aruco_quality_t
} vision_marker_t;

// CRC-8: polynomial 0x07, init 0, no reflection, xorout 0.
uint8_t vision_crc8(const uint8_t *data, size_t len);

// Writes one message and returns its length (7 + 6 * n). out must hold VISION_FRAME_MAX
// bytes. Values outside a field's range are capped, and n is capped at VISION_MAX_MARKERS.
size_t vision_frame_build(uint8_t *out, uint8_t seq, uint8_t status, int64_t age_ms,
                          const vision_marker_t *markers, int n);
