#ifndef ROVER_CAMERA_TELEMETRY_H
#define ROVER_CAMERA_TELEMETRY_H

#include "camera_protocol.h"

#define CAMERA_UART_TIMEOUT_MS 1000u
// JSON for robot/camera. 0 = invalid input/insufficient buffer; never publish a prefix.
size_t camera_telemetry_format(char *out, size_t capacity, const camera_frame_t *frame,
                               uint32_t received_ms, uint32_t now_ms);

#endif
