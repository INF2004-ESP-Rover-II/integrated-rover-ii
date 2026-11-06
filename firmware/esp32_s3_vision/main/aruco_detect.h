#ifndef ARUCO_DETECT_H
#define ARUCO_DETECT_H

#include <stddef.h>
#include <stdint.h>

typedef struct
{
    int   id;
    float corners[4][2]; // image pixels, in OpenCV's order: marker's TL, TR,
                         // BR, BL
    float perimeter;
    int   hamming;       // inner bits corrected
    int   border_errors; // border cells that read white
} aruco_marker_t;

#define ARUCO_MAX_CONTOUR(w, h) (4 * ((size_t)(w) + (h)))
// Scratch memory for aruco_detect(): threshold map, column sums, contour
// points.
#define ARUCO_WORK_SIZE(w, h)                                                  \
    ((size_t)(w) * (h) + (size_t)(w) * sizeof(uint32_t) +                      \
     ARUCO_MAX_CONTOUR(w, h) * 2 * sizeof(int16_t) + 16)

// Finds DICT_6X6 markers in one grayscale image. Portable C with no allocation.
// Pixel (x, y) is img[y * stride + x * pixel_step], so a decoded CbYCrY frame
// can be read in place (img = buf + 1, pixel_step = 2, stride = 2 * w). work
// must hold ARUCO_WORK_SIZE(w, h) bytes, 4-byte aligned. Returns markers
// written.
int aruco_detect (const uint8_t * img, int w, int h, int stride, int pixel_step,
                  void * work, aruco_marker_t * out, int max_out);

#endif /* ARUCO_DETECT_H */
