/**
 * @file aruco_pose.c
 * @brief Aruco pose firmware module.
 */
#include "aruco_pose.h"

#include <math.h>
#include <stdint.h>

#include "camera_calib.h"

/* Fixed dimensions and iteration limits used by this module. */
#define UNDISTORT_ITERATIONS 5
#define MARKER_CORNER_COUNT 4
#define COORDINATE_COMPONENT_COUNT 2

#define DEG_PER_RAD 57.29578f
#define MAX_SHAPE_ERR 0.25f
#define MIN_SIDE_PX 20.0f
#define MAX_RANGE_MM 1500.0f

// Pixel to ideal (undistorted) camera coordinates, by fixed-point iteration as
// OpenCV does.
static void
undistort (float pixel_x, float pixel_y, float * p_x_coordinate,
           float * p_y_coordinate)
{
    const float origin_x      = (pixel_x - CALIB_CX) / CALIB_FX;
    const float origin_y      = (pixel_y - CALIB_CY) / CALIB_FY;
    float       undistorted_x = origin_x;
    float       undistorted_y = origin_y;
    for (int index = 0; index < UNDISTORT_ITERATIONS; index++)
    {
        const float radius_squared =
            undistorted_x * undistorted_x + undistorted_y * undistorted_y;
        const float radial =
            1.0f + radius_squared *
                       (CALIB_K1 + radius_squared *
                                       (CALIB_K2 + radius_squared * CALIB_K3));
        const float delta_x =
            2 * CALIB_P1 * undistorted_x * undistorted_y +
            CALIB_P2 * (radius_squared + 2 * undistorted_x * undistorted_x);
        const float delta_y =
            CALIB_P1 * (radius_squared + 2 * undistorted_y * undistorted_y) +
            2 * CALIB_P2 * undistorted_x * undistorted_y;
        undistorted_x = (origin_x - delta_x) / radial;
        undistorted_y = (origin_y - delta_y) / radial;
    }
    *p_x_coordinate = undistorted_x;
    *p_y_coordinate = undistorted_y;
}

aruco_pose_t
aruco_pose (const float corners[4][2], float marker_mm)
{
    aruco_pose_t pose = { 0 };
    float        value_q[4][2];
    for (int index = 0; index < MARKER_CORNER_COUNT; index++)
    {
        undistort(corners[index][0], corners[index][1], &value_q[index][0],
                  &value_q[index][1]);
        const float * p_first = corners[index];
        const float * p_second =
            corners[(uint32_t) ((index + 1)) & (uint32_t) (3)];
        pose.side_px +=
            hypotf(p_second[0] - p_first[0], p_second[1] - p_first[1]) / 4;
    }

    // Marker centre: where the diagonals cross, which perspective leaves in
    // place.
    const float d1x   = value_q[2][0] - value_q[0][0];
    const float d1y   = value_q[2][1] - value_q[0][1];
    const float d2x   = value_q[3][0] - value_q[1][0];
    const float d2y   = value_q[3][1] - value_q[1][1];
    const float cross = d1x * d2y - d1y * d2x;
    if (0.0f == cross)
    {
        return pose;
    }
    const float time_value = ((value_q[1][0] - value_q[0][0]) * d2y -
                              (value_q[1][1] - value_q[0][1]) * d2x) /
                             cross;
    const float cx = value_q[0][0] + time_value * d1x;
    const float cy = value_q[0][1] + time_value * d1y;

    // shortcut: up to about 5% off for a close, turned marker near the frame
    // edge (typically 1%); port a full planar pose if navigation needs better
    // there. Depth from the sideways extent of both edge pairs. A floor
    // marker's vertical extent is squashed and unreliable, but its sideways
    // extent is not, and a^2 + b^2 stays (size / depth)^2 however the marker is
    // turned on the floor.
    const float first = (fabsf(value_q[1][0] - value_q[0][0]) +
                         fabsf(value_q[2][0] - value_q[3][0])) /
                        2;
    const float second = (fabsf(value_q[2][0] - value_q[1][0]) +
                          fabsf(value_q[3][0] - value_q[0][0])) /
                         2;
    const float extent = hypotf(first, second);
    if (0.0f == extent)
    {
        return pose;
    }

    pose.forward_mm  = marker_mm / extent;
    pose.range_mm    = pose.forward_mm * sqrtf(1.0f + cx * cx + cy * cy);
    pose.bearing_deg = atanf(cx) * DEG_PER_RAD;

    // Opposite edges of a true square should match, apart from mild
    // perspective.
    float worst = 0;
    for (int index = 0; index < COORDINATE_COMPONENT_COUNT; index++)
    {
        const float e0 = hypotf(value_q[index + 1][0] - value_q[index][0],
                                value_q[index + 1][1] - value_q[index][1]);
        const float e1 =
            hypotf(value_q[(uint32_t) ((index + 3)) & (uint32_t) (3)][0] -
                       value_q[index + 2][0],
                   value_q[(uint32_t) ((index + 3)) & (uint32_t) (3)][1] -
                       value_q[index + 2][1]);
        const float mismatch = fabsf(e0 - e1) / (e0 + e1);
        if (mismatch > worst)
        {
            worst = mismatch;
        }
    }
    pose.shape_err = worst;
    pose.ok        = true;
    return pose;
}

aruco_quality_t
aruco_pose_quality (const aruco_pose_t * p_pose)
{
    if (!p_pose->ok)
    {
        return ARUCO_POSE_FAILED;
    }
    if (p_pose->shape_err > MAX_SHAPE_ERR)
    {
        return ARUCO_BAD_SHAPE;
    }
    if (p_pose->side_px < MIN_SIDE_PX)
    {
        return ARUCO_TOO_SMALL;
    }
    if (p_pose->range_mm > MAX_RANGE_MM)
    {
        return ARUCO_TOO_FAR;
    }
    return ARUCO_OK;
}

/* End of file. */
