#include "aruco_pose.h"

#include <math.h>

#include "camera_calib.h"

#define DEG_PER_RAD 57.29578f
#define MAX_SHAPE_ERR 0.25f
#define MIN_SIDE_PX 20.0f
#define MAX_RANGE_MM 1500.0f

// Pixel to ideal (undistorted) camera coordinates, by fixed-point iteration as OpenCV does.
static void undistort(float px, float py, float *x, float *y)
{
    const float x0 = (px - CALIB_CX) / CALIB_FX, y0 = (py - CALIB_CY) / CALIB_FY;
    float xu = x0, yu = y0;
    for (int i = 0; i < 5; i++) {
        const float r2 = xu * xu + yu * yu;
        const float radial = 1.0f + r2 * (CALIB_K1 + r2 * (CALIB_K2 + r2 * CALIB_K3));
        const float dx = 2 * CALIB_P1 * xu * yu + CALIB_P2 * (r2 + 2 * xu * xu);
        const float dy = CALIB_P1 * (r2 + 2 * yu * yu) + 2 * CALIB_P2 * xu * yu;
        xu = (x0 - dx) / radial;
        yu = (y0 - dy) / radial;
    }
    *x = xu;
    *y = yu;
}

aruco_pose_t aruco_pose(const float corners[4][2], float marker_mm)
{
    aruco_pose_t pose = {0};
    float q[4][2];
    for (int i = 0; i < 4; i++) {
        undistort(corners[i][0], corners[i][1], &q[i][0], &q[i][1]);
        const float *a = corners[i], *b = corners[(i + 1) & 3];
        pose.side_px += hypotf(b[0] - a[0], b[1] - a[1]) / 4;
    }

    // Marker centre: where the diagonals cross, which perspective leaves in place.
    const float d1x = q[2][0] - q[0][0], d1y = q[2][1] - q[0][1];
    const float d2x = q[3][0] - q[1][0], d2y = q[3][1] - q[1][1];
    const float cross = d1x * d2y - d1y * d2x;
    if (cross == 0.0f) return pose;
    const float t = ((q[1][0] - q[0][0]) * d2y - (q[1][1] - q[0][1]) * d2x) / cross;
    const float cx = q[0][0] + t * d1x, cy = q[0][1] + t * d1y;

    // shortcut: up to about 5% off for a close, turned marker near the frame edge (typically 1%);
    // port a full planar pose if navigation needs better there.
    // Depth from the sideways extent of both edge pairs. A floor marker's vertical
    // extent is squashed and unreliable, but its sideways extent is not, and
    // a^2 + b^2 stays (size / depth)^2 however the marker is turned on the floor.
    const float a = (fabsf(q[1][0] - q[0][0]) + fabsf(q[2][0] - q[3][0])) / 2;
    const float b = (fabsf(q[2][0] - q[1][0]) + fabsf(q[3][0] - q[0][0])) / 2;
    const float extent = hypotf(a, b);
    if (extent == 0.0f) return pose;

    pose.forward_mm = marker_mm / extent;
    pose.range_mm = pose.forward_mm * sqrtf(1.0f + cx * cx + cy * cy);
    pose.bearing_deg = atanf(cx) * DEG_PER_RAD;

    // Opposite edges of a true square should match, apart from mild perspective.
    float worst = 0;
    for (int i = 0; i < 2; i++) {
        const float e0 = hypotf(q[i + 1][0] - q[i][0], q[i + 1][1] - q[i][1]);
        const float e1 = hypotf(q[(i + 3) & 3][0] - q[i + 2][0], q[(i + 3) & 3][1] - q[i + 2][1]);
        const float mismatch = fabsf(e0 - e1) / (e0 + e1);
        if (mismatch > worst) worst = mismatch;
    }
    pose.shape_err = worst;
    pose.ok = true;
    return pose;
}

aruco_quality_t aruco_pose_quality(const aruco_pose_t *pose)
{
    if (!pose->ok) return ARUCO_POSE_FAILED;
    if (pose->shape_err > MAX_SHAPE_ERR) return ARUCO_BAD_SHAPE;
    if (pose->side_px < MIN_SIDE_PX) return ARUCO_TOO_SMALL;
    if (pose->range_mm > MAX_RANGE_MM) return ARUCO_TOO_FAR;
    return ARUCO_OK;
}
