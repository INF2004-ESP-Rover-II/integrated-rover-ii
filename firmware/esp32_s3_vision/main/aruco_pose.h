#pragma once

#include <stdbool.h>

// Same meaning as the laptop's marker_pose.py (camera frame).
typedef struct {
    bool ok;              // false: the corners gave no usable pose
    float range_mm;       // straight line, lens to marker centre
    float forward_mm;     // depth along the lens axis
    float bearing_deg;    // 0 = ahead, + = marker to the right
    float shape_err;      // 0 to 1: how much opposite edges differ in length
    float side_px;        // mean edge length
} aruco_pose_t;

// Why a marker is not usable.
typedef enum {
    ARUCO_OK = 0,
    ARUCO_POSE_FAILED,
    ARUCO_BAD_SHAPE,
    ARUCO_TOO_SMALL,
    ARUCO_TOO_FAR,
} aruco_quality_t;

// Pose of one marker from its corners (TL, TR, BR, BL), using camera_calib.h.
aruco_pose_t aruco_pose(const float corners[4][2], float marker_mm);

// Same limits as the laptop's validity check, except shape_err replaces its
// reprojection error, since this method fits no full pose to reproject.
aruco_quality_t aruco_pose_quality(const aruco_pose_t *pose);
