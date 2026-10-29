#!/usr/bin/env python3
"""
Marker pose -> distance and bearing, from a camera calibration and the
printed marker size.

Conventions (camera frame, OpenCV axes: +X right, +Y down, +Z forward):
    range_mm    straight-line camera-to-marker-centre distance, |tvec|
    forward_mm  depth along the camera's optical axis, tvec.z
    lateral_mm  sideways offset, tvec.x (+ = marker right of the camera axis)
    bearing_deg atan2(x, z): 0 = dead ahead, + = marker to the RIGHT,
                - = to the left. Camera frame, not rover heading.

Rover frame (only when a camera mount is given), on the floor plane, from
the rover's centre: +forward along the rover's nose, + = right.
    rover_forward_mm, rover_right_mm  marker centre relative to rover centre
    rover_range_mm    horizontal distance rover centre -> marker centre
    rover_bearing_deg 0 = straight ahead of the rover, + = right
These assume the camera is mounted level (not tilted up/down); the height
difference between camera and marker is ignored.

Not meant to be run directly; used by pose_stream.py.
"""

import json
import math
from typing import Dict, List, Optional

import cv2
import numpy as np


def load_calibration(path: str, frame_size=None):
    """Load camera_calib.json from calibrate.py. If the live frame size
    differs from the calibrated one (same aspect ratio), scale the camera
    matrix to match rather than silently returning wrong distances."""
    with open(path) as f:
        calib = json.load(f)
    K = np.array(calib["camera_matrix"], dtype=np.float64)
    dist = np.array(calib["dist_coeffs"], dtype=np.float64)
    calib_w, calib_h = calib["width"], calib["height"]

    if frame_size is not None and tuple(frame_size) != (calib_w, calib_h):
        w, h = frame_size
        if abs(w / h - calib_w / calib_h) > 0.01:
            raise ValueError(f"Frame {w}x{h} has a different aspect ratio from the "
                             f"calibration ({calib_w}x{calib_h}); recalibrate.")
        sx, sy = w / calib_w, h / calib_h
        K[0, 0] *= sx; K[0, 2] *= sx
        K[1, 1] *= sy; K[1, 2] *= sy
        print(f"Note: calibration was {calib_w}x{calib_h}, stream is {w}x{h}; "
              f"scaled the camera matrix. Recalibrating at {w}x{h} is more accurate.")
    return K, dist, calib


class MarkerPoseEstimator:
    def __init__(self, camera_matrix, dist_coeffs, marker_mm: float,
                 aruco_dict_type=cv2.aruco.DICT_6X6_250, mount=None):
        """mount: (forward_mm, right_mm, yaw_deg) of the camera lens relative
        to the rover centre; yaw + = camera turned to the right. None = report
        camera-frame values only."""
        self.K = camera_matrix
        self.dist = dist_coeffs
        self.marker_mm = marker_mm
        self.mount = mount
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(aruco_dict_type),
            cv2.aruco.DetectorParameters())

        # Marker corners in the marker's own frame (+Y up, +Z out of the
        # marker towards the camera), in the order ArUco returns them:
        # TL, TR, BR, BL.
        s = marker_mm / 2.0
        self.object_points = np.array([[-s, s, 0], [s, s, 0],
                                       [s, -s, 0], [-s, -s, 0]], dtype=np.float64)

    def estimate(self, corners) -> Optional[Dict]:
        """Pose for one marker's 4 image corners, or None if solvePnP fails."""
        img_pts = corners.reshape(4, 2).astype(np.float64)
        # ITERATIVE rather than IPPE_SQUARE: on OpenCV 5.0 IPPE_SQUARE returned
        # poses that did not reproject onto the corners in a synthetic test.
        ok, rvec, tvec = cv2.solvePnP(self.object_points, img_pts, self.K, self.dist,
                                      flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None

        proj, _ = cv2.projectPoints(self.object_points, rvec, tvec, self.K, self.dist)
        reproj_px = float(np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - img_pts) ** 2, axis=1))))
        side_px = float(np.mean([np.linalg.norm(img_pts[i] - img_pts[(i + 1) % 4])
                                 for i in range(4)]))

        x, y, z = tvec.flatten()
        pose = {
            "range_mm": round(float(np.linalg.norm(tvec)), 1),
            "forward_mm": round(float(z), 1),
            "lateral_mm": round(float(x), 1),
            "bearing_deg": round(math.degrees(math.atan2(x, z)), 2),
            "reproj_err_px": round(reproj_px, 3),
            "side_px": round(side_px, 1),
            "rvec": rvec.flatten().tolist(),
            "tvec": tvec.flatten().tolist(),
        }
        if self.mount is not None:
            mf, mr, yaw_deg = self.mount
            yaw = math.radians(yaw_deg)
            fwd = mf + z * math.cos(yaw) - x * math.sin(yaw)
            right = mr + z * math.sin(yaw) + x * math.cos(yaw)
            pose.update({
                "rover_forward_mm": round(fwd, 1),
                "rover_right_mm": round(right, 1),
                "rover_range_mm": round(math.hypot(fwd, right), 1),
                "rover_bearing_deg": round(math.degrees(math.atan2(right, fwd)), 2),
            })
        return pose

    def detect(self, frame: np.ndarray) -> List[Dict]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        markers = []
        if ids is None:
            return markers
        for c, marker_id in zip(corners, ids.flatten()):
            pose = self.estimate(c)
            m = {"marker_id": int(marker_id), "corners": c.reshape(4, 2).tolist(),
                 "valid": pose is not None}
            if pose:
                m.update(pose)
            markers.append(m)
        return markers

    def draw(self, frame: np.ndarray, markers: List[Dict]) -> np.ndarray:
        out = frame.copy()
        for m in markers:
            c = np.array(m["corners"], dtype=np.int32)
            cv2.polylines(out, [c], True, (0, 255, 0), 2)
            if not m["valid"]:
                continue
            cv2.drawFrameAxes(out, self.K, self.dist, np.array(m["rvec"]),
                              np.array(m["tvec"]), self.marker_mm * 0.5)
            if "rover_range_mm" in m:
                lines = [f"ID: {m['marker_id']}", f"rover {m['rover_range_mm'] / 10:.1f} cm",
                         f"{round(m['rover_bearing_deg'], 1) + 0.0:+.1f} deg"]  # +0.0: no "-0.0"
            else:
                lines = [f"ID: {m['marker_id']}", f"cam {m['range_mm'] / 10:.1f} cm",
                         f"{round(m['bearing_deg'], 1) + 0.0:+.1f} deg"]
            self._draw_label(out, lines, c)
        return out

    @staticmethod
    def _draw_label(img, lines, corners):
        """Multi-line label above the marker (below it if there is no room),
        shifted so the whole box stays inside the frame."""
        font, scale, thick, pad, gap = cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2, 4, 6
        sizes = [cv2.getTextSize(t, font, scale, thick)[0] for t in lines]
        line_h = max(s[1] for s in sizes) + gap
        box_w = max(s[0] for s in sizes) + 2 * pad
        box_h = line_h * len(lines) + pad
        h, w = img.shape[:2]

        x = int(np.clip(corners[:, 0].min(), 0, max(w - box_w, 0)))
        top = int(corners[:, 1].min()) - box_h - 6
        if top < 0:
            top = int(corners[:, 1].max()) + 6
        top = int(np.clip(top, 0, max(h - box_h, 0)))

        cv2.rectangle(img, (x, top), (x + box_w, top + box_h), (0, 0, 0), -1)
        for i, t in enumerate(lines):
            cv2.putText(img, t, (x + pad, top + (i + 1) * line_h - gap // 2),
                        font, scale, (0, 255, 255), thick)
