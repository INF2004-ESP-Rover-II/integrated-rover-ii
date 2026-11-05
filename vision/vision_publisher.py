#!/usr/bin/env python3
"""
Marker-observation feed for the dashboard DFS: one JSON message per processed
frame, plus health messages when frames stop. Field meanings: VISION_CONTRACT.md
in ESP32-LogiCam-Test. With --mqtt it publishes to camera/marker, which
dashboard/robot_dashboard.py already decodes.

Usage (from the repo root, broker running):
    python3 vision/vision_publisher.py --marker-mm 74 --mqtt localhost
    python3 vision/vision_publisher.py --marker-mm 74 --source 0 --mqtt localhost   # laptop webcam
    python3 vision/vision_publisher.py --marker-mm 74 --stdout --no-view

Keys (video window): q = quit, f = freeze/unfreeze (pretend the stream
stalled, to test the stalled/disconnected health states).
"""

import argparse
import json
import os
import sys
import time
import uuid

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))

from frame_reader import LatestFrameReader, STREAM_URL
from marker_pose import MarkerPoseEstimator, load_calibration

SCHEMA = 1
# Validity cut-offs. Proposed, not yet tuned on the rover: see VISION_CONTRACT.md.
MAX_REPROJ_PX = 1.0     # at 640 px wide; scaled with frame width (pixel errors grow with resolution)
MIN_SIDE_PX = 20.0      # marker too small in the image -> distance too noisy
MAX_DISTANCE_MM = 1500  # beyond the range tested (120 cm seen)
STALL_S = 1.0           # no new frame for this long -> "stalled"
DISCONNECT_S = 5.0      # ... and for this long -> "disconnected"
HEARTBEAT_S = 0.2       # health message rate while there are no frames


def now_ms():
    return int(time.time() * 1000)


def load_types(path):
    try:
        with open(path) as f:
            return {int(k): v for k, v in json.load(f).items() if not k.startswith("_")}
    except FileNotFoundError:
        print(f"Note: {path} not found; every marker_type will be 'unknown'.", file=sys.stderr)
        return {}


def check(m, dist_key, max_reproj):
    """Return (valid, reason) for one detected marker."""
    if not m["valid"]:
        return False, "pose_failed"
    if m["reproj_err_px"] > max_reproj:
        return False, "high_reproj_error"
    if m["side_px"] < MIN_SIDE_PX:
        return False, "too_small"
    if m[dist_key] > MAX_DISTANCE_MM:
        return False, "too_far"
    return True, ""


def marker_msg(m, types, rover, max_reproj):
    dist_key, brg_key = ("rover_range_mm", "rover_bearing_deg") if rover else ("range_mm", "bearing_deg")
    valid, reason = check(m, dist_key, max_reproj)
    out = {"marker_id": m["marker_id"],
           "marker_type": types.get(m["marker_id"], "unknown"),
           "valid": valid, "reject_reason": reason}
    if m["valid"]:
        out.update({"distance_mm": m[dist_key], "bearing_deg": m[brg_key],
                    "distance_ref": "rover_centre" if rover else "camera_lens",
                    "camera_range_mm": m["range_mm"], "camera_forward_mm": m["forward_mm"],
                    "camera_bearing_deg": m["bearing_deg"],
                    "reproj_err_px": m["reproj_err_px"], "side_px": m["side_px"]})
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default=STREAM_URL, help="stream URL, video file, or webcam index like 0")
    ap.add_argument("--marker-mm", type=float, required=True)
    ap.add_argument("--dict", default="DICT_6X6_250")
    ap.add_argument("--calib", default=os.path.join(HERE, "camera_calib.json"))
    ap.add_argument("--markers", default=os.path.join(HERE, "markers.json"), help="marker ID -> plain/junction table")
    ap.add_argument("--cam-forward-mm", type=float, default=None)
    ap.add_argument("--cam-right-mm", type=float, default=None)
    ap.add_argument("--cam-yaw-deg", type=float, default=None)
    ap.add_argument("--record", default=None, help="also append every message to this .jsonl file")
    ap.add_argument("--stdout", action="store_true", help="print every message as one JSON line")
    ap.add_argument("--no-view", action="store_true", help="no video window")
    ap.add_argument("--mqtt", default=None, metavar="HOST", help="publish every message to this MQTT broker")
    ap.add_argument("--mqtt-port", type=int, default=1883)
    ap.add_argument("--topic", default="camera/marker")
    args = ap.parse_args()

    mount_args = (args.cam_forward_mm, args.cam_right_mm, args.cam_yaw_deg)
    mount = None if all(v is None for v in mount_args) else tuple(v or 0.0 for v in mount_args)
    types = load_types(args.markers)
    session = uuid.uuid4().hex[:8]
    log = sys.stderr if args.stdout else sys.stdout  # keep stdout clean for the JSON feed

    source = int(args.source) if args.source.isdigit() else args.source
    print(f"Opening {source} ...", file=log)
    reader = LatestFrameReader(source)
    frame, last_id = reader.read_new(0, timeout=10)
    if frame is None:
        print("No frames received. Check the ESP32 IP / Wi-Fi, and close any browser tab on /stream.", file=log)
        reader.stop()
        return
    h, w = frame.shape[:2]
    K, dist, _ = load_calibration(args.calib, (w, h))
    est = MarkerPoseEstimator(K, dist, args.marker_mm, getattr(cv2.aruco, args.dict), mount)
    max_reproj = MAX_REPROJ_PX * w / 640
    record = open(args.record, "a") if args.record else None
    client = None
    if args.mqtt:
        import paho.mqtt.client as mqtt
        client = mqtt.Client()
        client.connect(args.mqtt, args.mqtt_port)
        client.loop_start()  # background network thread; publish() is thread-safe
        print(f"Publishing to mqtt://{args.mqtt}:{args.mqtt_port}/{args.topic}", file=log)
    print(f"Session {session}: {w}x{h}, marker {args.marker_mm} mm, {args.dict}, "
          f"distances from the {'rover centre' if mount else 'camera lens'}. q = quit, f = freeze.", file=log)

    def publish(msg):
        line = json.dumps(msg)
        if args.stdout:
            print(line, flush=True)
        if record:
            record.write(line + "\n")
            record.flush()
        if client:
            client.publish(args.topic, line)  # QoS 0, not retained: old frames are useless

    last_capture, last_print, frozen = reader.read_time, 0.0, False
    try:
        while True:
            new = None
            if not frozen:
                new, new_id = reader.read_new(last_id, timeout=HEARTBEAT_S)
            else:
                time.sleep(HEARTBEAT_S)

            if new is not None:
                frame, last_id, last_capture = new, new_id, reader.read_time
                markers = est.detect(frame)
                msgs = [marker_msg(m, types, mount is not None, max_reproj) for m in markers]
                msg = {"schema": SCHEMA, "source_session_id": session, "frame_seq": last_id,
                       "capture_time_ms": int(last_capture * 1000),
                       "capture_time_source": "laptop_receive",
                       "processed_time_ms": now_ms(),
                       "status": "ok" if msgs else "no_marker",
                       "last_frame_age_ms": now_ms() - int(last_capture * 1000),
                       "markers": msgs}
            else:
                age = time.time() - last_capture
                if age < STALL_S:
                    continue
                # Health only: no markers, and frame_seq/capture_time stay at the last real frame.
                msg = {"schema": SCHEMA, "source_session_id": session, "frame_seq": last_id,
                       "capture_time_ms": int(last_capture * 1000),
                       "capture_time_source": "laptop_receive",
                       "processed_time_ms": now_ms(),
                       "status": "disconnected" if age >= DISCONNECT_S else "stalled",
                       "last_frame_age_ms": int(age * 1000), "markers": []}
            publish(msg)

            if time.time() - last_print >= 0.5:
                last_print = time.time()
                desc = "  ".join(f"ID {m['marker_id']} ({m['marker_type']}) "
                                 + (f"{m['distance_mm']:.0f} mm {m['bearing_deg']:+.1f} deg" if "distance_mm" in m else "")
                                 + ("" if m["valid"] else f" INVALID:{m['reject_reason']}")
                                 for m in msg["markers"])
                print(f"seq {msg['frame_seq']} {msg['status']} age {msg['last_frame_age_ms']} ms  {desc}", file=log)

            if not args.no_view:
                view = est.draw(frame, markers) if new is not None else frame.copy()
                label = msg["status"] + (" (FROZEN - press f)" if frozen else "")
                cv2.putText(view, label, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.imshow("Vision publisher", view)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                if key == ord('f'):
                    frozen = not frozen
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
        if record:
            record.close()
        if client:
            client.loop_stop()
            client.disconnect()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
