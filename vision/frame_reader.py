"""Newest-frame MJPEG reader for the ESP32-S3 camera stream (from ESP32-LogiCam-Test range_test.py)."""

import threading
import time

import cv2

ESP32_IP = "192.168.50.5"
STREAM_URL = f"http://{ESP32_IP}/stream"


class LatestFrameReader:
    """Reads the stream in a background thread and keeps only the newest
    frame, so OpenCV's internal buffering doesn't hand us stale frames."""

    def __init__(self, source):
        self.source = source
        self.cap = cv2.VideoCapture(source)
        self.lock = threading.Lock()
        self.frame = None
        self.frame_id = 0
        self.frame_time = 0.0   # time.time() when the newest frame arrived
        self.read_time = 0.0    # arrival time of the frame last returned by read_new
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.5)
                self.cap.release()
                self.cap = cv2.VideoCapture(self.source)
                continue
            with self.lock:
                self.frame = frame
                self.frame_id += 1
                self.frame_time = time.time()

    def read_new(self, last_id, timeout=2.0):
        """Wait for a frame newer than last_id."""
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                if self.frame is not None and self.frame_id != last_id:
                    self.read_time = self.frame_time
                    return self.frame.copy(), self.frame_id
            time.sleep(0.005)
        return None, last_id

    def stop(self):
        self.running = False
        self.cap.release()
