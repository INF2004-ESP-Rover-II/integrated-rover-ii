"""Generate S3 wire-v1 packets without a camera; optionally send via USB-TTL UART.

File generation uses only the standard library. Serial mode needs pyserial.
This is test input, not a camera detector or real position measurement.
"""
import argparse
import math
from pathlib import Path
import struct
import sys
import time

STATUS = {"ok": 0, "no_marker": 1, "stalled": 2, "camera_lost": 3}


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (0x07 if crc & 0x80 else 0)) & 0xFF
    return crc


def encode(sequence, status, age_ms, markers):
    """markers = [(id, lens_range_mm, bearing_centidegrees, opaque_quality)]."""
    if status not in STATUS.values() or not 0 <= sequence <= 255 or not 0 <= age_ms <= 65535:
        raise ValueError("invalid sequence/status/age")
    if len(markers) > 4 or ((status == STATUS["ok"]) != bool(markers)):
        raise ValueError("OK requires 1..4 markers; other statuses require zero")
    ids = set()
    body = bytearray(struct.pack("<BBHB", sequence, status, age_ms, len(markers)))
    for marker_id, distance, bearing, quality in markers:
        if (not 1 <= marker_id <= 255 or marker_id in ids or
                not 1 <= distance <= 65535 or not -18000 <= bearing <= 18000 or
                not 0 <= quality <= 255):
            raise ValueError("invalid or repeated marker")
        ids.add(marker_id)
        body.extend(struct.pack("<BHhB", marker_id, distance, bearing, quality))
    return b"\xA5" + body + bytes([crc8(body)])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    destination = ap.add_mutually_exclusive_group(required=True)
    destination.add_argument("--output", type=Path, help="write binary fixture (no hardware required)")
    destination.add_argument("--port", help="3.3 V USB-to-UART adapter port, e.g. COM7")
    ap.add_argument("--status", choices=STATUS, default="ok")
    ap.add_argument("--markers", type=int, choices=range(1, 5), default=2)
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--fps", type=float, default=5.6)
    ap.add_argument("--start-seq", type=int, choices=range(256), default=0)
    ap.add_argument("--age-ms", type=int, default=100)
    ap.add_argument("--bearing-deg", type=float, default=-2.5)
    args = ap.parse_args()
    if (args.frames < 1 or not math.isfinite(args.fps) or not 0 < args.fps <= 1000 or
            not 0 <= args.age_ms <= 65535 or not math.isfinite(args.bearing_deg) or
            not -180 <= args.bearing_deg <= 180):
        ap.error("require frames>0, fps in (0,1000], age in 0..65535 and bearing in -180..180")
    period = 1.0 / args.fps
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        stream = args.output.open("wb")
    else:
        try:
            import serial
        except ImportError:
            ap.error("serial mode needs pyserial; file generation needs no extra package")
        stream = serial.Serial(args.port, baudrate=115200, bytesize=8, parity="N", stopbits=1,
                               timeout=1, write_timeout=1)
    total = 0
    with stream:
        next_send = time.monotonic()
        for i in range(args.frames):
            is_image = args.status in ("ok", "no_marker")
            seq = (args.start_seq + i) % 256 if is_image else args.start_seq
            age = args.age_ms if is_image else min(65535, args.age_ms + round(i * period * 1000))
            markers = []
            if args.status == "ok":
                # Repeated IDs with changing range. Arbitrary quality is test data only.
                distance = 1200 - (i % 70) * 10
                markers = [(j + 1, distance + j * 600, round(args.bearing_deg * 100), 165)
                           for j in range(args.markers)]
            packet = encode(seq, STATUS[args.status], age, markers)
            if args.port is not None:
                time.sleep(max(0, next_send - time.monotonic()))
                next_send += period
            written = stream.write(packet)
            if written != len(packet):
                raise OSError("incomplete UART write")
            total += len(packet)
    print(f"Wrote {args.frames} {args.status} packets / {total} bytes to {args.output or args.port}",
          file=sys.stderr)


if __name__ == "__main__":
    main()
