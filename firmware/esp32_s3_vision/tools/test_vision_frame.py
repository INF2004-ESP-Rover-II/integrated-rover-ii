#!/usr/bin/env python3
"""Self-check for firmware/main/vision_frame.c, built with the host compiler. Run: python3 test_vision_frame.py"""

import os
import struct
import subprocess
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "main")

MAIN = r"""
#include <math.h>
#include <stdio.h>
#include "vision_frame.h"
static void show(const uint8_t *b, size_t n) {
    for (size_t i = 0; i < n; i++) printf("%02x", b[i]);
    printf("\n");
}
int main(void) {
    uint8_t f[VISION_FRAME_MAX];
    vision_marker_t m[5] = {
        {7, 545.4f, 1.9f, 0}, {36, 1125.6f, -12.34f, 4}, {1, 70000.f, 200.f, 2}, {2, -5.f, -400.f, 3}, {3, 1.f, 1.f, 1},
    };
    vision_marker_t bad = {9, NAN, NAN, 1};
    printf("%02x\n", vision_crc8((const uint8_t *)"123456789", 9));
    show(f, vision_frame_build(f, 200, 1, 170, NULL, 0));
    show(f, vision_frame_build(f, 255, 0, 170, m, 1));
    show(f, vision_frame_build(f, 3, 0, 300, m, 4));
    show(f, vision_frame_build(f, 3, 0, 300, m, 5));
    show(f, vision_frame_build(f, 9, 3, 70000, NULL, 0));
    show(f, vision_frame_build(f, 9, 2, -1, NULL, 0));
    show(f, vision_frame_build(f, 9, 0, 1, &bad, 1));
    return 0;
}
"""


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def frame(seq, status, age, markers=()):
    body = struct.pack("<BBHB", seq, status, age, len(markers))
    for m in markers:
        body += struct.pack("<BHhB", *m)
    return (b"\xa5" + body + bytes([crc8(body)])).hex()


with tempfile.TemporaryDirectory(dir=HERE) as d:
    main_c, exe = os.path.join(d, "main.c"), os.path.join(d, "t")
    with open(main_c, "w") as f:
        f.write(MAIN)
    subprocess.run(["cc", "-Wall", "-Wextra", "-Werror", "-I", SRC, main_c, os.path.join(SRC, "vision_frame.c"),
                    "-lm", "-o", exe], check=True)
    lines = subprocess.run([exe], check=True, capture_output=True, text=True).stdout.split()

four = [(7, 545, 190, 0), (36, 1126, -1234, 4), (1, 65535, 20000, 2), (2, 0, -32768, 3)]

assert lines[0] == "f4", lines[0]  # the published check value of CRC-8/SMBUS
assert lines[1] == frame(200, 1, 170), lines[1]
assert len(bytes.fromhex(lines[1])) == 7
assert lines[2] == frame(255, 0, 170, four[:1]), lines[2]
assert len(bytes.fromhex(lines[2])) == 13
assert lines[3] == frame(3, 0, 300, four), lines[3]
assert len(bytes.fromhex(lines[3])) == 31
assert lines[4] == lines[3], "a fifth marker must be dropped, not overflow"
assert lines[5] == frame(9, 3, 65535), lines[5]  # age caps
assert lines[6] == frame(9, 2, 0), lines[6]
assert lines[7] == frame(9, 0, 1, [(9, 0, 0, 1)]), lines[7]  # NaN pose reads as 0
print("OK")
