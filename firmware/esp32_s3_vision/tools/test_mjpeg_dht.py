#!/usr/bin/env python3
"""Self-check for firmware/main/mjpeg_dht.c, built with the host compiler. Run: python3 test_mjpeg_dht.py"""

import os
import subprocess
import tempfile

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "main")

MAIN = r"""
#include <stdio.h>
#include <stdlib.h>
#include "mjpeg_dht.h"
int main(int argc, char **argv) {
    (void)argc;
    size_t cap = atoi(argv[1]);
    static uint8_t src[1 << 20], dst[1 << 20];
    size_t len = fread(src, 1, sizeof src, stdin);
    bool had = false;
    size_t n = mjpeg_with_dht(src, len, dst, cap, &had);
    fprintf(stderr, "%d", had);
    fwrite(dst, 1, n, stdout);
    return 0;
}
"""


def strip_dht(data):
    i, out = 2, data[:2]
    while True:
        marker, n = data[i + 1], int.from_bytes(data[i + 2:i + 4], "big")
        if marker == 0xDA:
            return out + data[i:]
        if marker != 0xC4:
            out += data[i:i + 2 + n]
        i += 2 + n


with tempfile.TemporaryDirectory(dir=HERE) as d:
    main_c, exe = os.path.join(d, "main.c"), os.path.join(d, "t")
    with open(main_c, "w") as f:
        f.write(MAIN)
    subprocess.run(["cc", "-Wall", "-Wextra", "-Werror", "-I", SRC, main_c, os.path.join(SRC, "mjpeg_dht.c"), "-o", exe], check=True)

    def run(data, cap=1 << 20):
        r = subprocess.run([exe, str(cap)], input=data, capture_output=True, check=True)
        return r.stdout, r.stderr

    img = cv2.GaussianBlur((np.random.default_rng(1).random((480, 640, 3)) * 255).astype(np.uint8), (9, 9), 0)
    full = cv2.imencode(".jpg", img)[1].tobytes()
    bare = strip_dht(full)
    assert len(full) - len(bare) == 432 and b"\xff\xc4" not in bare[:700]

    out, had = run(full)
    assert out == full and had == b"1", "a frame with tables must pass through unchanged"

    out, had = run(bare)
    assert had == b"0" and len(out) == len(bare) + 432
    assert np.array_equal(cv2.imdecode(np.frombuffer(out, np.uint8), cv2.IMREAD_COLOR),
                          cv2.imdecode(np.frombuffer(full, np.uint8), cv2.IMREAD_COLOR)), "pixels must match the original"

    # Rejected, not overrun: too-small output, not a JPEG, truncated and malformed headers.
    assert run(bare, len(bare) + 431)[0] == b""
    assert run(full, len(full) - 1)[0] == b""
    assert run(b"not a jpeg at all")[0] == b""
    assert run(b"")[0] == b""
    assert run(bare[:10])[0] == b""
    assert run(b"\xff\xd8\xff\xdb\x00\x00" + bare[6:])[0] == b"", "segment length under 2"

print("OK")
