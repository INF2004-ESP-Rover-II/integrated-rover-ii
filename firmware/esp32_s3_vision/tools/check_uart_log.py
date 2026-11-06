#!/usr/bin/env python3
"""Check a VISION_LOOPBACK_TEST log: every sent frame was read back unchanged. Usage: check_uart_log.py <log>"""
import re
import sys
from collections import Counter

sent, read, statuses = [], {}, Counter()
last = None
for line in open(sys.argv[1], errors="replace"):
    m = re.search(r"uart sent: seq (\d+) status (\d+) age (\d+) N (\d+), first ID (-?\d+) range (\d+) bearing ([-+\d.]+) quality (-?\d+)", line)
    if m:
        sent.append(m.groups())
        statuses[m[2]] += 1
    m = re.search(r"uart read: seq (\d+) status (\d+) age (\d+) N (\d+) \|", line)
    if m:
        last = m.groups()
        read.setdefault(last, None)
    m = re.search(r"uart read:   ID (\d+) range (\d+) bearing ([-+\d.]+) quality (\d+)", line)
    if m and last and read[last] is None:
        read[last] = m.groups()

mismatch = 0
for s in sent[:-1]:  # the last line may be cut off by the end of the capture
    key = s[:4]
    if key not in read:
        mismatch += 1
    elif s[3] != "0":
        got = read[key]
        if got is None or got[0] != s[4] or abs(int(got[1]) - float(s[5])) > 1 \
                or abs(float(got[2]) - float(s[6])) > 0.011 or got[3] != s[7]:
            mismatch += 1
print(f"sent {len(sent)}, read {len(read)}, mismatched {mismatch}, by status {dict(statuses)}")
sys.exit(1 if mismatch else 0)
