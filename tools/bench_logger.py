"""Motor bench logger: type bench commands AND get a timestamped log file.

Needs: pip install pyserial
Run:   python bench_logger.py COM3
       python bench_logger.py COM3 --log m11_run.txt

- Everything you type is sent to the Pico (same as a serial monitor).
- Every line from the Pico is printed with a laptop timestamp in ms.
- The same lines are saved to a log file (default: bench_log_<date-time>.txt,
  in the folder you ran the command from).
- Every 100 ms the logger silently sends 'q' so the motor state is sampled
  10 times a second (the bench only prints it once a second by itself).
- Whenever the motor state changes it prints a STATE line, with the time
  since your last typed command. This is the number for M11 / M12.
- Ctrl+C stops and closes the log file.
Close other serial monitors first (only one program can use the port).
"""
import argparse
import re
import sys
import threading
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--log", default=time.strftime("bench_log_%Y%m%d_%H%M%S.txt"))
    ap.add_argument("--poll-ms", type=int, default=100)
    a = ap.parse_args()
    import serial
    s = serial.serial_for_url(a.port, a.baud, timeout=0.05) if "://" in a.port \
        else serial.Serial(a.port, a.baud, timeout=0.05)
    t0 = time.perf_counter()
    lock = threading.Lock()
    import os
    base, ext = os.path.splitext(a.log)
    n = 1
    while os.path.exists(a.log):  # never overwrite an earlier log
        n += 1
        a.log = "%s_%d%s" % (base, n, ext)
    f = open(a.log, "w", encoding="utf-8")
    last_cmd = {"text": "", "t": t0}
    state = {"v": None}

    def now_ms():
        return (time.perf_counter() - t0) * 1000.0

    def emit(msg):
        with lock:
            print(msg)
            f.write(msg + "\n")
            f.flush()

    emit("# bench_logger started %s, port %s, log file %s"
         % (time.strftime("%Y-%m-%d %H:%M:%S"), a.port, a.log))

    def typer():
        for text in sys.stdin:
            text = text.strip()
            if not text:
                continue
            last_cmd["text"], last_cmd["t"] = text, time.perf_counter()
            emit("%9.1f ms  SENT > %s" % (now_ms(), text))
            s.write((text + "\n").encode())

    def poller():
        while True:
            time.sleep(a.poll_ms / 1000.0)
            try:
                s.write(b"q\n")
            except Exception:
                return

    threading.Thread(target=typer, daemon=True).start()
    if a.poll_ms > 0:
        threading.Thread(target=poller, daemon=True).start()

    buf = b""
    try:
        while True:
            try:
                buf += s.read(256)
            except serial.SerialException:
                emit("# port lost (USB unplugged or Pico reset)")
                f.close()
                return
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                tm = now_ms()
                text = line.decode("utf-8", "replace").rstrip()
                if text.strip() == "[bench] > q":
                    continue  # our own poll echo
                m = re.search(r"\[motor\] state=(\w+) faults=(0x[0-9a-fA-F]+)", text)
                if m:
                    # print the full status only when something changed
                    key = (m.group(1), m.group(2))
                    if key != state["v"]:
                        since = (time.perf_counter() - last_cmd["t"]) * 1000.0
                        emit("%9.1f ms  STATE %s faults=%s  (%.0f ms after '%s')"
                             % (tm, m.group(1), m.group(2), since, last_cmd["text"]))
                        state["v"] = key
                    continue
                emit("%9.1f ms  %s" % (tm, text))
    except KeyboardInterrupt:
        emit("# stopped %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
        f.close()


if __name__ == "__main__":
    main()
