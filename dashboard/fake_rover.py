#!/usr/bin/env python3
"""
Fake rover for testing the dashboard without the real robot.

Pretends to be the Robo Pico over MQTT:
  - publishes READY on robot/status at start
  - publishes telemetry (same JSON shape as the real firmware) every 1 s
  - listens on robot/command and replies RUNNING / STOPPED like the firmware
  - simulates the 800 ms motor timeout (stops if commands stop arriving)
  - simulates the planned link-loss stop: if robot/heartbeat messages were
    arriving and then stop for LINK_TIMEOUT_S, it stops the "motors"

Run:   python3 fake_rover.py            (broker on localhost:1883)
       python3 fake_rover.py 172.20.10.2
Stop:  Ctrl+C  (useful for testing the dashboard's "Robot: Lost" watchdog)
"""
import json
import sys
import time
import random
import paho.mqtt.client as mqtt

BROKER = sys.argv[1] if len(sys.argv) > 1 else "localhost"
PORT = 1883

TOPIC_COMMAND = "robot/command"
TOPIC_STATUS = "robot/status"
TOPIC_TELEMETRY = "robot/telemetry"
TOPIC_HEARTBEAT = "robot/heartbeat"   # not used by the old code yet

MOTOR_TIMEOUT_S = 0.8   # matches TIMEOUT_MS in config.h
LINK_TIMEOUT_S = 1.5    # planned FR-08 link timeout (value TBD, see A11)

state = {
    "moving": None,          # current command, or None when stopped
    "last_cmd": 0.0,
    "last_heartbeat": None,  # None until the first heartbeat arrives
    "heading": 0.0,
}


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def stop(reason):
    if state["moving"]:
        log(f"MOTORS STOP ({reason})")
    state["moving"] = None
    client.publish(TOPIC_STATUS, "STOPPED")


def on_message(client, userdata, msg):
    payload = msg.payload.decode(errors="replace")
    if msg.topic == TOPIC_HEARTBEAT:
        state["last_heartbeat"] = time.time()
        return
    if msg.topic != TOPIC_COMMAND:
        return
    cmd = payload.split(":")[0]
    log(f"<- {payload}")
    if cmd in ("STOP", "AUTO_STOP"):
        stop("STOP command")
        return
    state["moving"] = cmd
    state["last_cmd"] = time.time()
    client.publish(TOPIC_STATUS, "RUNNING")


try:
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="fake_rover")
except AttributeError:  # paho-mqtt 1.x
    client = mqtt.Client(client_id="fake_rover")
client.on_message = on_message

log(f"Connecting to {BROKER}:{PORT} ...")
client.connect(BROKER, PORT, 60)
client.subscribe(TOPIC_COMMAND)
client.subscribe(TOPIC_HEARTBEAT)
client.loop_start()
client.publish(TOPIC_STATUS, "READY")
log("Fake rover READY. Ctrl+C to 'unplug' it.")

last_telemetry = 0.0
try:
    while True:
        now = time.time()

        # Motor timeout: same behaviour as motor_check_timeout() on the Pico
        if state["moving"] and now - state["last_cmd"] > MOTOR_TIMEOUT_S:
            stop("motor timeout, no command for 800 ms")

        # Planned FR-08 behaviour: link lost -> stop
        hb = state["last_heartbeat"]
        if hb is not None and now - hb > LINK_TIMEOUT_S:
            if state["moving"]:
                stop("LINK LOST, no heartbeat")
            log("Link lost (no heartbeat). Waiting...")
            state["last_heartbeat"] = None

        # Fake heading drift while rotating
        if state["moving"] == "ROTATE_LEFT":
            state["heading"] = (state["heading"] + 9) % 360
        elif state["moving"] == "ROTATE_RIGHT":
            state["heading"] = (state["heading"] - 9) % 360

        if now - last_telemetry >= 1.0:
            last_telemetry = now
            telemetry = {
                "timestamp": int(now * 1000) % 2**32,
                "imu": {
                    "accel": {"x": 0.0, "y": 0.0, "z": 9.81},
                    "mag": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "heading": round(state["heading"], 1),
                    "pitch": 77.0,
                    "status": 0,
                },
                "ultrasonic": {
                    "front": random.randint(40, 120),
                    "back": random.randint(40, 120),
                    "left": 500,   # 500 = no sensor fitted
                    "right": 500,
                    "status": 0,
                },
            }
            client.publish(TOPIC_TELEMETRY, json.dumps(telemetry))

        time.sleep(0.1)
except KeyboardInterrupt:
    log("Fake rover unplugged.")
    client.loop_stop()
    client.disconnect()
