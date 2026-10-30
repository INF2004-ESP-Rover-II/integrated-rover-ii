#!/usr/bin/env python3
"""
Fake rover: pretends to be the Robo Pico over MQTT so the dashboard can be
tested without the real robot. Matches the firmware on main (30 Oct 2026):

  Telemetry  robot/telemetry, 10 Hz, slim JSON (imu heading/pitch/status,
             ultrasonic front/back, obstacle block in Vivi's B1->B2 format:
             distances 1..1000 mm, 1023 = no target, 0 = unavailable,
             blocked <=400 mm, urgent <=150 mm, stop_required = front urgent)
  Status     robot/status: READY, RUNNING (on change), STOPPED,
             SAFETY_LOCKED / SAFETY_READY / SAFETY_RESET_REFUSED,
             SAFE STOP: ... / REFUSED: SAFE STOP (...),
             STOPPED: LINK LOST, AUTO_START_DISABLED_USE_LAPTOP_DFS
  Safety     - boots with the collision interlock LOCKED (needs Reset Safety)
             - front urgent obstacle locks it again
             - Motion Control SAFE STOP if a move gets no command for 0.8 s
             - FR-08: no heartbeat for 1.5 s -> stop (SAFE STOP if it was moving)

Type in this window (then Enter) to change the fake world:
  o  obstacle 100 mm in front (urgent -> safety lock)
  n  obstacle 300 mm in front (blocked only)
  c  clear the front
  f  toggle a ToF fault on the left sensor
  p  pause telemetry for 3 s (tests the dashboard's grey-out and "Robot: Lost")
  h  help

Run:   python fake_rover.py              (broker on localhost:1883)
       python fake_rover.py 192.168.50.10
       python fake_rover.py --maze       (also drives a simulated 6x6 maze: ToF from real walls,
                                          camera/marker frames, so DFS + the Mapping tab can run
                                          with NO hardware; keeps all the safety behaviour above)
       python fake_rover.py --maze --geometry --obstacles 10 --speed 2
                                         (ray-cast ToF + box obstacles for the obstacle map)
       With --maze set  MAPPING_STANDOFF_CM=0 MAPPING_CAMERA_HEIGHT_CM=0  before the dashboard.
Stop:  Ctrl+C  ("unplugs" the rover)
"""
import json
import random
import sys
import threading
import time

import paho.mqtt.client as mqtt

import argparse
import os

_ap = argparse.ArgumentParser(description="Fake RoboPico")
_ap.add_argument("broker", nargs="?", default="localhost")
_ap.add_argument("--maze", action="store_true", help="simulate a 6x6 marker maze + camera")
_ap.add_argument("--geometry", action="store_true", help="with --maze: ray-cast ToF + box obstacles")
_ap.add_argument("--obstacles", type=int, default=10)
_ap.add_argument("--seed", type=int, default=3)
_ap.add_argument("--speed", type=float, default=1.0, help="maze speed factor (1 = real pace; above 1 the 30 mm arrival band can be skipped at 10 Hz)")
ARGS = _ap.parse_args()
BROKER = ARGS.broker
PORT = 1883

PHYS = None
if ARGS.maze:
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rover_mapping"))
    from sim.maze import Maze
    from sim.rover_physics import RoverPhysics
    from sim.world import World, add_corridor_boxes
    _maze = Maze.random(6, 6, extra_links=8, seed=ARGS.seed)
    _kw = {}
    if ARGS.geometry:
        _world = World(_maze)
        add_corridor_boxes(_world, _maze, ARGS.obstacles, ARGS.seed + 2)
        _kw = dict(world=_world, standoff_cm=0.0, no_target_code=1000)
    PHYS = RoverPhysics(_maze, 0, rotate_dps=45 * ARGS.speed, drive_mm_s=200 * ARGS.speed, **_kw)
TOPIC_MARKER = "camera/marker"

TOPIC_COMMAND = "robot/command"
TOPIC_STATUS = "robot/status"
TOPIC_TELEMETRY = "robot/telemetry"
TOPIC_HEARTBEAT = "robot/heartbeat"

TELEMETRY_INTERVAL_S = 0.1   # SENSOR_TELEMETRY_INTERVAL_MS 100u
CMD_TIMEOUT_S = 0.8          # Motion Control command timeout (TIMEOUT_MS)
LINK_TIMEOUT_S = 1.5         # LINK_TIMEOUT_MS (FR-08)
BLOCKED_MM, URGENT_MM = 400, 150
NO_TARGET, UNAVAILABLE = 1023, 0

MOVES = {"FORWARD", "BACKWARD", "LEFT", "RIGHT", "ROTATE_LEFT", "ROTATE_RIGHT",
         "FRONT_LEFT", "FRONT_RIGHT", "BACK_LEFT", "BACK_RIGHT",
         "ARM_UP", "ARM_DOWN", "CLAW_OPEN", "CLAW_CLOSE"}

lock = threading.Lock()
s = {
    "moving": None,            # current move or None
    "last_cmd": 0.0,
    "last_reported": "",       # RUNNING only when the command changes
    "interlock": True,         # collision interlock latched (boots locked)
    "safe_stop": None,         # Motion Control SAFE STOP reason, or None
    "last_hb": None,           # last heartbeat time, None until first one
    "link_lost": True,
    "heading": 0.0,
    "front_mm": 800,           # world: distance to whatever is in front
    "left_fault": False,
    "pause_until": 0.0,
    "blocked": 0,              # proximity bits, kept between samples (hysteresis)
    "urgent": 0,
}
SPEED_MM_S = 150               # fake driving speed


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def status(text):
    client.publish(TOPIC_STATUS, text)
    log(f"-> status: {text}")


def tof_reading(mm):
    """Encode like the Sensor Pico: 1..1000 mm, 1023 = no target beyond 1 m."""
    return NO_TARGET if mm > 1000 else max(1, int(mm))


def obstacle_block():
    if PHYS:
        return PHYS.telemetry()["obstacle"]
    front = s["front_mm"] + random.randint(-5, 5)
    left = 450 + random.randint(-10, 10)
    right = 1200
    readings = {"left": tof_reading(left), "front": tof_reading(front), "right": tof_reading(right)}
    fault = 0
    if s["left_fault"]:
        readings["left"] = UNAVAILABLE
        fault |= 0x01
    # Same thresholds as the Sensor Pico, with 50 mm hysteresis:
    # blocked sets <=400 clears >=450, urgent sets <=150 clears >=200.
    blocked, urgent = s["blocked"], s["urgent"]
    for bit, name in ((0x01, "left"), (0x02, "front"), (0x04, "right")):
        d = readings[name]
        if not (1 <= d <= 1000) or (fault & bit):
            blocked &= ~bit            # no target / unavailable clears the bits
            urgent &= ~bit
            continue
        if d <= BLOCKED_MM:
            blocked |= bit
        elif d >= BLOCKED_MM + 50:
            blocked &= ~bit
        if d <= URGENT_MM:
            urgent |= bit
        elif d >= URGENT_MM + 50:
            urgent &= ~bit
    s["blocked"], s["urgent"] = blocked, urgent
    return {
        "distance_mm": readings,
        "blocked_mask": blocked,
        "urgent_mask": urgent,
        "fault_mask": fault,
        "stop_required": bool(urgent & 0x02),
        "age_ms": random.randint(10, 60),
    }


def stop_motors(reason):
    if s["moving"]:
        log(f"MOTORS STOP ({reason})")
    if PHYS:
        PHYS.command("STOP")
    s["moving"] = None
    s["last_reported"] = ""


def enter_safe_stop(reason):
    if s["safe_stop"] is None:
        s["safe_stop"] = reason
        status(f"SAFE STOP: {reason} - press Reset Safety")


def on_message(client, userdata, msg):
    payload = msg.payload.decode(errors="replace")
    with lock:
        if msg.topic == TOPIC_HEARTBEAT:
            s["last_hb"] = time.time()
            if s["link_lost"]:
                s["link_lost"] = False
                log("[LINK] Laptop link up")
            return
        if msg.topic != TOPIC_COMMAND:
            return
        s["last_hb"] = time.time()          # a command also proves the link
        cmd = payload.split(":")[0]
        log(f"<- {payload}")

        if cmd == "RESET_SAFETY":
            stop_motors("reset")
            if obstacle_block()["stop_required"]:
                status("SAFETY_RESET_REFUSED")
            else:
                s["interlock"] = False
                s["safe_stop"] = None
                status("SAFETY_READY")
            return
        if cmd in ("STOP", "AUTO_STOP"):
            stop_motors("STOP command")
            status("STOPPED" if cmd == "STOP" else "AUTO_DISABLED")
            return
        if cmd == "AUTO_START":
            status("AUTO_START_DISABLED_USE_LAPTOP_DFS")
            return
        if cmd not in MOVES:
            log(f"unknown command {cmd!r}")
            return
        if s["interlock"]:
            status("SAFETY_LOCKED")
            return
        if s["safe_stop"] is not None:
            status(f"REFUSED: SAFE STOP ({s['safe_stop']}) - press Reset Safety")
            return

        s["moving"] = cmd
        s["last_cmd"] = time.time()
        if PHYS:
            PHYS.command(cmd)
        if s["last_reported"] != cmd:
            s["last_reported"] = cmd
            status("RUNNING")


def keyboard():
    for line in sys.stdin:
        key = line.strip().lower()
        with lock:
            if key == "o":
                s["front_mm"] = 100
                log("world: obstacle 100 mm in front")
            elif key == "n":
                s["front_mm"] = 300
                log("world: obstacle 300 mm in front")
            elif key == "c":
                s["front_mm"] = 800
                log("world: front clear")
            elif key == "f":
                s["left_fault"] = not s["left_fault"]
                log(f"world: left ToF fault {'ON' if s['left_fault'] else 'OFF'}")
            elif key == "p":
                s["pause_until"] = time.time() + 3.0
                log("telemetry paused for 3 s")
            else:
                log("keys: o=obstacle 100mm  n=obstacle 300mm  c=clear  f=left ToF fault  p=pause telemetry")


try:
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="fake_rover")
except AttributeError:  # paho-mqtt 1.x
    client = mqtt.Client(client_id="fake_rover")
client.on_message = on_message

log(f"Connecting to {BROKER}:{PORT} ...")
client.connect(BROKER, PORT, 5)            # keep-alive 5 s, like the real rover
client.subscribe(TOPIC_COMMAND)
client.subscribe(TOPIC_HEARTBEAT)
client.loop_start()
status("READY")
log("Fake rover READY, safety LOCKED until Reset Safety. Type h + Enter for keys, Ctrl+C to unplug.")
threading.Thread(target=keyboard, daemon=True).start()

last_telemetry = 0.0
last_loop = time.time()
try:
    while True:
        now = time.time()
        dt, last_loop = now - last_loop, now
        with lock:
            # Motion Control command timeout -> SAFE STOP (only if it was moving)
            if s["moving"] and now - s["last_cmd"] > CMD_TIMEOUT_S:
                stop_motors("no command for 0.8 s")
                enter_safe_stop("no command for 0.8 s (link lost?)")

            # FR-08 heartbeat timeout
            hb = s["last_hb"]
            if not s["link_lost"] and (hb is None or now - hb > LINK_TIMEOUT_S):
                s["link_lost"] = True
                was_moving = s["moving"] is not None
                stop_motors("LINK LOST")
                if was_moving:
                    enter_safe_stop("laptop link lost")
                log(f"[LINK] LOST, last laptop msg {now - (hb or now):.2f} s ago")
                status("STOPPED: LINK LOST")

            if PHYS:
                PHYS.step(dt)                    # maze world drives everything below
            # Simple world: driving forward closes the gap, backward opens it
            elif s["moving"] == "FORWARD":
                s["front_mm"] = max(60, s["front_mm"] - SPEED_MM_S * dt)
            elif s["moving"] == "BACKWARD":
                s["front_mm"] = min(1500, s["front_mm"] + SPEED_MM_S * dt)
            elif s["moving"] == "ROTATE_LEFT":
                s["heading"] = (s["heading"] - 90 * dt) % 360
            elif s["moving"] == "ROTATE_RIGHT":
                s["heading"] = (s["heading"] + 90 * dt) % 360

            if PHYS:
                maze_tel = PHYS.telemetry()
                obstacle = maze_tel["obstacle"]
                s["heading"] = PHYS.heading
            else:
                obstacle = obstacle_block()
            # Collision interlock: front urgent locks it
            if obstacle["stop_required"] and not s["interlock"]:
                s["interlock"] = True
                stop_motors("front obstacle")
                status("SAFETY_LOCKED")

            send = (now - last_telemetry >= TELEMETRY_INTERVAL_S) and now >= s["pause_until"]
            if send:
                last_telemetry = now
                telemetry = {
                    "timestamp": int(now * 1000) % 2**32,
                    "imu": {"heading": round(s["heading"], 1), "pitch": 0.0, "roll": 0.0, "status": 0},
                    "ultrasonic": {"front": min(400, int(s["front_mm"]) // 10), "back": 400},
                    "obstacle": obstacle,
                }
        if send:
            client.publish(TOPIC_TELEMETRY, json.dumps(telemetry, separators=(",", ":")))
            if PHYS:
                client.publish(TOPIC_MARKER, json.dumps(PHYS.camera_frame(now * 1000)))
        time.sleep(0.02)
except KeyboardInterrupt:
    log("Fake rover unplugged.")
    client.loop_stop()
    client.disconnect()
