#!/usr/bin/env python3
"""Fake Rover II over MQTT for trying the dashboard + Mapping tab with NO hardware.

Stands in for RoboPico + the camera: it publishes robot/telemetry (heading + ToF, 10 Hz) and
camera/marker (B3 schema 1, 10 Hz) from a simulated 6x6 maze, and obeys the dashboard's movement
commands (ROTATE_LEFT / ROTATE_RIGHT / FORWARD / STOP). Like the real firmware it stops by itself
if no command arrives for 0.8 s.

Run:  python -m sim.mqtt_maze_rover [host] [--port 1883] [--seed 3] [--speed 1.5] [--geometry]
Then in the dashboard: Connect, enter "DFS target markers" = 36 and "Open path >= (mm)" = 500,
wait for 'Robot: Connected', enable LAPTOP DFS. Marker 1 is where the rover starts.
Run it from the rover_mapping folder."""
import argparse
import json
import time

from sim.maze import Maze
from sim.rover_physics import RoverPhysics
from sim.world import World, add_corridor_boxes

T_COMMAND, T_STATUS, T_TELEMETRY, T_MARKER = ("robot/command", "robot/status",
                                              "robot/telemetry", "camera/marker")
MOTOR_TIMEOUT_S = 0.8


def main():
    import paho.mqtt.client as mqtt
    ap = argparse.ArgumentParser()
    ap.add_argument("host", nargs="?", default="localhost")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--speed", type=float, default=1.5, help="1 = real rover pace; keep <= 2")
    ap.add_argument("--geometry", action="store_true",
                    help="ToF from ray casts against real walls + simulated box obstacles (so the occupancy "
                         "grid has something to find). The rover stands ON the marker and 'no target' is sent "
                         "as 1000; start the dashboard with MAPPING_STANDOFF_CM=0 MAPPING_CAMERA_HEIGHT_CM=0 to match.")
    ap.add_argument("--obstacles", type=int, default=10, help="number of boxes with --geometry")
    a = ap.parse_args()
    maze = Maze.random(6, 6, extra_links=8, seed=a.seed)
    kw = {}
    if a.geometry:
        world = World(maze)
        add_corridor_boxes(world, maze, a.obstacles, a.seed + 2)
        kw = dict(world=world, standoff_cm=0.0, no_target_code=1000)
    phys = RoverPhysics(maze, 0, rotate_dps=45 * a.speed, drive_mm_s=200 * a.speed, **kw)
    state = {"last_cmd": 0.0}

    def on_message(client, userdata, msg):
        if msg.topic == T_COMMAND:
            phys.command(msg.payload.decode(errors="replace"))
            state["last_cmd"] = time.time()

    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="maze_sim_rover")
    except AttributeError:
        client = mqtt.Client(client_id="maze_sim_rover")
    client.on_message = on_message
    client.connect(a.host, a.port, 60)
    client.subscribe(T_COMMAND)
    client.loop_start()
    client.publish(T_STATUS, "READY")
    print(f"Maze rover on {a.host}:{a.port}, seed {a.seed}. Rover is at marker 1, facing north.")
    last = time.time()
    try:
        while True:
            time.sleep(0.1)
            now = time.time()
            if phys.cmd != "STOP" and now - state["last_cmd"] > MOTOR_TIMEOUT_S:
                phys.command("STOP")
            phys.step(now - last)
            last = now
            client.publish(T_TELEMETRY, json.dumps(phys.telemetry()))
            client.publish(T_MARKER, json.dumps(phys.camera_frame(now * 1000)))
    except KeyboardInterrupt:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
