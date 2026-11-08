"""Default hardware dashboard: receives authoritative maps and sends mission commands."""
import argparse
import json
import queue
import time
import tkinter as tk
import math
from datetime import datetime

import paho.mqtt.client as mqtt
from onboard_layout import RestoredDashboardUI, draw_onboard_map

from onboard_state import OnboardState, mission_command, geometry_command

KEYMAP = {"w": "FORWARD", "s": "BACKWARD", "a": "ROTATE_LEFT", "d": "ROTATE_RIGHT",
          "q": "LEFT", "e": "RIGHT", "r": "FRONT_LEFT", "t": "FRONT_RIGHT",
          "f": "BACK_LEFT", "g": "BACK_RIGHT", "o": "CLAW_OPEN", "c": "CLAW_CLOSE",
          "i": "ARM_UP", "k": "ARM_DOWN"}
ARM_COMMANDS = {"CLAW_OPEN", "CLAW_CLOSE", "ARM_UP", "ARM_DOWN"}


class OnboardDashboard(RestoredDashboardUI):
    def __init__(self, root, broker="192.168.50.11", replay=None):
        self.root, self.client = root, None
        self.model = OnboardState()
        self.messages = queue.Queue(maxsize=128)
        self.replay = iter(replay) if replay is not None else None
        self.replay_mode = replay is not None
        root.title("Rover II Controller - MQTT + TOF")
        sw, sh = root.winfo_screenwidth(), root.winfo_screenheight()
        w, h = min(1600, int(sw*.95)), min(900, int(sh*.9))
        root.geometry(f"{w}x{h}+{(sw-w)//2}+0")
        root.minsize(1000, 650)
        root.configure(bg="#2c3e50")
        self.broker = self.broker_ip = tk.StringVar(value=broker)
        self.broker_port = tk.IntVar(value=1883)
        self.start_id = self.start_node_id = tk.StringVar(value="1")
        self.count = self.dfs_target = tk.StringVar(value="36")
        self.threshold = self.dfs_threshold = tk.StringVar(value="500")
        self.status = tk.StringVar(value="Connect, place rover on start marker, then Set start node.")
        self.location = tk.StringVar(value="Current node: --")
        self.dfs_status = tk.StringVar(value="IDLE")
        self.sensor_text = tk.StringVar(value="Waiting for Sensor Pico telemetry.")
        self.last_sensor = None
        self.last_heartbeat = 0
        self.height = tk.StringVar(); self.forward = tk.StringVar(); self.down = tk.StringVar()
        self.camera_text = self.marker_confirmation_text = tk.StringVar(value="Waiting for S3 measurements. Camera geometry is unmeasured.")
        self.node_confirmation_text = tk.StringVar(value="Set the physical starting marker before DFS.")
        self.current_marker_text = tk.StringVar(value="--")
        self.marker_distance_text = tk.StringVar(value="--")
        self.marker_bearing_text = tk.StringVar(value="--")
        self.last_reached_marker_text = tk.StringVar(value="--")
        self.dfs_current_node_text = tk.StringVar(value="--")
        self.broker_connected = tk.StringVar(value="Not Connected")
        self.robot_connected = tk.StringVar(value="Unknown")
        self.robot_status = tk.StringVar(value="Unknown")
        self.auto_mode = tk.BooleanVar(value=False)
        self.turn_heading = tk.StringVar(value="0")
        self.active_command = None
        self.pressed_keys = set()
        self.release_jobs = {}
        self.last_manual_send = 0
        self.setup_ui()
        self.draw()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(50, self.tick)

    def enqueue(self, topic, payload):
        try: self.messages.put_nowait((topic, payload))
        except queue.Full:
            try: self.messages.get_nowait()
            except queue.Empty: pass
            try: self.messages.put_nowait((topic, payload))
            except queue.Full: pass

    def connect(self):
        if self.replay_mode:
            self.status.set("Offline replay; no rover commands are sent."); return
        if self.client is not None: self.client.disconnect(); self.client.loop_stop()
        try:
            try: self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id="onboard_dashboard")
            except AttributeError: self.client = mqtt.Client(client_id="onboard_dashboard")
            self.client.on_connect = self.on_connect
            self.client.on_message = lambda client, userdata, msg: self.enqueue(msg.topic, msg.payload.decode("utf-8", errors="replace"))
            self.client.on_disconnect = lambda client, userdata, rc: self.enqueue("connection", "Disconnected")
            self.client.connect_async(self.broker.get(), self.broker_port.get(), 30)
            self.client.loop_start()
            self.status.set("Connecting...")
        except Exception as error: self.status.set(str(error))

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe([("robot/map", 0), ("robot/status", 0), ("robot/telemetry", 0)])
            client.publish("robot/command", "MAP_REQUEST", qos=0, retain=False)
            self.enqueue("connection", "Connected; waiting for rover snapshot.")
        else: self.enqueue("local", f"Connection failed: {rc}")

    def send(self, action):
        if self.replay_mode:
            self.status.set("Offline replay: commands disabled."); return
        try:
            if self.client is None or not self.client.is_connected(): raise ValueError("Connect to the broker first")
            arguments = {}
            if action == "start_node": arguments["marker"] = int(self.start_id.get())
            if action == "start": arguments.update(count=int(self.count.get()), opening_mm=int(self.threshold.get()))
            snapshot = self.model.snapshot or {}
            if "capacity" in snapshot: arguments["capacity"] = snapshot["capacity"]
            if "id_max" in snapshot: arguments["id_max"] = snapshot["id_max"]
            command = (geometry_command(float(self.height.get()),float(self.forward.get()),float(self.down.get()))
                       if action=="geometry" else mission_command(action, **arguments))
            if action in ("stop", "estop", "reset_map", "reset_fault"):
                self.active_command = None
                self.auto_mode.set(False)
            if action == "reset_fault": self.client.publish("robot/command", "RESET_SAFETY", qos=0, retain=False)
            result = self.client.publish("robot/command", command, qos=0, retain=False)
            if result.rc != mqtt.MQTT_ERR_SUCCESS: raise ValueError("Command was not sent")
            self.status.set(f"Sent {command}; waiting for rover response.")
            self.log(f"â†’ {command}")
        except (ValueError, KeyError) as error: self.status.set(str(error))

    def tick(self):
        if not self.replay_mode and self.client is not None and self.client.is_connected() and time.monotonic()-self.last_heartbeat>=1:
            self.client.publish("robot/heartbeat","viewer",qos=0,retain=False);self.last_heartbeat=time.monotonic()
        if self.replay is not None:
            try: self.enqueue("robot/map", json.dumps(next(self.replay)))
            except StopIteration: self.replay = None
        now = time.monotonic()
        if self.active_command and now-self.last_manual_send >= .3:
            if self.can_manual(self.active_command):
                self.send_command(self.active_command, quiet=True)
                self.last_manual_send = now
            else:
                self.stop_manual()
        changed = False
        for _ in range(128):
            try: topic, payload = self.messages.get_nowait()
            except queue.Empty: break
            if topic == "robot/map":
                try:
                    previous = self.model.snapshot
                    self.model.update(json.loads(payload))
                    changed = True
                    if previous is None:
                        self.status.set("Receiving onboard map snapshots")
                        self.log("Receiving robot/map from RoboPico.")
                    self.robot_connected.set("Connected")
                    self.robot_conn_label.config(fg="#27ae60")
                except (ValueError, TypeError) as error:
                    self.status.set(f"Rejected map: {error}")
                    self.log(self.status.get())
            elif topic == "robot/telemetry":
                try:
                    value = json.loads(payload)
                    self.update_telemetry(value)
                    self.last_sensor = time.monotonic()
                except (ValueError, TypeError, AttributeError): pass
            elif topic == "connection":
                connected = payload.startswith("Connected")
                self.broker_connected.set("Connected" if connected else "Disconnected")
                self.broker_status_label.config(fg="#27ae60" if connected else "#e74c3c")
                self.connect_btn.config(text="Disconnect" if connected else "Connect",
                                        bg="#e74c3c" if connected else "#27ae60")
                if not connected:
                    self.active_command = None
                    self.robot_connected.set("Unknown")
                self.status.set(payload)
                self.log(payload)
            else:
                self.status.set(payload)
                self.log(payload)
                if payload in ("SAFETY_LOCKED", "SAFETY_READY", "SAFETY_RESET_REFUSED"):
                    self.active_command = None
                    self.estop_status_label.config(text=payload,
                        fg="#2ecc71" if payload == "SAFETY_READY" else "#e74c3c")
        if changed: self.draw()
        if self.last_sensor is not None and now-self.last_sensor>3:
            self.clear_sensor_display()
        if self.model.stale() and (self.last_sensor is None or now-self.last_sensor>3):
            self.robot_connected.set("Stale" if self.model.snapshot else "Unknown")
            self.robot_conn_label.config(fg="#e74c3c")
        if self.model.snapshot:
            snap = self.model.snapshot
            ready=snap.get("geometry_ready",False)
            self.start_button.configure(state="normal" if ready and snap["state"]=="READY" and not self.replay_mode and not self.model.stale() and self.client is not None and self.client.is_connected() else "disabled")
            self.auto_mode.set(snap["state"] in {"SCAN", "TURN", "SETTLE", "ACQUIRE", "DRIVE", "CONFIRM", "HOLD", "RECOVERY_TURN", "RECOVERY_SETTLE"})
            self.current_marker_text.set("--")
            self.marker_distance_text.set("--")
            self.marker_bearing_text.set("--")
            self.last_reached_marker_text.set(str(snap.get("current") or "--"))
            self.dfs_current_node_text.set(str(snap.get("current") or "--"))
            camera=snap.get("camera",{})
            if camera.get("received"):
                first = next(iter(camera.get("markers", [])), None)
                if first:
                    self.current_marker_text.set(str(first["id"]))
                    self.marker_distance_text.set(f"{first['range_mm']} mm")
                    self.marker_bearing_text.set(f"{first['bearing_cdeg']/100:+.2f}°")
                observations="; ".join(f"ID {m['id']}: {m['range_mm']} mm, {m['bearing_cdeg']/100:+.2f} deg, quality {m['quality']}" for m in camera.get("markers",[])) or "no marker"
                self.camera_text.set(f"S3 status={camera['status']} seq={camera['sequence']} received {camera['received_age_ms']} ms ago, effective age={camera['effective_age_ms']} ms | {observations} | Geometry {'SET' if ready else 'UNMEASURED: movement disabled'}")
            else:
                self.camera_text.set("Waiting for S3 measurements. Geometry "+("SET" if ready else "UNMEASURED: movement disabled"))
            stale = " | DISPLAY STALE" if self.model.stale() else ""
            self.location.set(f"{snap['state']}: {snap.get('reason', '')} | visited {snap['count']}/{snap.get('n', 36)} | last confirmed node {snap.get('current', '--')} | at node: {snap.get('confirmed', False)} | target {snap.get('target', '--')}{stale}")
            self.node_confirmation_text.set(self.location.get())
            self.dfs_status.set(f"{snap['state']} | visited {snap['count']}/{snap.get('n', 0)}")
            self.robot_status.set(snap["state"])
        self.root.after(50 if self.replay is None else 100, self.tick)

    def log(self, message):
        self.log_text.config(state="normal")
        self.log_text.insert(tk.END, f"{datetime.now():%H:%M:%S}  {message}\n")
        # Keep the widget bounded during long missions.
        if int(self.log_text.index("end-1c").split(".")[0]) > 1000:
            self.log_text.delete("1.0", "100.0")
        self.log_text.config(state="disabled")
        self.log_text.see(tk.END)

    def toggle_connection(self):
        if self.client is not None and self.client.is_connected():
            self.stop_manual()
            self.client.disconnect()
            self.client.loop_stop()
            self.enqueue("connection", "Disconnected")
        else:
            self.connect()

    def restart(self):
        # Reconnect the viewer without resetting the rover or its map.
        self.stop_manual()
        self.connect()

    def toggle_auto_mode(self):
        if self.auto_mode.get():
            if str(self.start_button["state"]) == "normal":
                self.send("start")
            else:
                self.auto_mode.set(False)
                self.log("Set camera geometry and start marker; wait for a fresh READY snapshot.")
        else:
            self.send("stop")

    def set_start_node(self):
        self.send("start_node")

    def trigger_estop(self):
        self.send("estop")

    def request_safety_reset(self):
        self.active_command = None
        if self.pressed_keys:
            self.log("Release all control keys before resetting safety.")
            return
        self.send_command("RESET_SAFETY")

    def request_manual_turn(self):
        self.log("Turn to heading awaits the onboard command interface; use manual keys before anchoring a map.")

    def request_heading_zero(self):
        if self.can_manual("ZERO_HEADING"):
            self.stop_manual()
            self.send_command("ZERO_HEADING")
        else:
            self.log("Heading zero requires fresh telemetry and an unanchored map.")

    def can_manual(self, command=None):
        if self.replay_mode or self.client is None or not self.client.is_connected():
            return False
        if self.last_sensor is None or time.monotonic()-self.last_sensor > .6:
            return False
        snap = self.model.snapshot
        # An absent/stale map cannot establish that manual chassis control is allowed.
        if snap is None or self.model.stale():
            return False
        return command in ARM_COMMANDS or not snap.get("start", snap.get("current", 0))

    def send_command(self, command, quiet=False):
        if self.replay_mode:
            self.status.set("Offline replay: commands disabled.")
            return False
        if self.client is None or not self.client.is_connected():
            self.log("Connect to the broker first.")
            return False
        result = self.client.publish("robot/command", command, qos=0, retain=False)
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            self.log("Command was not sent.")
            return False
        if not quiet:
            self.log(f"â†’ {command}")
        return True

    def stop_manual(self):
        if self.active_command is not None:
            self.active_command = None
            self.send_command("STOP")

    @staticmethod
    def typing_in_field(widget):
        return isinstance(widget, (tk.Entry, tk.Text)) or widget.winfo_class() in ("TEntry", "TCombobox")

    def on_key_press(self, event):
        key = event.keysym.lower()
        if key == "space" or (key == "x" and not self.typing_in_field(event.widget)):
            self.trigger_estop()
            return "break"
        if self.typing_in_field(event.widget) or key not in KEYMAP:
            return
        if key in self.release_jobs:
            self.root.after_cancel(self.release_jobs.pop(key))
        if key in self.pressed_keys:
            return
        self.pressed_keys.add(key)
        command = KEYMAP[key]
        if not self.can_manual(command):
            self.log("Manual chassis keys require an unanchored map and fresh inputs; firmware enforces mission ownership.")
            return
        self.stop_manual()
        if self.send_command(command):
            self.active_command = command
            self.last_manual_send = time.monotonic()

    def on_key_release(self, event):
        key = event.keysym.lower()
        if key not in self.pressed_keys:
            return
        if key in self.release_jobs:
            self.root.after_cancel(self.release_jobs[key])

        def released():
            self.release_jobs.pop(key, None)
            self.pressed_keys.discard(key)
            if self.active_command == KEYMAP.get(key):
                self.stop_manual()

        self.release_jobs[key] = self.root.after(50, released)

    def on_focus_out(self, event):
        # Losing focus stops only a manually held command, never an onboard mission.
        self.stop_manual()
        self.pressed_keys.clear()

    def clear_sensor_display(self):
        for direction in ("left", "front", "right"):
            self.tof_labels[direction].config(text="--", fg="#95a5a6")
            self.tof_state_labels[direction].config(text="Stale / unavailable", fg="#95a5a6")
        self.tof_age_label.config(text="Waiting for current ToF telemetry.")
        for label in (self.heading_label, self.pitch_label, self.roll_label):
            label.config(text="N/A")
        self.ultrasonic_label.config(text="Front:-- Back:--")

    def update_telemetry(self, value):
        obstacle = value.get("obstacle", {})
        distances = obstacle.get("distance_mm", {})
        faults = obstacle.get("fault_mask", 0)
        blocked = obstacle.get("blocked_mask", 0)
        urgent = obstacle.get("urgent_mask", 0)
        age = obstacle.get("age_ms")
        stale = type(age) in (int, float) and age > 500
        for direction, bit in (("left", 1), ("front", 2), ("right", 4)):
            distance = distances.get(direction)
            if stale:
                text, state, colour = "--", "UART data stale", "#95a5a6"
            elif type(faults) is int and faults & bit:
                text, state, colour = "--", "Sensor fault", "#e74c3c"
            elif distance == 1023:
                text, state, colour = ">1000 mm", "No target within 1 m", "#3498db"
            elif type(distance) is not int or not 1 <= distance <= 1000:
                text, state, colour = "--", "Unavailable", "#95a5a6"
            elif type(urgent) is int and urgent & bit:
                text, state, colour = f"{distance} mm", "Urgent", "#e74c3c"
            elif type(blocked) is int and blocked & bit:
                text, state, colour = f"{distance} mm", "Blocked", "#f39c12"
            else:
                text, state, colour = f"{distance} mm", "Range", "#3498db"
            self.tof_labels[direction].config(text=text, fg=colour)
            self.tof_state_labels[direction].config(text=state, fg=colour)
        self.tof_summary_label.config(text=f"Stop request: {obstacle.get('stop_required', '--')}   |   Blocked: {blocked}   |   Urgent: {urgent}")
        self.tof_age_label.config(text=f"Sensor packet age: {age if age is not None else '--'} ms")
        imu = value.get("imu", {})
        for name, label in (("heading", self.heading_label), ("pitch", self.pitch_label), ("roll", self.roll_label)):
            number = imu.get(name)
            valid = imu.get("status", 0) == 0 and type(number) in (float, int) and math.isfinite(number)
            label.config(text=f"{number:.1f}°" if valid else "N/A")
        ultrasonic = value.get("ultrasonic", {})
        self.ultrasonic_label.config(text=f"Front:{ultrasonic.get('front', '--')} Back:{ultrasonic.get('back', '--')}")
        self.robot_connected.set("Connected")
        self.robot_conn_label.config(fg="#27ae60")

    def draw(self):
        draw_onboard_map(self.ax, self.model.snapshot, live=True)
        draw_onboard_map(self.mapping.ax, self.model.snapshot, selected=self.mapping.selected)
        self.canvas.draw_idle()
        self.mapping.canvas.draw_idle()

    def close(self):
        # Closing a viewer does not cancel an accepted onboard mission.
        self.stop_manual()
        if self.client is not None: self.client.disconnect(); self.client.loop_stop()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--broker", default="192.168.50.11")
    parser.add_argument("--replay", help="JSONL snapshots from test_onboard_dfs --demo")
    args = parser.parse_args()
    replay = None
    if args.replay:
        with open(args.replay, encoding="utf-8-sig") as stream:
            replay = [json.loads(line) for line in stream if line.strip().startswith("{")]
    root = tk.Tk(); OnboardDashboard(root, args.broker, replay); root.mainloop()


if __name__ == "__main__": main()
