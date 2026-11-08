"""Default hardware dashboard: receives authoritative maps and sends mission commands."""
import argparse
import json
import queue
import time
import tkinter as tk
from tkinter import ttk

import paho.mqtt.client as mqtt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from onboard_state import OnboardState, mission_command, geometry_command


class OnboardDashboard:
    def __init__(self, root, broker="192.168.50.11", replay=None):
        self.root, self.client = root, None
        self.model = OnboardState()
        self.messages = queue.Queue(maxsize=128)
        self.replay = iter(replay) if replay is not None else None
        self.replay_mode = replay is not None
        root.title("Rover II - onboard DFS map")
        root.geometry("1100x800")
        self.broker = tk.StringVar(value=broker)
        self.start_id = tk.StringVar(value="1")
        self.count = tk.StringVar(value="36")
        self.threshold = tk.StringVar(value="500")
        self.status = tk.StringVar(value="Connect, place rover on start marker, then Set start node.")
        self.location = tk.StringVar(value="Current node: --")
        self.sensor_text = tk.StringVar(value="Waiting for Sensor Pico telemetry.")
        self.last_sensor = None
        self.last_heartbeat = 0
        self.height = tk.StringVar(); self.forward = tk.StringVar(); self.down = tk.StringVar()
        self.camera_text = tk.StringVar(value="Waiting for S3 measurements. Camera geometry is unmeasured.")
        row = ttk.Frame(root); row.pack(fill="x", padx=10, pady=5)
        ttk.Label(row, text="Broker").pack(side="left")
        ttk.Entry(row, textvariable=self.broker, width=20).pack(side="left")
        ttk.Button(row, text="Connect", command=self.connect).pack(side="left", padx=5)
        ttk.Button(row, text="Refresh map", command=lambda: self.send("snapshot")).pack(side="left")
        row = ttk.Frame(root); row.pack(fill="x", padx=10, pady=5)
        for label, variable in (("Start node", self.start_id), ("Markers to visit", self.count), ("Open exit >= mm", self.threshold)):
            ttk.Label(row, text=label).pack(side="left", padx=5)
            ttk.Entry(row, textvariable=variable, width=6).pack(side="left")
        ttk.Button(row, text="Set start node", command=lambda: self.send("start_node")).pack(side="left", padx=5)
        self.start_button=ttk.Button(row, text="Start DFS", command=lambda: self.send("start"), state="disabled")
        self.start_button.pack(side="left")
        row = ttk.Frame(root); row.pack(fill="x", padx=10, pady=5)
        for label, action in (("STOP", "stop"), ("E-STOP", "estop"), ("Resume", "resume"), ("Reset fault", "reset_fault"), ("Reset map", "reset_map")):
            ttk.Button(row, text=label, command=lambda a=action: self.send(a)).pack(side="left", padx=5)
        ttk.Label(root, textvariable=self.status, wraplength=1050).pack(fill="x", padx=10)
        ttk.Label(root, textvariable=self.location).pack(fill="x", padx=10)
        ttk.Label(root, textvariable=self.sensor_text).pack(fill="x", padx=10)
        ttk.Label(root, text="Green: visited | blue: seen | orange: last confirmed node | dashed edge: open, untraversed | red edge: blocked").pack()
        row=ttk.Frame(root);row.pack(fill="x",padx=10,pady=4)
        for label,variable in (("Lens height mm",self.height),("Forward offset mm",self.forward),("Downward tilt deg",self.down)):
            ttk.Label(row,text=label).pack(side="left",padx=4)
            ttk.Entry(row,textvariable=variable,width=7).pack(side="left")
        ttk.Button(row,text="Set camera geometry",command=lambda:self.send("geometry")).pack(side="left",padx=6)
        ttk.Label(root,textvariable=self.camera_text,wraplength=1060).pack(fill="x",padx=10)
        self.figure = Figure(figsize=(8, 6)); self.ax = self.figure.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.figure, master=root)
        self.toolbar = NavigationToolbar2Tk(self.canvas, root, pack_toolbar=False)
        self.toolbar.pack(side="bottom", fill="x")
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
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
            self.client.connect_async(self.broker.get(), 1883, 30)
            self.client.loop_start()
            self.status.set("Connecting...")
        except Exception as error: self.status.set(str(error))

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe([("robot/map", 0), ("robot/status", 0), ("robot/telemetry", 0)])
            client.publish("robot/command", "MAP_REQUEST", qos=0, retain=False)
            self.enqueue("local", "Connected; waiting for rover snapshot.")
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
            if action == "reset_fault": self.client.publish("robot/command", "RESET_SAFETY", qos=0, retain=False)
            result = self.client.publish("robot/command", command, qos=0, retain=False)
            if result.rc != mqtt.MQTT_ERR_SUCCESS: raise ValueError("Command was not sent")
            self.status.set(f"Sent {command}; waiting for rover response.")
        except (ValueError, KeyError) as error: self.status.set(str(error))

    def tick(self):
        if not self.replay_mode and self.client is not None and self.client.is_connected() and time.monotonic()-self.last_heartbeat>=1:
            self.client.publish("robot/heartbeat","viewer",qos=0,retain=False);self.last_heartbeat=time.monotonic()
        if self.replay is not None:
            try: self.enqueue("robot/map", json.dumps(next(self.replay)))
            except StopIteration: self.replay = None
        changed = False
        for _ in range(128):
            try: topic, payload = self.messages.get_nowait()
            except queue.Empty: break
            if topic == "robot/map":
                try: self.model.update(json.loads(payload)); changed = True
                except (ValueError, TypeError) as error: self.status.set(f"Rejected map: {error}")
            elif topic == "robot/telemetry":
                try:
                    value = json.loads(payload)
                    obstacle = value.get("obstacle", {})
                    distances = obstacle.get("distance_mm", {})
                    self.sensor_text.set(f"ToF mm: left={distances.get('left', '--')} front={distances.get('front', '--')} right={distances.get('right', '--')} | faults={obstacle.get('fault_mask', '--')} | heading={value.get('imu', {}).get('heading', '--')} deg")
                    self.last_sensor = time.monotonic()
                except (ValueError, TypeError, AttributeError): pass
            else: self.status.set(payload)
        if changed: self.draw()
        if self.last_sensor is not None and time.monotonic()-self.last_sensor>3 and "STALE" not in self.sensor_text.get():
            self.sensor_text.set(self.sensor_text.get()+" | DISPLAY STALE")
        if self.model.snapshot:
            snap = self.model.snapshot
            ready=snap.get("geometry_ready",False)
            self.start_button.configure(state="normal" if ready and snap["state"]=="READY" and not self.replay_mode and not self.model.stale() else "disabled")
            camera=snap.get("camera",{})
            if camera.get("received"):
                observations="; ".join(f"ID {m['id']}: {m['range_mm']} mm, {m['bearing_cdeg']/100:+.2f} deg, quality {m['quality']}" for m in camera.get("markers",[])) or "no marker"
                self.camera_text.set(f"S3 status={camera['status']} seq={camera['sequence']} received {camera['received_age_ms']} ms ago, effective age={camera['effective_age_ms']} ms | {observations} | Geometry {'SET' if ready else 'UNMEASURED: movement disabled'}")
            else:
                self.camera_text.set("Waiting for S3 measurements. Geometry "+("SET" if ready else "UNMEASURED: movement disabled"))
            stale = " | DISPLAY STALE" if self.model.stale() else ""
            self.location.set(f"{snap['state']}: {snap.get('reason', '')} | visited {snap['count']}/{snap.get('n', 36)} | last confirmed node {snap.get('current', '--')} | at node: {snap.get('confirmed', False)} | target {snap.get('target', '--')}{stale}")
        self.root.after(50 if self.replay is None else 100, self.tick)

    def draw(self):
        ax = self.ax; ax.clear(); ax.set_aspect("equal")
        snap = self.model.snapshot
        if snap:
            markers = snap["markers"]
            for marker in markers:
                x, y = marker["x"], marker["y"]
                colour = "#27ae60" if marker["visited"] else "#3498db"
                if marker["id"] == snap.get("current"): colour = "#f39c12"
                ax.scatter([x], [y], s=450, color=colour)
                ax.text(x, y, str(marker["id"]), ha="center", va="center")
                for d, state in enumerate(marker["exits"]):
                    if not state: continue
                    dx, dy = ((0, 1), (1, 0), (0, -1), (-1, 0))[d]
                    ax.plot([x, x+dx*.45], [y, y+dy*.45], color="red" if state == 3 else "#34495e", linestyle="--" if state == 1 else "-")
            if markers:
                minx, maxx = min(m["x"] for m in markers), max(m["x"] for m in markers)
                miny, maxy = min(m["y"] for m in markers), max(m["y"] for m in markers)
                ax.set_xlim(minx-.6, maxx+.6); ax.set_ylim(miny-.6, maxy+.6)
                ax.set_title(f"Known map: {maxx-minx+1} columns x {maxy-miny+1} rows | {len(markers)}/{snap.get('capacity', 64)} nodes")
            pose = snap.get("pose", {})
            if pose.get("valid"): ax.plot(pose["x_mm"]/snap.get("pitch_mm", 600), pose["y_mm"]/snap.get("pitch_mm", 600), "kx", markersize=12)
        else: ax.set_xlim(-.6, .6); ax.set_ylim(-.6, .6)
        ax.set_xlabel("Grid x (east)"); ax.set_ylabel("Grid y (north)"); ax.grid(True)
        self.canvas.draw_idle()

    def close(self):
        # Closing a viewer does not cancel an accepted onboard mission.
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
