#!/usr/bin/env python3
"""
MQTT Robot Controller Dashboard
Robot-centered map with odometry tracking
Requires: pip install paho-mqtt matplotlib numpy
"""

import tkinter as tk
from tkinter import messagebox
import paho.mqtt.client as mqtt
import threading
import time
import json
from datetime import datetime
import math
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import gc
from laptop_navigation import LaptopNavigator

HEARTBEAT_MS = 300
KEY_RELEASE_DEBOUNCE_MS = 50

# MQTT Topics
TOPIC_COMMAND = "robot/command"
TOPIC_STATUS = "robot/status"
TOPIC_TELEMETRY = "robot/telemetry"
TOPIC_MARKER = "camera/marker"

TOPIC_HEARTBEAT = "robot/heartbeat"
LINK_HEARTBEAT_MS = 500

ROBOT_LOST_S = 2.0
WATCHDOG_CHECK_MS = 250
TOF_UART_STALE_MS = 500

ARM_COMMANDS = {'CLAW_OPEN', 'CLAW_CLOSE', 'ARM_UP', 'ARM_DOWN'}
KEYMAP = {
            'w':'FORWARD',
            's':'BACKWARD',
            'a':'LEFT',
            'd':'RIGHT',
            'q':'ROTATE_LEFT',
            'e':'ROTATE_RIGHT',
            # Diagonal movements for mecanum wheels
            'r':'FRONT_LEFT',
            't':'FRONT_RIGHT',
            'f':'BACK_LEFT',
            'g':'BACK_RIGHT',
            # Arm & Claw mapping per hardware
            'o':'CLAW_OPEN',
            'c':'CLAW_CLOSE',
            'i':'ARM_UP',
            'k':'ARM_DOWN',
        }


class RobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Rover II Controller - MQTT + TOF")
        self.root.geometry("1600x900")
        self.root.configure(bg='#2c3e50')

        # MQTT Client
        self.mqtt_client = None
        self.connected = False
        self.broker_ip = tk.StringVar(value="localhost")
        self.broker_port = tk.IntVar(value=1883)
        self.client_id = "robot_dashboard"

        # Connection state
        self.broker_connected = tk.StringVar(value="Not Connected")
        self.robot_connected = tk.StringVar(value="Unknown")
        self.robot_status = tk.StringVar(value="Unknown")
        self.robot_ip = tk.StringVar(value="N/A")
        self.last_msg_time = tk.StringVar(value="Never")
        self.msg_count = 0

        # Press/hold state
        self.active_command = None
        self.heartbeat_job = None
        self.pressed_keys = set()
        self.release_jobs = {}

        # Link heartbeat state
        self.link_heartbeat_job = None
        self.link_heartbeat_seq = 0

        # Watchdog state
        self.last_telemetry_rx = None
        self.connect_time = 0.0
        self.robot_link = "unknown"
        
        # E-Stop state
        self.estop_active = False
        self.estop_time = 0
        self.estop_cooldown = 3.0  # 3 seconds cooldown

        # Assumed speed (%) for the odometry estimate only
        self.current_speed = tk.IntVar(value=80)
        

        # Robot odometry state
        self.robot_x = 0.0  # mm
        self.robot_y = 0.0  # mm
        self.robot_heading = 0.0  # degrees (calibrated)
        self.robot_pitch = 0.0  # degrees
        self.last_telemetry_time = time.time()
        self.odometry_lock = threading.Lock()

        # Ultrasonic distances (mm)
        self.ultrasonic = {'front': 0, 'back': 0, 'left': 0, 'right': 0}

        # ToF values arrive in the obstacle object of robot/telemetry.
        self.last_tof_rx = None

        # Map following mode
        self.follow_robot = tk.BooleanVar(value=True)
        
        # Autonomous mode
        self.auto_mode = tk.BooleanVar(value=False)
        self.dfs_target = tk.StringVar(value="36")
        # Set this from measured maze geometry before starting a mission.
        self.dfs_threshold = tk.StringVar(value="")
        self.dfs_status = tk.StringVar(value="Waiting for a marker at 450 mm from the camera lens and RoboPico telemetry")
        self.navigator = LaptopNavigator(self.send_navigation_command, self.report_navigation)

        # No on-screen control buttons (keyboard-only controls)

        self.setup_ui()
        self.root.after(100, self.navigation_tick)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def setup_ui(self):
        # Header
        header = tk.Frame(self.root, bg='#2c3e50')
        header.pack(fill='x', pady=(10, 0))
        header.grid_columnconfigure(0, weight=1)
        
        tk.Label(header, text="ROVER II CONTROLLER - MQTT + TOF",
                 font=("Arial", 20, "bold"), fg='white', bg='#2c3e50').grid(row=0, column=0, padx=20, sticky='w')
                
        # Autonomous mode toggle (top right)
        self.auto_btn = tk.Checkbutton(header, text="LAPTOP DFS", variable=self.auto_mode,
                      command=self.toggle_auto_mode,
                      bg='#2c3e50', fg='yellow', selectcolor='#1a1a1a',
                      font=("Arial", 12, "bold"))
        self.auto_btn.grid(row=0, column=1, padx=20, sticky='e')

        # Connection bar
        conn_frame = tk.Frame(self.root, bg='#34495e', relief='raised', bd=2)
        conn_frame.pack(pady=10, padx=20, fill='x')

        tk.Label(conn_frame, text="Broker IP:", fg='white', bg='#34495e',
                 font=("Arial", 10)).grid(row=0, column=0, padx=6, pady=6, sticky='w')
        self.ip_entry = tk.Entry(conn_frame, textvariable=self.broker_ip, width=16, font=("Arial", 10))
        self.ip_entry.grid(row=0, column=1, padx=6, pady=6)

        tk.Label(conn_frame, text="Port:", fg='white', bg='#34495e',
                 font=("Arial", 10)).grid(row=0, column=2, padx=6, pady=6, sticky='w')
        self.port_entry = tk.Entry(conn_frame, textvariable=self.broker_port, width=8, font=("Arial", 10))
        self.port_entry.grid(row=0, column=3, padx=6, pady=6)

        self.connect_btn = tk.Button(conn_frame, text="Connect",
                                     command=self.toggle_connection, bg='#27ae60',
                                     fg='white', font=("Arial", 10, "bold"), width=12)
        self.connect_btn.grid(row=0, column=4, padx=10, pady=6)

        # Status indicators
        status_frame = tk.Frame(self.root, bg='#34495e', relief='raised', bd=2)
        status_frame.pack(pady=5, padx=20, fill='x')

        tk.Label(status_frame, text="Broker:", fg='white', bg='#34495e',
                 font=("Arial", 10, "bold")).grid(row=0, column=0, padx=10, pady=8, sticky='w')
        self.broker_status_label = tk.Label(status_frame, textvariable=self.broker_connected,
                                            fg='#e74c3c', bg='#34495e', font=("Arial", 10, "bold"))
        self.broker_status_label.grid(row=0, column=1, padx=5, pady=8, sticky='w')

        tk.Label(status_frame, text="Robot:", fg='white', bg='#34495e',
                 font=("Arial", 10, "bold")).grid(row=0, column=2, padx=10, pady=8, sticky='w')
        self.robot_conn_label = tk.Label(status_frame, textvariable=self.robot_connected,
                                         fg='#95a5a6', bg='#34495e', font=("Arial", 10, "bold"))
        self.robot_conn_label.grid(row=0, column=3, padx=5, pady=8, sticky='w')

        tk.Label(status_frame, text="Status:", fg='white', bg='#34495e',
                 font=("Arial", 10, "bold")).grid(row=0, column=4, padx=10, pady=8, sticky='w')
        self.robot_status_label = tk.Label(status_frame, textvariable=self.robot_status,
                                           fg='#95a5a6', bg='#34495e', font=("Arial", 10, "bold"))
        self.robot_status_label.grid(row=0, column=5, padx=5, pady=8, sticky='w')

        # Heading display
        tk.Label(status_frame, text="Heading:", fg='white', bg='#34495e',
                 font=("Arial", 10, "bold")).grid(row=0, column=6, padx=10, pady=8, sticky='w')
        self.heading_label = tk.Label(status_frame, text="N/A",
                                      fg='#3498db', bg='#34495e', font=("Arial", 10, "bold"))
        self.heading_label.grid(row=0, column=7, padx=5, pady=8, sticky='w')

        # Pitch display
        tk.Label(status_frame, text="Pitch:", fg='white', bg='#34495e',
                 font=("Arial", 10, "bold")).grid(row=0, column=8, padx=10, pady=8, sticky='w')
        self.pitch_label = tk.Label(status_frame, text="N/A",
                                    fg='#3498db', bg='#34495e', font=("Arial", 10, "bold"))
        self.pitch_label.grid(row=0, column=9, padx=5, pady=8, sticky='w')

        # Add ultrasonic readings display
        tk.Label(status_frame, text="Ultrasonic (mm):", fg='white', bg='#34495e',
                 font=("Arial", 10, "bold")).grid(row=0, column=12, padx=10, pady=8, sticky='w')
        self.ultrasonic_label = tk.Label(status_frame, text="Front:0 Back:0",
                                        fg='#3498db', bg='#34495e', font=("Arial", 10, "bold"))
        self.ultrasonic_label.grid(row=0, column=13, padx=5, pady=8, sticky='w')
        
        # E-Stop button and status (prominent position)
        self.estop_btn = tk.Button(status_frame, text="⚠ E-STOP", 
                                   command=self.trigger_estop,
                                   bg='#c0392b', fg='white', font=("Arial", 12, "bold"),
                                   width=10, relief='raised', bd=3)
        self.estop_btn.grid(row=0, column=14, padx=15, pady=8)
        
        # E-Stop status indicator
        self.estop_status_label = tk.Label(status_frame, text="",
                                          fg='white', bg='#34495e', font=("Arial", 9, "bold"))
        self.estop_status_label.grid(row=0, column=15, padx=5, pady=8, sticky='w')

        self.reset_safety_btn = tk.Button(
            status_frame, text="Reset Safety", command=self.request_safety_reset,
            bg='#f39c12', fg='white', font=("Arial", 10, "bold"))
        self.reset_safety_btn.grid(row=1, column=14, padx=15, pady=(0, 8))

        # Keep ToF readings on their own row so they do not crowd the controls.
        tof_frame = tk.LabelFrame(self.root, text="ToF sensors (mm)",
                                  fg='white', bg='#34495e', font=("Arial", 12, "bold"))
        tof_frame.pack(padx=20, pady=5, fill='x')
        self.tof_labels = {}
        self.tof_state_labels = {}
        for column, direction in enumerate(('left', 'front', 'right')):
            tof_frame.grid_columnconfigure(column, weight=1, uniform='tof')
            tk.Label(tof_frame, text=direction.title(), fg='white', bg='#34495e',
                     font=("Arial", 10, "bold")).grid(row=0, column=column, pady=(5, 0))
            self.tof_labels[direction] = tk.Label(
                tof_frame, text="--", fg='#95a5a6', bg='#34495e', font=("Consolas", 22, "bold"))
            self.tof_labels[direction].grid(row=1, column=column, padx=12)
            self.tof_state_labels[direction] = tk.Label(
                tof_frame, text="Waiting", fg='#95a5a6', bg='#34495e', font=("Arial", 10))
            self.tof_state_labels[direction].grid(row=2, column=column, pady=(0, 5))
        self.tof_summary_label = tk.Label(
            tof_frame, text="Stop request: --   |   Blocked: --   |   Urgent: --",
            fg='#95a5a6', bg='#34495e', font=("Consolas", 11, "bold"))
        self.tof_summary_label.grid(row=3, column=0, columnspan=3, sticky='w', padx=12)
        self.tof_age_label = tk.Label(
            tof_frame, text="Waiting for ToF telemetry. Stop request is separate from the motor lock.",
            fg='#bdc3c7', bg='#34495e', font=("Arial", 9))
        self.tof_age_label.grid(row=4, column=0, columnspan=3, sticky='w', padx=12, pady=(3, 7))

        dfs_frame = tk.Frame(self.root, bg='#34495e')
        dfs_frame.pack(padx=20, pady=5, fill='x')
        tk.Label(dfs_frame, text="DFS target markers (1–36):", fg='white', bg='#34495e').pack(side='left', padx=6)
        tk.Entry(dfs_frame, textvariable=self.dfs_target, width=4).pack(side='left')
        tk.Label(dfs_frame, text="Open path ≥ (mm):", fg='white', bg='#34495e').pack(side='left', padx=6)
        tk.Entry(dfs_frame, textvariable=self.dfs_threshold, width=6).pack(side='left')
        tk.Label(dfs_frame, textvariable=self.dfs_status, fg='yellow', bg='#34495e').pack(side='left', padx=12)

        # Main area with 3 columns
        main = tk.Frame(self.root, bg='#2c3e50')
        main.pack(fill='both', expand=True, padx=20, pady=10)
        # Make left column slimmer to free space for map and logs
        main.grid_columnconfigure(0, weight=0, minsize=240)
        main.grid_columnconfigure(1, weight=3, minsize=800)
        main.grid_columnconfigure(2, weight=2, minsize=520)
        main.grid_rowconfigure(0, weight=1)

        # LEFT COLUMN - Key mappings only (no movement buttons)
        left_col = tk.Frame(main, bg='#2c3e50')
        left_col.grid(row=0, column=0, sticky='nsew', padx=(0, 8))
        # Key mappings panel
        keymap_frame = tk.LabelFrame(left_col, text="Key Mappings", fg='white', bg='#34495e', font=("Arial", 12, "bold"))
        keymap_frame.pack(pady=6, fill='x')
        mappings = (
            "W/S - Move Forward/Backward\n"
            "A/D - Strafe Left/Right\n"
            "Q/E - Rotate Left/Right\n"
            "R/T/F/G - Diagonals\n"
            "\n"
            "O/C - Open/Close Claw\n"
            "I/K - Arm Up/Down\n"
            "\n"
            "Space/X - Emergency Stop"
        )
        tk.Label(keymap_frame, text=mappings, justify='left', anchor='w', fg='white', bg='#34495e', font=("Consolas", 10)).pack(fill='x', padx=8, pady=8)

        # MIDDLE COLUMN - Mapping
        middle_col = tk.Frame(main, bg='#2c3e50')
        middle_col.grid(row=0, column=1, sticky='nsew', padx=8)

        self.map_frame = tk.LabelFrame(middle_col, text="Map",
                                       fg='white', bg='#34495e', font=("Arial", 14, "bold"))
        self.map_frame.pack(fill='both', expand=True)

        # !! ToF mapping display to be here
        self.fig = Figure(figsize=(6, 6), dpi=100, facecolor='#2c3e50')
        self.ax = self.fig.add_subplot(111, facecolor='#1a1a1a')
        self.ax.set_xlim(-3000, 3000)
        self.ax.set_ylim(-3000, 3000)
        self.ax.set_aspect('equal')
        self.ax.grid(True, color='#3498db', alpha=0.3, linewidth=0.5)
        self.ax.set_xlabel('X (mm)', color='white', fontsize=10)
        self.ax.set_ylabel('Y (mm)', color='white', fontsize=10)
        self.ax.set_title('Map (waiting for ToF / mapping data)', color='white', pad=15, fontsize=12)
        self.ax.tick_params(colors='white', labelsize=8)
        self.ax.plot(0, 0, 'go', markersize=8, zorder=10)  # robot

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.map_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill='both', expand=True, padx=10, pady=10)

        # RIGHT COLUMN - Log
        right_col = tk.Frame(main, bg='#2c3e50')
        right_col.grid(row=0, column=2, sticky='nsew', padx=(8, 0))
        right_col.grid_rowconfigure(0, weight=1)
        right_col.grid_columnconfigure(0, weight=1)

        log_frame = tk.LabelFrame(right_col, text="Command & Status Log",
                                  fg='white', bg='#34495e')
        log_frame.grid(row=0, column=0, sticky='nsew')
        self.log_text = tk.Text(log_frame, height=30, bg='#2c3e50', fg='#ecf0f1', 
                               font=("Consolas", 9), wrap='word', state='disabled')
        scrollbar = tk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Global key handling
        self.root.bind_all('<KeyPress>', self.on_key_press, add="+")
        self.root.bind_all('<KeyRelease>', self.on_key_release, add="+")
        self.root.focus_set()

        # Initialize E-Stop UI
        self.update_estop_ui()

        self.log("Dashboard ready. Enter broker IP and click Connect.")

        # Start watchdog timer
        self.root.after(WATCHDOG_CHECK_MS, self.watchdog_tick)

    def clear_tof_readings(self, reason="Waiting"):
        """Remove old numerical readings rather than displaying them as live."""
        self.last_tof_rx = None
        for direction in ('left', 'front', 'right'):
            self.tof_labels[direction].config(text="--", fg='#95a5a6')
            self.tof_state_labels[direction].config(text=reason, fg='#95a5a6')
        self.tof_summary_label.config(
            text="Stop request: --   |   Blocked: --   |   Urgent: --", fg='#95a5a6')
        self.tof_age_label.config(text=reason + ". Waiting for current ToF telemetry.")

    def update_tof_readings(self, obstacle):
        """Called on the Tk thread by handle_mqtt_message; never sends commands."""
        if not isinstance(obstacle, dict) or not isinstance(obstacle.get('distance_mm'), dict):
            self.clear_tof_readings("No ToF data")
            return
        age_ms = obstacle.get('age_ms') or 0
        
        age_known = (isinstance(age_ms, (int, float)) and not isinstance(age_ms, bool)
                     and math.isfinite(age_ms) and age_ms >= 0)
        if age_known and age_ms > TOF_UART_STALE_MS:
            self.clear_tof_readings("UART data stale")
            return

        def mask_value(name):
            value = obstacle.get(name)
            return value if type(value) is int and 0 <= value <= 7 else None

        blocked = mask_value('blocked_mask')
        urgent = mask_value('urgent_mask')
        faults = mask_value('fault_mask')
        distances = obstacle['distance_mm']
        for direction, bit in (('left', 1), ('front', 2), ('right', 4)):
            distance = distances.get(direction)
            usable = type(distance) is int and 1 <= distance <= 1000
            if faults is not None and faults & bit:
                text, state, color = "--", "Sensor fault", '#e74c3c'
            elif not usable:
                text = "--"
                state = "No target" if type(distance) is int and distance in (1023, 65535) else "Unavailable"
                color = '#95a5a6'
            elif urgent is not None and urgent & bit:
                text, state, color = f"{distance} mm", "Urgent", '#e74c3c'
            elif blocked is not None and blocked & bit:
                text, state, color = f"{distance} mm", "Blocked", '#f39c12'
            else:
                text, state, color = f"{distance} mm", "Range", '#3498db'
            self.tof_labels[direction].config(text=text, fg=color)
            self.tof_state_labels[direction].config(text=state, fg=color)

        stop = obstacle.get('stop_required')
        stop_text = "YES" if stop is True else ("NO" if stop is False else "--")
        blocked_text = f"0x{blocked:02X}" if blocked is not None else "--"
        urgent_text = f"0x{urgent:02X}" if urgent is not None else "--"
        self.tof_summary_label.config(
            text=f"Stop request: {stop_text}   |   Blocked: {blocked_text}   |   Urgent: {urgent_text}",
            fg='#e74c3c' if stop is True else '#ecf0f1')
        age_text = f"{age_ms:.0f} ms" if age_known else "unknown"
        self.tof_age_label.config(
            text=f"Sensor packet age: {age_text}   |   -- = no target / unavailable; see each label. "
                 "Stop request is separate from the motor lock.")
        self.last_tof_rx = time.time()

    def update_odometry(self, command, dt):
        """Update robot position based on movement command"""
        with self.odometry_lock:
            # Estimate velocity (mm/s) based on speed percentage
            speed_factor = self.current_speed.get() / 100.0
            base_velocity = 200.0 * speed_factor  # mm/s (adjust based on your robot)
            
            # Convert heading to radians
            heading_rad = math.radians(self.robot_heading)
            
            # Update position based on command
            if command == "FORWARD":
                self.robot_x += base_velocity * math.sin(heading_rad) * dt
                self.robot_y += base_velocity * math.cos(heading_rad) * dt
            elif command == "BACKWARD":
                self.robot_x -= base_velocity * math.sin(heading_rad) * dt
                self.robot_y -= base_velocity * math.cos(heading_rad) * dt
            elif command == "LEFT":
                # Strafe left (perpendicular to heading)
                self.robot_x += base_velocity * math.cos(heading_rad) * dt
                self.robot_y -= base_velocity * math.sin(heading_rad) * dt
            elif command == "RIGHT":
                # Strafe right
                self.robot_x -= base_velocity * math.cos(heading_rad) * dt
                self.robot_y += base_velocity * math.sin(heading_rad) * dt
            elif command == "FRONT_LEFT":
                # Diagonal forward-left (combine forward + left)
                diag_velocity = base_velocity * 0.707  # sqrt(2)/2 for 45° movement
                self.robot_x += diag_velocity * math.sin(heading_rad) * dt + diag_velocity * math.cos(heading_rad) * dt
                self.robot_y += diag_velocity * math.cos(heading_rad) * dt - diag_velocity * math.sin(heading_rad) * dt
            elif command == "FRONT_RIGHT":
                # Diagonal forward-right (combine forward + right)
                diag_velocity = base_velocity * 0.707
                self.robot_x += diag_velocity * math.sin(heading_rad) * dt - diag_velocity * math.cos(heading_rad) * dt
                self.robot_y += diag_velocity * math.cos(heading_rad) * dt + diag_velocity * math.sin(heading_rad) * dt
            elif command == "BACK_LEFT":
                # Diagonal backward-left (combine backward + left)
                diag_velocity = base_velocity * 0.707
                self.robot_x -= diag_velocity * math.sin(heading_rad) * dt - diag_velocity * math.cos(heading_rad) * dt
                self.robot_y -= diag_velocity * math.cos(heading_rad) * dt + diag_velocity * math.sin(heading_rad) * dt
            elif command == "BACK_RIGHT":
                # Diagonal backward-right (combine backward + right)
                diag_velocity = base_velocity * 0.707
                self.robot_x -= diag_velocity * math.sin(heading_rad) * dt + diag_velocity * math.cos(heading_rad) * dt
                self.robot_y -= diag_velocity * math.cos(heading_rad) * dt - diag_velocity * math.sin(heading_rad) * dt
            elif command == "ROTATE_LEFT":
                self.robot_heading = (self.robot_heading + 90 * dt) % 360
            elif command == "ROTATE_RIGHT":
                self.robot_heading = (self.robot_heading - 90 * dt) % 360

    def toggle_auto_mode(self):
        """The laptop owns exploration; never start the inherited Pico AUTO mode."""
        if self.auto_mode.get():
            self.stop_hold()
            self.cancel_heartbeat()
            try:
                self.navigator.start(int(self.dfs_target.get()), int(self.dfs_threshold.get()))
            except (ValueError, TypeError) as error:
                self.auto_mode.set(False)
                self.log("Cannot start laptop DFS: " + str(error))
        else:
            self.navigator.stop()

    def report_navigation(self, message):
        self.dfs_status.set(message)
        self.log(message)

    def send_navigation_command(self, command):
        if command != "STOP" and (self.estop_active or self.robot_link != "live"):
            return False
        return self.send_command(command, quiet=True)

    def navigation_tick(self):
        self.navigator.tick()
        if not self.navigator.active:
            self.auto_mode.set(False)
        self.root.after(100, self.navigation_tick)

    def submit_vision_observation(self, message):
        """Queue B3's full schema-1 JSON object from an in-process camera worker."""
        self.root.after(0, self.navigator.vision_received, message)

    def submit_marker_detection(self, marker_id, frame_id=None, distance_mm=None, bearing_deg=None):
        """Queue a simple lens-relative observation; an ID alone is not arrival."""
        if frame_id is None:
            frame_id = time.monotonic_ns()
        captured_at = time.monotonic()
        self.root.after(0, lambda: self.navigator.marker_seen(
            marker_id, frame_id, int(time.monotonic() - captured_at) * 1000,
            distance_mm, bearing_deg))

    def close(self):
        self.disconnect()
        self.root.destroy()

    def reset_odometry(self):
        """Reset robot position to origin"""
        with self.odometry_lock:
            self.robot_x = 0.0
            self.robot_y = 0.0
            self.robot_heading = 0.0
        # reset ultrasonic readings too
        self.ultrasonic = {'front': 0, 'back': 0, 'left': 0, 'right': 0}
        self.log("Robot position reset to origin")

    def request_safety_reset(self):
        """Request an unlock without resuming a held movement command."""
        self.navigator.stop("Safety reset requested")
        self.cancel_heartbeat()
        self.active_command = None
        self.auto_mode.set(False)
        if self.pressed_keys:
            self.log("Release all control keys before resetting safety.")
            return
        self.send_command("RESET_SAFETY")

    def handle_safety_status(self, status):
        """Require a new key press after every safety response."""
        self.navigator.stop(status)
        self.cancel_heartbeat()
        self.active_command = None
        self.auto_mode.set(False)
        if status == "SAFETY_READY":
            self.log("Safety reset accepted. Press a movement key to drive.")
        elif status == "SAFETY_RESET_REFUSED":
            self.log("Safety reset refused: check the front stop request and Sensor Pico UART connection.")
        else:
            self.log("Safety locked: clear the front obstacle, release control keys, then click Reset Safety.")

    def trigger_estop(self):
        """Trigger emergency stop with 3-second lockout"""
        self.navigator.stop("Emergency stop")
        # Immediately send STOP command
        self.send_command("STOP")
        
        # Cancel any active heartbeat
        self.cancel_heartbeat()
        self.active_command = None
        
        # Disable autonomous mode if active
        if self.auto_mode.get():
            self.auto_mode.set(False)
            self.log("Autonomous mode disabled by E-Stop")
        
        # Set E-Stop state
        self.estop_active = True
        self.estop_time = time.time()
        
        # Update UI
        self.update_estop_ui()
        
        # Log event with timestamp
        timestamp = time.strftime("%H:%M:%S")
        self.log(f"⚠ E-STOP ACTIVATED at {timestamp} - 3-second movement lockout enabled")
    
    def update_estop_ui(self):
        """Update E-Stop status indicator"""
        if self.estop_active:
            time_since_estop = time.time() - self.estop_time
            remaining = max(0, 3.0 - time_since_estop)
            if remaining > 0:
                self.estop_status_label.config(
                    text=f"🔒 LOCKED ({remaining:.1f}s)",
                    fg='#e74c3c',
                    bg='#c0392b'
                )
                # Schedule next update
                self.root.after(100, self.update_estop_ui)
            else:
                # Lockout expired
                self.estop_active = False
                self.estop_status_label.config(text="✓ Ready", fg='#2ecc71', bg='#34495e')
        else:
            self.estop_status_label.config(text="✓ Ready", fg='#2ecc71', bg='#34495e')

    def start_hold(self, cmd):
        """Start driving with cmd and keep resending it while held"""
        old = self.active_command
        if old and old != cmd and (old in ARM_COMMANDS or cmd in ARM_COMMANDS):
            # Drive <-> arm: stop the old one first
            self.stop_hold()
        self.active_command = cmd
        self.last_command_time = time.time()
        self.send_command(cmd)
        self.schedule_heartbeat()

    def stop_hold(self):
        if self.active_command:
            self.cancel_heartbeat()
            self.send_command("STOP")
            self.active_command = None
    
    def schedule_heartbeat(self):
        self.cancel_heartbeat()
        if self.active_command:
            self.heartbeat_job = self.root.after(HEARTBEAT_MS, self.heartbeat_tick)

    def heartbeat_tick(self):
        if self.active_command:
            # Update odometry
            current_time = time.time()
            dt = current_time - getattr(self, 'last_command_time', current_time)
            self.last_command_time = current_time
            self.update_odometry(self.active_command, dt)
            
            self.send_command(self.active_command, quiet=True)
            self.schedule_heartbeat()

    def cancel_heartbeat(self):
        if self.heartbeat_job:
            self.root.after_cancel(self.heartbeat_job)
            self.heartbeat_job = None

    def start_link_heartbeat(self):
        self.stop_link_heartbeat()
        self.link_heartbeat_tick()

    def stop_link_heartbeat(self):
        if self.link_heartbeat_job:
            self.root.after_cancel(self.link_heartbeat_job)
            self.link_heartbeat_job = None

    def link_heartbeat_tick(self):
        self.link_heartbeat_job = None
        if not self.connected or not self.mqtt_client:
            return
        self.link_heartbeat_seq = (self.link_heartbeat_seq + 1) % 256
        self.mqtt_client.publish(TOPIC_HEARTBEAT, str(self.link_heartbeat_seq), qos=0)
        self.link_heartbeat_job = self.root.after(LINK_HEARTBEAT_MS, self.link_heartbeat_tick)

    # Removed reset_all_buttons (no on-screen movement buttons to manage)

    def on_key_press(self, event):
        if isinstance(event.widget, tk.Entry):
                    return
        
        ch = event.char.lower() if event.char else ''

        if ch in self.release_jobs:
            self.root.after_cancel(self.release_jobs.pop(ch))
            return "break"

        # Key already held down: ignore the repeated presses
        if ch in self.pressed_keys:
            return "break"
        
        # Always allow emergency stop (triggers E-Stop button)
        if ch in {'x', ' '}:
            self.pressed_keys.add(ch)
            self.trigger_estop()
            return "break"
        
        # Check E-Stop lockout for movement commands
        if self.estop_active:
            time_since_estop = time.time() - self.estop_time
            if time_since_estop < 3.0:
                # Movement blocked during lockout
                if ch in {'w', 'a', 's', 'd', 'q', 'e', 'z', 'c', 'r', 't', 'f', 'g', 'o', 'i', 'k'}:
                    self.pressed_keys.add(ch)
                    self.log(f"Movement blocked - E-Stop lockout ({3.0 - time_since_estop:.1f}s remaining)")
                    return "break"
            else:
                # Lockout expired, clear E-Stop
                self.estop_active = False
                self.update_estop_ui()
                self.log("E-Stop lockout expired - movement enabled")

        # Block movement commands if robot is not connected or telemetry not received
        if ch in KEYMAP and self.connected and self.robot_link != "live":
            self.pressed_keys.add(ch)
            reason = "Robot: Lost" if self.robot_link == "lost" else "no telemetry yet"
            self.log(f"Movement blocked - {reason}")
            return "break"
        
        # Block all other keys when autonomous mode is active
        if self.auto_mode.get():
            return "break"
        
        if ch in KEYMAP:
            self.pressed_keys.add(ch)
            self.start_hold(KEYMAP[ch])
            return "break"

    def on_key_release(self, event):
        if isinstance(event.widget, tk.Entry):
                    return
        ch = event.char.lower() if event.char else ''
        if ch not in self.pressed_keys:
            return
        if ch in self.release_jobs:
            self.root.after_cancel(self.release_jobs[ch])
        self.release_jobs[ch] = self.root.after(
            KEY_RELEASE_DEBOUNCE_MS, lambda: self.finish_key_release(ch))
        return "break"
    
    def finish_key_release(self, ch):
        """A real key release (no press arrived during the debounce)"""
        self.release_jobs.pop(ch, None)
        self.pressed_keys.discard(ch)

        cmd = KEYMAP.get(ch)
        if cmd is None or cmd != self.active_command:
            return  # not the key that is driving right now: nothing to do

        for other in self.pressed_keys:
            if other in KEYMAP and other not in self.release_jobs:
                self.start_hold(KEYMAP[other])
                return

        self.stop_hold()

    def watchdog_tick(self):
        if self.connected:
            if self.last_tof_rx is not None and time.time() - self.last_tof_rx > ROBOT_LOST_S:
                self.clear_tof_readings("Telemetry stale")
            # Time since the last telemetry. If none yet, count from connect.
            ref = self.last_telemetry_rx or self.connect_time
            age = time.time() - ref
            if age > ROBOT_LOST_S and self.robot_link != "lost":
                self.set_robot_link("lost", age)
            elif (age <= ROBOT_LOST_S and self.last_telemetry_rx
                  and self.robot_link != "live"):
                self.set_robot_link("live")
        self.root.after(WATCHDOG_CHECK_MS, self.watchdog_tick)

    def set_robot_link(self, state, age=0.0):
        """Update the Robot: label and react to the link changing"""
        prev = self.robot_link
        self.robot_link = state
        ts = datetime.now().strftime("%H:%M:%S")
        if state == "live":
            self.robot_connected.set("Connected")
            self.robot_conn_label.config(fg='#27ae60')
            if prev == "lost":
                self.log(f"[{ts}] Robot link restored - movement enabled")
        elif state == "lost":
            self.navigator.stop("RoboPico telemetry lost")
            self.clear_tof_readings("Telemetry stale")
            self.robot_connected.set("Lost")
            self.robot_conn_label.config(fg='#e74c3c')
            if self.last_telemetry_rx is None:
                self.log(f"[{ts}] ⚠ Robot: Lost - no telemetry since connecting. Movement blocked.")
            else:
                self.log(f"[{ts}] ⚠ Robot: Lost - no telemetry for {age:.1f}s. Movement blocked.")
            # Stop any held key (STOP is still sent in case only one direction is broken)
            if self.active_command:
                self.stop_hold()
        else:  # "unknown"
            self.robot_connected.set("Unknown")
            self.robot_conn_label.config(fg='#95a5a6')

    def toggle_connection(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        try:
            broker = self.broker_ip.get()
            port = self.broker_port.get()
            
            self.log(f"Connecting to MQTT broker at {broker}:{port}...")
            
            self.mqtt_client = mqtt.Client(client_id=self.client_id)
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_message = self.on_mqtt_message
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            
            self.mqtt_client.connect(broker, port, 60)
            self.mqtt_client.loop_start()
            
            self.connected = True
            self.connect_time = time.time()
            self.last_telemetry_rx = None
            self.robot_link = "unknown"
            self.robot_connected.set("Waiting...")
            self.clear_tof_readings("Waiting")

            self.connect_btn.config(text="Disconnect", bg='#e74c3c')
            self.root.focus_set()
            
        except Exception as e:
            self.log(f"Connection failed: {e}")
            messagebox.showerror("Connection Error", f"Failed to connect:\n{e}")
            self.connected = False

    def disconnect(self):
        self.navigator.stop("Dashboard disconnected")
        if self.active_command:
            self.stop_hold()
        self.stop_link_heartbeat()
        self.cancel_heartbeat()
        if self.mqtt_client:
            self.mqtt_client.disconnect()
            self.mqtt_client.loop_stop()
            self.mqtt_client = None
        self.connected = False
        self.broker_connected.set("Not Connected")
        self.broker_status_label.config(fg='#e74c3c')
        self.last_telemetry_rx = None
        self.set_robot_link("unknown")
        self.clear_tof_readings("Disconnected")
        self.robot_status.set("Unknown")
        self.robot_status_label.config(fg='#95a5a6')
        self.robot_ip.set("N/A")
        self.connect_btn.config(text="Connect", bg='#27ae60')
        self.log("Disconnected from MQTT broker")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(TOPIC_STATUS)
            client.subscribe(TOPIC_TELEMETRY)
            client.subscribe(TOPIC_MARKER)
        self.root.after(0, self.handle_mqtt_connect, rc)

    def handle_mqtt_connect(self, rc):
        if rc == 0:
            self.broker_connected.set("Connected")
            self.broker_status_label.config(fg='#27ae60')
            self.log("Connected to MQTT broker")
            self.log(f"Subscribed to {TOPIC_STATUS}, {TOPIC_TELEMETRY}, {TOPIC_MARKER}")
            self.start_link_heartbeat()
        else:
            self.log(f"MQTT connection failed (code {rc})")
            self.broker_connected.set("Failed")
            self.broker_status_label.config(fg='#e74c3c')

    def on_mqtt_message(self, client, userdata, msg):
        # Replayed broker state cannot constitute a live camera/sensor observation.
        if msg.retain and msg.topic in (TOPIC_TELEMETRY, TOPIC_MARKER):
            return
        payload = msg.payload.decode(errors="replace")
        self.root.after(0, self.handle_mqtt_message, msg.topic, payload)

    def handle_mqtt_message(self, topic, payload):
        ts = datetime.now().strftime("%H:%M:%S")
        self.last_msg_time.set(ts)
        self.msg_count += 1

        if topic == TOPIC_STATUS:
            self.robot_status.set(payload)
            
            if payload in {"SAFETY_LOCKED", "SAFETY_RESET_REFUSED", "SAFETY_READY"}:
                self.robot_status_label.config(
                    fg='#27ae60' if payload == "SAFETY_READY" else '#e74c3c')
                self.handle_safety_status(payload)
            elif payload == "READY":
                self.navigator.stop("RoboPico restarted")
                self.robot_status_label.config(fg='#27ae60')
                self.log(f"[{ts}] Robot ready")
            elif payload == "STOPPED":
                if self.robot_status != "SAFETY_LOCKED":
                    self.robot_status_label.config(fg='#f39c12')
                    self.log(f"[{ts}] Robot stopped")
            elif payload == "RUNNING":
                self.robot_status_label.config(fg='#3498db')
            elif payload == "ERROR":
                self.navigator.stop("RoboPico command error")
                self.robot_status_label.config(fg='#e74c3c')
                self.log(f"[{ts}] ⚠ Robot error")
            elif payload.startswith("AUTO:"):
                # Autonomous mode status messages
                self.robot_status_label.config(fg='#f39c12')
                self.log(f"[{ts}] {payload}")
            elif payload in ["AUTO_ENABLED", "AUTO_DISABLED", "AUTO_STOPPED", "AUTO_MOVING"]:
                # Legacy auto status
                self.robot_status_label.config(fg='#f39c12')
                self.log(f"[{ts}] {payload}")
        elif topic == TOPIC_MARKER:
            try:
                data = json.loads(payload)
                if not isinstance(data, dict):
                    raise ValueError("Expected an object")
                if "schema" in data or "markers" in data:
                    self.navigator.vision_received(data)
                else:
                    self.navigator.marker_seen(data["marker_id"], data["frame_id"], data.get("age_ms", 0),
                                               data.get("distance_mm"), data.get("bearing_deg"))
            except (KeyError, TypeError, ValueError):
                self.log("Rejected invalid camera marker event")

        elif topic == TOPIC_TELEMETRY:
            try:
                data = json.loads(payload)
                if not isinstance(data, dict):
                    return
                self.navigator.sensor_received(data)
                self.update_tof_readings(data.get("obstacle"))
                imu = data.get("imu", {})
                ultrasonic = data.get("ultrasonic", {})

                # Simply use the calibrated heading directly
                heading = imu.get("heading", None)
                pitch = imu.get("pitch", None)
                
                if heading is not None:
                    with self.odometry_lock:
                        self.robot_heading = float(heading)
                    # Update heading display
                    self.heading_label.config(text=f"{self.robot_heading:.1f}°")
                
                if pitch is not None:
                    self.robot_pitch = float(pitch)
                    # Update pitch display
                    self.pitch_label.config(text=f"{self.robot_pitch:.1f}°")

                # Update ultrasonic display
                front = ultrasonic.get("front", self.ultrasonic.get('front', 0))
                back = ultrasonic.get("back", self.ultrasonic.get('back', 0))
                left = ultrasonic.get("left", self.ultrasonic.get('left', 0))
                right = ultrasonic.get("right", self.ultrasonic.get('right', 0))

                self.ultrasonic['front'] = front
                self.ultrasonic['back'] = back
                self.ultrasonic['left'] = left
                self.ultrasonic['right'] = right

                # Update label - check if offline/disconnected
                # 500 = disconnected (no echo), 400 = max range, <400 = valid reading
                front_str = "Offline" if front >= 500 else ("Far" if front >= 400 else str(front))
                back_str = "Offline" if back >= 500 else ("Far" if back >= 400 else str(back))
                
                self.ultrasonic_label.config(
                    text=f"Front:{front_str} Back:{back_str}")

                self.last_telemetry_rx = time.time()

            except (KeyError, TypeError, ValueError):
                pass

    def on_mqtt_disconnect(self, client, userdata, rc):
        if rc != 0:
            self.root.after(0, self.handle_mqtt_disconnect, rc)
            
    def handle_mqtt_disconnect(self, rc):
        self.navigator.stop("MQTT connection lost")
        self.clear_tof_readings("Disconnected")
        self.log(f"MQTT disconnected (code {rc})")
        self.broker_connected.set("Disconnected")
        self.broker_status_label.config(fg='#e74c3c')

    def send_command(self, command, quiet=False):
        if not self.connected or not self.mqtt_client:
            self.log("Not connected to broker")
            return False
        
        try:
            ts = datetime.now().strftime("%H:%M:%S")
            # Speed per move is set on the robot (motor_config.h), so only
            # the command word is sent.
            payload = command

            if not quiet:
                self.log(f"[{ts}] → {command}")
            result = self.mqtt_client.publish(TOPIC_COMMAND, payload, qos=0)
            
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                self.log(f"Publish failed (code {result.rc})")
                return False
            return True
            
        except Exception as e:
            self.log(f"Send failed: {e}")
            return False

    # Removed no-op connection monitor

    def log(self, message):
        self.log_text.config(state='normal')
        self.log_text.insert(tk.END, message + '\n')
        self.log_text.config(state='disabled')
        self.log_text.see(tk.END)
        self.root.update_idletasks()

def main():
    root = tk.Tk()
    app = RobotGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
