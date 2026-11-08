#!/usr/bin/env python3
"""
MQTT Robot Controller Dashboard
Marker grid oriented relative to the first fresh IMU heading.
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
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import gc
import os
import sys
import uuid
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'rover_mapping'))
from tkinter import ttk
import os
from laptop_navigation import LaptopNavigator
from mission_display import parse_mission_report, parse_target_selection, parse_dfs_reply, parse_dfs_tracking, parse_dfs_state, LEG_FIELDS
from rover_camera import RoverCameraCache, QUALITY_NAMES
from onboard_map import OnboardMapCache
from vision_feed import VisionFeed
from matplotlib.ticker import MaxNLocator


def make_mqtt_client(client_id):
    """paho-mqtt 1.x and 2.x both work (2.x needs an explicit callback API version)."""
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=client_id)
    except AttributeError:
        return mqtt.Client(client_id=client_id)


HEARTBEAT_MS = 300
KEY_RELEASE_DEBOUNCE_MS = 50

# MQTT Topics
TOPIC_COMMAND = "robot/command"
TOPIC_STATUS = "robot/status"
TOPIC_TELEMETRY = "robot/telemetry"
TOPIC_MARKER = "camera/marker"
TOPIC_ROVER_CAMERA = "robot/camera"
TOPIC_ROVER_MAP = "robot/map"

TOPIC_HEARTBEAT = "robot/heartbeat"
LINK_HEARTBEAT_MS = 500

ROBOT_LOST_S = 2.0
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
WATCHDOG_CHECK_MS = 250
TOF_UART_STALE_MS = 500

ARM_COMMANDS = {'CLAW_OPEN', 'CLAW_CLOSE', 'ARM_UP', 'ARM_DOWN'}
KEYMAP = {
            'w':'FORWARD',
            's':'BACKWARD',
            'a':'ROTATE_LEFT',
            'd':'ROTATE_RIGHT',
            'q':'LEFT',
            'e':'RIGHT',
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
        self.root.title("Rover II Controller - MQTT + TOF | Reference tracking")
        sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        w, h = min(1600, int(sw * 0.95)), min(900, int(sh * 0.9))
        self.root.geometry(f"{w}x{h}+{(sw - w) // 2}+0")
        self.root.minsize(1000, 650)
        self.root.configure(bg='#2c3e50')

        # MQTT Client
        self.mqtt_client = None
        self.connected = False
        self.broker_ip = tk.StringVar(value="192.168.50.11")
        self.broker_port = tk.IntVar(value=1883)
        self.client_id = "robot_dashboard"

        # Connection state
        self.broker_connected = tk.StringVar(value="Not Connected")
        self.robot_connected = tk.StringVar(value="Unknown")
        self.robot_status = tk.StringVar(value="Unknown")
        self.mission_state = tk.StringVar(value="Unknown")
        self.mission_reason = tk.StringVar(value="--")
        self.mission_mode = tk.StringVar(value="Unknown")
        self.mission_motion = tk.StringVar(value="--")
        self.mission_leg_fields = {key: tk.StringVar(value='Unknown' if key == 'phase' else '--') for key in LEG_FIELDS}
        self.mission_report = None
        self.target_selection_report = None
        self.target_selection_status = tk.StringVar(value='Waiting for RoboPico selection report')
        self.dfs_tracking_text = tk.StringVar(value='Tracking: waiting for RoboPico')
        self.dfs_tracking_report = None
        self.dfs_planner_report = None
        self.dfs_planner_text = tk.StringVar(value='DFS: waiting for RoboPico')
        self.last_dfs_reply = None
        self.last_mission_rx = None
        self.last_mission_reply = None
        self.onboard_map = OnboardMapCache()
        self.pending_start_request = None
        self.start_node_status = tk.StringVar(value="Set the node under the rover; RoboPico will confirm the map")
        self.onboard_map_status = tk.StringVar(value="Waiting for RoboPico map")
        self.robot_ip = tk.StringVar(value="N/A")
        self.last_msg_time = tk.StringVar(value="Never")
        self.msg_count = 0

        # Press/hold state
        self.active_command = None
        self.heartbeat_job = None
        self.front_blocked = False
        self.front_obstacle_snapshot = None
        self.forward_requires_release = False
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
        self.stop_sent_at = None
        self.stop_latencies = []
        self.last_refused = None
        self.turn_heading = tk.StringVar(value="90")
        self.manual_turn_target = None
        self.manual_turn_job = None
        self.manual_turn_acked = False
        self.heading_zero_pending = False
        self.heading_zero_job = None

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
        self.dfs_threshold = tk.StringVar(value="500")
        self.dfs_status = tk.StringVar(value="Set the starting node under the rover before starting DFS")
        self.dfs_feedback = tk.StringVar(value='')
        self.start_node_id = tk.StringVar(value="")
        self.current_marker_text = tk.StringVar(value="--")
        self.marker_distance_text = tk.StringVar(value="--")
        self.marker_bearing_text = tk.StringVar(value="--")
        self.rover_camera = RoverCameraCache()
        self.camera_relay_text = tk.StringVar(value="RoboPico camera: waiting for UART observations")
        self.camera_marker_count = tk.StringVar(value="Markers in view: 0")
        self.camera_marker_rows = None
        self.last_reached_marker_text = tk.StringVar(value="--")
        self.dfs_current_node_text = tk.StringVar(value="--")
        self.marker_confirmation_text = tk.StringVar(value="Waiting for fresh camera data")
        self.node_confirmation_text = tk.StringVar(value="")
        self.navigator = LaptopNavigator(self.send_navigation_command, self.report_navigation)
        self.navigator.grid_required = True

        # Keyboard drive controls and the manual heading test share this window.

        # Logging
        os.makedirs(LOG_DIR, exist_ok=True)
        log_name = datetime.now().strftime("dashboard_%Y%m%d_%H%M%S.txt")
        self.log_file = open(os.path.join(LOG_DIR, log_name), "a", encoding="utf-8", buffering=1)

        self.setup_ui()
        self.log('Dashboard source: ' + os.path.abspath(__file__))
        self.root.after(100, self.navigation_tick)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def setup_ui(self):
        # Header
        header = tk.Frame(self.root, bg='#2c3e50')
        header.pack(fill='x', pady=(4, 0))
        header.grid_columnconfigure(0, weight=1)
        
        tk.Label(header, text="ROVER II CONTROLLER - MQTT + TOF",
                 font=("Arial", 20, "bold"), fg='white', bg='#2c3e50').grid(row=0, column=0, padx=20, sticky='w')
                
        # Autonomous mode toggle (top right)
        dfs_controls = tk.Frame(header, bg='#2c3e50')
        dfs_controls.grid(row=0, column=1, padx=20, sticky='e')
        self.auto_btn = tk.Checkbutton(dfs_controls, text="Start DFS", variable=self.auto_mode,
                      command=self.toggle_auto_mode,
                      bg='#2c3e50', fg='yellow', selectcolor='#1a1a1a',
                      font=("Arial", 12, "bold"))
        self.auto_btn.grid(row=0, column=0, sticky='w')
        tk.Button(dfs_controls, text='Reset mission', command=lambda: self.send_command('RESET_MISSION'),
                  bg='#7f8c8d', fg='white').grid(row=0, column=1, padx=6)
        self.dfs_feedback_label = tk.Label(dfs_controls, textvariable=self.dfs_feedback, fg='#f1c40f',
            bg='#2c3e50', wraplength=340, justify='left', font=('Arial', 9))
        self.dfs_feedback_label.grid(row=1, column=0, columnspan=2, sticky='w')

        # Connection bar
        conn_frame = tk.Frame(self.root, bg='#34495e', relief='raised', bd=2)
        conn_frame.pack(pady=3, padx=20, fill='x')

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

        tk.Button(conn_frame, text="Restart", command=self.restart, bg='#7f8c8d',
                  fg='white', font=("Arial", 10, "bold"), width=12).grid(row=0, column=5, padx=10, pady=6)

        # Status indicators
        status_frame = tk.Frame(self.root, bg='#34495e', relief='raised', bd=2)
        status_frame.pack(pady=2, padx=20, fill='x')

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
                                           fg='#95a5a6', bg='#34495e', font=("Arial", 10, "bold"),
                                           wraplength=220, justify='left')
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

        tk.Label(status_frame, text="Roll:", fg='white', bg='#34495e',
                 font=("Arial", 10, "bold")).grid(row=0, column=10, padx=10, pady=8, sticky='w')
        self.roll_label = tk.Label(status_frame, text="N/A",
                                   fg='#3498db', bg='#34495e', font=("Arial", 10, "bold"))
        self.roll_label.grid(row=0, column=11, padx=5, pady=8, sticky='w')

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

        # Onboard mission reports have their own row, separate from robot status.
        mission_frame = tk.Frame(status_frame, bg='#34495e')
        mission_frame.grid(row=1, column=0, columnspan=14, padx=10, pady=(0, 8), sticky='w')
        for column, (caption, variable) in enumerate((
                ("Mission state:", self.mission_state), ("Reason:", self.mission_reason),
                ("Mode:", self.mission_mode), ("Mission motion:", self.mission_motion))):
            tk.Label(mission_frame, text=caption, fg='white', bg='#34495e',
                     font=("Arial", 10, "bold")).grid(row=0, column=2 * column, padx=(0, 5), sticky='w')
            label = tk.Label(mission_frame, textvariable=variable, fg='#95a5a6', bg='#34495e',
                             font=("Arial", 10, "bold"), wraplength=180, justify='left')
            label.grid(row=0, column=2 * column + 1, padx=(0, 20), sticky='w')
            if column == 0:
                self.mission_state_label = label
        leg_captions = (('Leg ID:', 'leg_id'), ('Departure:', 'departure_node'),
                        ('Next target marker:', 'next_target_node'), ('Reference:', 'arrival_reference_node'),
                        ('Expected cell:', 'target_cell'), ('Direction:', 'direction'),
                        ('Leg phase:', 'phase'), ('Configured:', 'configured'))
        for index, (caption, key) in enumerate(leg_captions):
            row, column = 1 + index // 4, (index % 4) * 2
            tk.Label(mission_frame, text=caption, fg='white', bg='#34495e',
                     font=("Arial", 9, "bold")).grid(row=row, column=column, padx=(0, 5), sticky='w')
            tk.Label(mission_frame, textvariable=self.mission_leg_fields[key],
                     fg='#f1c40f' if key == 'next_target_node' else '#bdc3c7', bg='#34495e',
                     font=("Arial", 10, "bold") if key == 'next_target_node' else ("Arial", 9),
                     wraplength=180).grid(row=row, column=column + 1, padx=(0, 20), sticky='w')

        # Compact ToF strip (one row of readings + one row of status) so the map gets the screen.
        tof_frame = tk.LabelFrame(self.root, text="ToF (mm)",
                                  fg='white', bg='#34495e', font=("Arial", 8, "bold"))
        tof_frame.pack(padx=20, pady=(2, 2), fill='x')
        self.tof_labels = {}
        self.tof_state_labels = {}
        for column, direction in enumerate(('left', 'front', 'right')):
            base = column * 3
            tof_frame.grid_columnconfigure(base + 1, weight=1)
            tk.Label(tof_frame, text=direction.title(), fg='white', bg='#34495e',
                     font=("Arial", 9, "bold")).grid(row=0, column=base, padx=(12, 4))
            self.tof_labels[direction] = tk.Label(
                tof_frame, text="--", fg='#95a5a6', bg='#34495e', font=("Consolas", 14, "bold"))
            self.tof_labels[direction].grid(row=0, column=base + 1, sticky='w')
            self.tof_state_labels[direction] = tk.Label(
                tof_frame, text="Waiting", fg='#95a5a6', bg='#34495e', font=("Arial", 9))
            self.tof_state_labels[direction].grid(row=0, column=base + 2, padx=(4, 12), sticky='w')
        self.tof_summary_label = tk.Label(
            tof_frame, text="Stop request: --   |   Blocked: --   |   Urgent: --",
            fg='#95a5a6', bg='#34495e', font=("Consolas", 9, "bold"))
        self.tof_summary_label.grid(row=1, column=0, columnspan=5, sticky='w', padx=12)
        self.tof_age_label = tk.Label(
            tof_frame, text="Waiting for ToF telemetry. Stop request is separate from the motor lock.",
            fg='#bdc3c7', bg='#34495e', font=("Arial", 8))
        self.tof_age_label.grid(row=1, column=5, columnspan=4, sticky='e', padx=12, pady=(0, 2))

        marker_frame = tk.Frame(self.root, bg='#34495e')
        marker_frame.pack(padx=20, pady=2, fill='x')
        for column, (title, value) in enumerate((
                ("Selected camera marker:", self.current_marker_text),
                ("Lens distance:", self.marker_distance_text),
                ("Bearing:", self.marker_bearing_text),
                ("Last reached:", self.last_reached_marker_text),
                ("Current node (RoboPico):", self.dfs_current_node_text))):
            tk.Label(marker_frame, text=title, fg='white', bg='#34495e').grid(
                row=0, column=column * 2, padx=(8, 4), sticky='w')
            tk.Label(marker_frame, textvariable=value, fg='#3498db', bg='#34495e',
                     font=("Arial", 10, "bold")).grid(
                row=0, column=column * 2 + 1, padx=(0, 12), sticky='w')
        tk.Label(marker_frame, textvariable=self.camera_relay_text, fg='#bdc3c7', bg='#34495e',
                 font=("Arial", 9), wraplength=1100, justify='left').grid(
                     row=3, column=0, columnspan=10, padx=8, pady=(0, 3), sticky='w')
        self.marker_confirmation_label = tk.Label(
            marker_frame, textvariable=self.marker_confirmation_text,
            fg='#bdc3c7', bg='#34495e', anchor='w')
        self.marker_confirmation_label.grid(row=1, column=0, columnspan=10,
                                            padx=8, pady=(0, 2), sticky='w')
        tk.Label(marker_frame, textvariable=self.node_confirmation_text,
                 fg='#bdc3c7', bg='#34495e', anchor='w', justify='left', wraplength=1100).grid(
                     row=2, column=0, columnspan=10, padx=8, pady=(0, 2), sticky='w')

        marker_list_frame = tk.Frame(marker_frame, bg='#34495e')
        marker_list_frame.grid(row=4, column=0, columnspan=10, sticky='ew', padx=8, pady=(0, 5))
        tk.Label(marker_list_frame, textvariable=self.camera_marker_count, fg='white', bg='#34495e',
                 font=("Arial", 10, "bold")).pack(side='left', anchor='n', padx=(0, 16), pady=3)
        style = ttk.Style(self.root)
        style.configure('Camera.Treeview', background='#2c3e50', fieldbackground='#2c3e50',
                        foreground='white', rowheight=20)
        self.camera_marker_table = ttk.Treeview(marker_list_frame, columns=('id', 'range', 'bearing', 'quality'),
                                               show='headings', height=1, selectmode='none', style='Camera.Treeview')
        for name, title, width in (('id', 'Marker ID', 90), ('range', 'Lens distance (mm)', 150),
                                   ('bearing', 'Bearing (deg)', 130), ('quality', 'Quality', 150)):
            self.camera_marker_table.heading(name, text=title)
            self.camera_marker_table.column(name, width=width, minwidth=width, anchor='center')
        self.camera_marker_table.tag_configure('unusable', foreground='#f39c12')
        self.camera_marker_table.pack(side='left', fill='x', expand=True)

        dfs_frame = tk.Frame(self.root, bg='#34495e')
        dfs_frame.pack(padx=20, pady=2, fill='x')
        tk.Label(dfs_frame, text="Start node ID:", fg='white', bg='#34495e').pack(side='left', padx=6)
        tk.Entry(dfs_frame, textvariable=self.start_node_id, width=4).pack(side='left')
        tk.Button(dfs_frame, text="Set start node", command=self.set_start_node,
                  bg='#2980b9', fg='white').pack(side='left', padx=6)
        tk.Label(dfs_frame, text="Finish: all reachable exits explored", fg='white', bg='#34495e').pack(side='left', padx=6)
        tk.Label(dfs_frame, text="Open exit ≥ (mm):", fg='white', bg='#34495e').pack(side='left', padx=6)
        tk.Entry(dfs_frame, textvariable=self.dfs_threshold, width=6).pack(side='left')
        tk.Label(dfs_frame, textvariable=self.start_node_status, fg='yellow', bg='#34495e', wraplength=450).pack(side='left', padx=12)

        select_frame = tk.Frame(self.root, bg='#34495e')
        select_frame.pack(padx=20, pady=2, fill='x')
        tk.Label(select_frame, text='RoboPico selects targets and tracks reference range. Arrival uses the configured empirical range rule.',
                 fg='#bdc3c7', bg='#34495e').grid(row=0, column=0, padx=6, sticky='w')
        tk.Label(select_frame, textvariable=self.target_selection_status, fg='#f1c40f', bg='#34495e',
                 wraplength=1200, justify='left').grid(row=1, column=0, columnspan=7, padx=6, sticky='w')
        tk.Label(select_frame, textvariable=self.dfs_tracking_text, fg='#3498db', bg='#34495e',
                 wraplength=1200, justify='left').grid(row=2, column=0, columnspan=7, padx=6, sticky='w')
        tk.Label(select_frame, textvariable=self.dfs_planner_text, fg='#2ecc71', bg='#34495e',
                 wraplength=1200, justify='left').grid(row=3, column=0, columnspan=7, padx=6, sticky='w')

        turn_frame = tk.Frame(self.root, bg='#34495e')
        turn_frame.pack(padx=20, pady=2, fill='x')
        tk.Label(turn_frame, text="Target heading (0–359°):", fg='white', bg='#34495e').pack(side='left', padx=6)
        turn_entry = tk.Entry(turn_frame, textvariable=self.turn_heading, width=6)
        turn_entry.pack(side='left')
        turn_entry.bind('<Return>', lambda event: self.request_manual_turn())
        tk.Button(turn_frame, text="Turn to heading", command=self.request_manual_turn,
                  bg='#2980b9', fg='white', font=("Arial", 10, "bold")).pack(side='left', padx=8)
        tk.Button(turn_frame, text="Stop turn", command=lambda: self.send_command("STOP"),
                  bg='#e67e22', fg='white').pack(side='left', padx=4)
        tk.Button(turn_frame, text="Set current heading to 0°", command=self.request_heading_zero,
                  bg='#8e44ad', fg='white').pack(side='left', padx=8)
        tk.Label(turn_frame, text="Absolute IMU heading; clockwise from the latest zero.",
                 fg='#bdc3c7', bg='#34495e').pack(side='left', padx=8)

        # Main area with 3 columns
        main = tk.Frame(self.root, bg='#2c3e50')
        main.pack(fill='both', expand=True, padx=20, pady=(4, 8))
        # Make left column slimmer to free space for map and logs
        main.grid_columnconfigure(0, weight=0, minsize=220)
        main.grid_columnconfigure(1, weight=10, minsize=800)   # the map is the main focus
        main.grid_columnconfigure(2, weight=1, minsize=300)
        main.grid_rowconfigure(0, weight=1)

        # LEFT COLUMN - Key mappings only (no movement buttons)
        left_col = tk.Frame(main, bg='#2c3e50')
        left_col.grid(row=0, column=0, sticky='nsew', padx=(0, 8))
        # Key mappings panel
        keymap_frame = tk.LabelFrame(left_col, text="Key Mappings", fg='white', bg='#34495e', font=("Arial", 12, "bold"))
        keymap_frame.pack(pady=6, fill='x')
        mappings = (
            "W/S - Move Forward/Backward\n"
            "A/D - Turn Left/Right\n"
            "Q/E - Strafe Left/Right\n"
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

        # Subsystem 4: tabs inside the Map frame. "Live view" keeps the original ToF placeholder.
        self.map_tabs = ttk.Notebook(self.map_frame)
        self.map_tabs.pack(fill='both', expand=True)
        live_tab = tk.Frame(self.map_tabs, bg='#2c3e50')
        self.map_tabs.add(live_tab, text="RoboPico map")
        tk.Label(live_tab, textvariable=self.onboard_map_status, fg='white', bg='#2c3e50',
                 wraplength=750).pack(fill='x', padx=10, pady=(5, 0))

        # !! ToF mapping display to be here
        self.fig = Figure(figsize=(6, 6), dpi=100, facecolor='#2c3e50')
        self.ax = self.fig.add_subplot(111, facecolor='#1a1a1a')
        # Empty view until an authoritative snapshot arrives. Coordinates are cell centres.
        self.ax.set_xlim(-0.5, 0.5)
        self.ax.set_ylim(-0.5, 0.5)
        self.ax.set_aspect('equal')
        self.ax.set_xticks([0])
        self.ax.set_yticks([0])
        self.ax.set_xlabel('Grid x (east)', color='white', fontsize=10)
        self.ax.set_ylabel('Grid y (north)', color='white', fontsize=10)
        self.ax.set_title('Waiting for RoboPico map', color='white', pad=15, fontsize=12)
        self.ax.tick_params(colors='white', labelsize=8)
        self.rover_dot, = self.ax.plot([], [], 'o', color='#f39c12', markersize=8, zorder=10)

        self.canvas = FigureCanvasTkAgg(self.fig, master=live_tab)
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
        self.front_obstacle_snapshot = None
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
            elif type(distance) is int and distance == 1023:
                text, state, color = ">1000 mm", "No target within 1 m", '#3498db'
            elif type(distance) is int and distance == 0 and faults is not None:
                text, state, color = "--", "No usable return", '#95a5a6'
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
        threshold = obstacle.get('forward_threshold_mm', 150)
        if type(threshold) is not int or not 1 <= threshold <= 1000:
            threshold = 150
        front = distances.get('front')
        self.front_obstacle_snapshot = (type(front) is int and 0 < front < threshold and
                                       not (faults is not None and faults & 2))
        stop_text = "YES" if stop is True else ("NO" if stop is False else "--")
        blocked_text = f"0x{blocked:02X}" if blocked is not None else "--"
        urgent_text = f"0x{urgent:02X}" if urgent is not None else "--"
        self.tof_summary_label.config(
            text=f"Stop request: {stop_text}   |   Blocked: {blocked_text}   |   Urgent: {urgent_text}",
            fg='#e74c3c' if stop is True else '#ecf0f1')
        age_text = f"{age_ms:.0f} ms" if age_known else "unknown"
        self.tof_age_label.config(
            text=f"Sensor packet age: {age_text}   |   No usable return = sensor responding, distance unknown; see each label. "
                 "Front stop disables W only; other movement keys remain available.")
        self.last_tof_rx = time.time()

    def set_front_blocked(self, blocked):
        changed = blocked != self.front_blocked
        self.front_blocked = blocked
        if blocked and changed:
            if self.active_command == "FORWARD":
                self.stop_hold()
            if self.navigator.active and self.navigator.phase == "drive":
                handler = getattr(self.navigator, 'handle_front_obstacle', lambda: False)
                if not handler():
                    grid = getattr(self.navigator, 'grid', None)
                    reason = ("Front obstacle: " + grid.confirmation_summary(self.navigator.clock())
                              if grid is not None else "Front obstacle")
                    self.navigator.stop(reason)
                    self.auto_mode.set(False)
            self.log("Front obstacle: W disabled; other keys remain available.")
        elif changed:
            self.forward_requires_release = False
            self.log("Front clear: W enabled. Press W again to drive forward.")

    def update_odometry(self, command, dt):
        """Update robot position based on movement command"""
        with self.odometry_lock:
            # Estimate velocity (mm/s) based on speed percentage
            speed_factor = self.current_speed.get() / 100.0
            base_velocity = 200.0 * speed_factor  # mm/s (adjust based on your robot)
            
            # (forward, right) parts of each move; heading is clockwise, so right = (cos, -sin).
            forward, right = {
                "FORWARD": (1, 0), "BACKWARD": (-1, 0), "LEFT": (0, -1), "RIGHT": (0, 1),
                "FRONT_LEFT": (0.707, -0.707), "FRONT_RIGHT": (0.707, 0.707),
                "BACK_LEFT": (-0.707, -0.707), "BACK_RIGHT": (-0.707, 0.707),
            }.get(command, (0, 0))
            heading_rad = math.radians(self.robot_heading)
            step = base_velocity * dt
            self.robot_x += step * (forward * math.sin(heading_rad) + right * math.cos(heading_rad))
            self.robot_y += step * (forward * math.cos(heading_rad) - right * math.sin(heading_rad))
            # Heading comes exclusively from healthy IMU telemetry. Rotation
            # commands provide no measured angle and must not overwrite it.

    def request_heading_zero(self):
        """Stop motion, then ask RoboPico to zero yaw after its braking interval."""
        if self.heading_zero_pending:
            self.log("Heading zero is already in progress.")
            return
        if not self.connected or self.robot_link != "live":
            self.log("Cannot zero heading: connect and wait for live telemetry.")
            return
        if self.pressed_keys:
            self.log("Release the control keys before zeroing the heading.")
            return
        sensor = self.navigator.sensor
        if (sensor is None or self.navigator.clock() - sensor.received_at + sensor.age_s >
                self.navigator.SENSOR_MAX_AGE_S):
            self.log("Cannot zero heading: wait for a fresh, healthy IMU reading.")
            return
        if self.navigator.active:
            self.navigator.stop("Setting current IMU heading to zero")
        self.auto_mode.set(False)
        self.cancel_heartbeat()
        self.active_command = None
        if not self.send_command("STOP", quiet=True):
            return
        self.heading_zero_pending = True
        if not self.send_command("ZERO_HEADING"):
            self.cancel_heading_zero()
            return
        self.heading_zero_job = self.root.after(5000, self.heading_zero_timeout)
        self.log("Stopping briefly, then setting the current direction to 0°. Keep the rover still.")

    def cancel_heading_zero(self):
        if self.heading_zero_job is not None:
            self.root.after_cancel(self.heading_zero_job)
        self.heading_zero_job = None
        self.heading_zero_pending = False

    def heading_zero_timeout(self):
        self.heading_zero_job = None
        self.cancel_heading_zero()
        self.send_command("STOP", quiet=True)
        self.log("Heading zero was not acknowledged. Check that RoboPico has the updated firmware.")

    def handle_heading_zero_status(self, status):
        if status in ("HEADING_ZERO_PENDING", "HEADING_ZERO_BUSY"):
            return
        self.cancel_heading_zero()
        if status == "HEADING_ZEROED":
            self.navigator.sensor = None
            self.navigator._sensor_timestamp = None
            self.navigator.visible_marker = None
            self.navigator._reset_arrival()
            with self.odometry_lock:
                self.robot_heading = 0.0
            self.heading_label.config(text="0.0°")
            self.turn_heading.set("0")
            self.log("Current direction is now 0°. Heading turns use this new zero; reacquire the marker before DFS.")
        elif status == "HEADING_ZERO_REFUSED":
            self.log("Heading zero refused: RoboPico needs a fresh, healthy IMU reading.")
        elif status == "HEADING_ZERO_CANCELLED":
            self.log("Heading zero cancelled by a stop/reset command.")

    def request_manual_turn(self):
        """Start one local Pico heading turn, without requiring a camera marker."""
        if getattr(self, 'heading_zero_pending', False):
            self.log("Wait for heading zero to finish before turning.")
            return
        try:
            heading = int(self.turn_heading.get().strip())
            if not 0 <= heading <= 359:
                raise ValueError
        except ValueError:
            self.log("Enter a whole-number target heading from 0 to 359 degrees.")
            return
        if not self.connected or self.robot_link != "live":
            self.log("Cannot turn: connect to the rover and wait for live telemetry.")
            return
        if self.estop_active:
            self.log("Cannot turn while E-STOP is active.")
            return
        if self.pressed_keys:
            self.log("Release the control keys before starting a heading turn.")
            return
        sensor = self.navigator.sensor
        if (sensor is None or self.navigator.clock() - sensor.received_at + sensor.age_s >
                self.navigator.SENSOR_MAX_AGE_S):
            self.log("Cannot turn: wait for a fresh, healthy IMU heading.")
            return
        if self.navigator.active:
            self.navigator.stop("Manual heading turn requested")
        self.auto_mode.set(False)
        self.cancel_heartbeat()
        self.active_command = None
        # STOP also cancels a previous manual turn and leaves Pico ready for a new target.
        if not self.send_command("STOP", quiet=True):
            return
        self.manual_turn_target = heading
        self.manual_turn_acked = False
        if not self.send_command("TURN_TO:%d" % heading):
            self.cancel_manual_turn()
            return
        # Longer than a DFS attempt: a manual turn has no automatic retry.
        self.manual_turn_job = self.root.after(15000, self.manual_turn_timeout)
        self.log("Turning to absolute IMU heading %d°. Stop turn or E-STOP cancels it." % heading)

    def cancel_manual_turn(self):
        if self.manual_turn_job is not None:
            self.root.after_cancel(self.manual_turn_job)
        self.manual_turn_job = None
        self.manual_turn_target = None
        self.manual_turn_acked = False

    def manual_turn_timeout(self):
        target = self.manual_turn_target
        self.manual_turn_job = None
        self.cancel_manual_turn()
        if target is not None:
            self.send_command("STOP", quiet=True)
            self.log("Turn to %d° timed out; STOP sent." % target)

    def handle_manual_turn_status(self, status):
        target = self.manual_turn_target
        if status == "AUTO_MOVE: TURN_TO:%d" % target:
            self.manual_turn_acked = True
        elif (status.startswith("AUTO_REFUSED: TURN_TO") or
              status in ("SAFETY_LOCKED", "SAFETY_READY", "READY") or
              (self.manual_turn_acked and (status.startswith("AUTO_RESULT:") or status == "STOPPED"))):
            self.cancel_manual_turn()
            if status == "AUTO_RESULT: HEADING_REACHED":
                self.log("Turn to %d° complete." % target)

    def toggle_auto_mode(self):
        requested = self.auto_mode.get()
        if requested:
            self.last_dfs_reply = None
        self.auto_mode.set(not requested) # Retain the last confirmed running state until the reply.
        if requested:
            try:
                opening=int(self.dfs_threshold.get().strip())
                if not 151<=opening<=1000:
                    raise ValueError('Opening threshold must be 151..1000 mm')
            except (TypeError,ValueError):
                self.dfs_feedback.set('Enter an opening threshold from 151 to 1000 mm.')
                self.dfs_feedback_label.config(fg='#e74c3c')
                return
            command='START_DFS:%d' % opening
        else:
            command='STOP_DFS'
        self.dfs_feedback.set('Waiting for RoboPico to start DFS...' if requested else 'Waiting for RoboPico to stop DFS...')
        self.dfs_feedback_label.config(fg='#f1c40f')
        if not self.send_command(command):
            self.dfs_feedback.set('Could not send the request. Connect to the broker and try again.')
            self.dfs_feedback_label.config(fg='#e74c3c')
        # Checkbox running state is confirmed by RoboPico replies/mission reports.

    def report_navigation(self, message):
        self.dfs_status.set(message)
        self.log('Dashboard monitor: ' + message)

    def set_start_node(self):
        if (self.active_command is not None or self.pressed_keys or
                self.manual_turn_target is not None or self.heading_zero_pending):
            self.log("Stop driving and release the keys before setting the starting node.")
            return
        try:
            marker_id = int(self.start_node_id.get())
            if not 1 <= marker_id <= 255:
                raise ValueError("Marker ID must be 1..255")
        except (TypeError, ValueError) as error:
            self.log("Cannot set starting node: " + str(error))
            return
        pending = self.pending_start_request
        if pending is not None:
            if marker_id != pending['marker_id']:
                self.log("Waiting for the previous start-node request; retry it or wait for RoboPico.")
                return
        else:
            token = uuid.uuid4().hex
            pending = {'marker_id': marker_id, 'token': token, 'command': 'SET_START:%s:%d' % (token, marker_id)}
        if self.send_command(pending['command']):
            self.pending_start_request = pending
            self.start_node_status.set('Requested node %d; waiting for RoboPico map confirmation' % marker_id)
        # No local map/current-node mutation. Only robot/map can change the view.

    def send_navigation_command(self, command):
        if command != 'STOP':
            if self.estop_active:
                self.log("Navigation command %s withheld: E-STOP active." % command)
                return False
            if self.robot_link != 'live':
                sensor = self.navigator.sensor
                age = (self.navigator.clock() - sensor.received_at + sensor.age_s
                       if sensor is not None else float('inf'))
                if sensor is not None and age <= self.navigator.SENSOR_MAX_AGE_S:
                    # A fresh packet may beat the 250 ms label watchdog. Use the
                    # validated telemetry now, rather than rejecting the command
                    # because the label still says Lost.
                    self.set_robot_link('live')
                else:
                    self.log("Navigation command %s withheld: robot telemetry unavailable/stale." % command)
                    return False
        return self.send_command(command, quiet=True, auto=True)

    def navigation_tick(self):
        if (self.front_obstacle_snapshot is not None and self.last_tof_rx is not None and
                time.time() - self.last_tof_rx <= TOF_UART_STALE_MS / 1000):
            self.set_front_blocked(self.front_obstacle_snapshot)
        else:
            self.set_front_blocked(False)
        self.navigator.tick()
        # Only RoboPico mission reports/replies own the onboard DFS checkbox.
        self.update_current_marker()
        self.root.after(100, self.navigation_tick)

    def update_current_marker(self):
        snapshot = self.onboard_map.snapshot
        current = snapshot.current_node if snapshot is not None else None
        suffix = '' if self.onboard_map.is_fresh(time.monotonic()) else ' (last reported)'
        self.dfs_current_node_text.set('--' if current is None else str(current) + suffix)
        last = self.onboard_map.last_confirmed_node
        self.last_reached_marker_text.set('--' if last is None else str(last))
        if self.rover_camera.seen:
            self.update_rover_camera_display()
        else:
            self.current_marker_text.set('--')
            self.marker_distance_text.set('--')
            self.marker_bearing_text.set('--')
            self.marker_confirmation_text.set('Waiting for RoboPico camera observations')

    def update_rover_camera_display(self):
        now = time.monotonic()
        marker, message, details, colour = self.rover_camera.display(now)
        self.update_camera_marker_list(now)
        self.current_marker_text.set("--" if marker is None else str(marker.id))
        self.marker_distance_text.set("--" if marker is None else "%d mm" % marker.range_mm)
        self.marker_bearing_text.set("--" if marker is None else "%+.2f°" % (marker.bearing_cdeg / 100))
        self.marker_confirmation_text.set(message)
        self.marker_confirmation_label.config(fg=colour)
        self.camera_relay_text.set(details)

    def update_camera_marker_list(self, now):
        markers = sorted(self.rover_camera.markers_in_view(now), key=lambda item: (
            item.range_mm if item.quality != 1 and item.range_mm > 0 else float('inf'), item.id))
        rows = tuple((marker.id,
                      str(marker.range_mm) if marker.quality != 1 and marker.range_mm > 0 else '--',
                      '%+.2f' % (marker.bearing_cdeg / 100) if marker.quality != 1 and marker.range_mm > 0 else '--',
                      QUALITY_NAMES[marker.quality]) for marker in markers)
        if rows == self.camera_marker_rows:
            return
        self.camera_marker_rows = rows
        self.camera_marker_count.set('Markers in view: %d' % len(rows))
        for item in self.camera_marker_table.get_children():
            self.camera_marker_table.delete(item)
        self.camera_marker_table.configure(height=max(1, len(rows)))
        for index, row in enumerate(rows):
            self.camera_marker_table.insert('', 'end', iid=str(index), values=row,
                tags=() if row[3] == 'OK' and row[1] != '--' else ('unusable',))

    def clear_rover_camera_display(self):
        self.rover_camera.clear()
        self.update_rover_camera_display()

    def receive_onboard_map(self, payload):
        try:
            data = json.loads(payload) if isinstance(payload, str) else payload
        except (TypeError, ValueError):
            return
        previous = self.onboard_map.snapshot
        if not self.onboard_map.update(data, time.monotonic()):
            return
        snapshot = self.onboard_map.snapshot
        if snapshot != previous:
            self.render_onboard_map()
        self.onboard_map_status.set('RoboPico map: %d nodes / %d capacity | %d columns x %d rows | current node: %s' % (
            len(snapshot.nodes), snapshot.capacity, snapshot.columns, snapshot.rows,
            snapshot.current_node if snapshot.current_node is not None else 'unknown'))
        pending = self.pending_start_request
        if pending is not None and snapshot.last_request == pending['token']:
            self.start_node_status.set('RoboPico confirmed start node %d at (0,0)' % pending['marker_id'])
            self.pending_start_request = None
            self.dfs_feedback.set('Start node confirmed. Wait for a target, then select Start DFS.')
            self.dfs_feedback_label.config(fg='#f1c40f')
        self.update_current_marker()

    def render_onboard_map(self):
        """Draw a validated received snapshot; never infer coordinates or arrivals."""
        snapshot = self.onboard_map.snapshot
        if snapshot is None:
            return
        self.ax.clear()
        self.ax.set_facecolor('#1a1a1a')
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_xlabel('Grid x (east)', color='white')
        self.ax.set_ylabel('Grid y (north)', color='white')
        self.ax.tick_params(colors='white')
        if not snapshot.nodes:
            self.ax.set_xlim(-0.5, 0.5)
            self.ax.set_ylim(-0.5, 0.5)
            self.ax.set_xticks([0])
            self.ax.set_yticks([0])
            self.ax.set_title('RoboPico map is empty; set the start node', color='white')
        else:
            min_x, max_x, min_y, max_y = snapshot.bounds
            self.ax.set_xlim(min_x - 0.5, max_x + 0.5)
            self.ax.set_ylim(min_y - 0.5, max_y + 0.5)
            for axis, low, high in ((self.ax.xaxis, min_x, max_x), (self.ax.yaxis, min_y, max_y)):
                if high - low < 32:
                    axis.set_ticks(list(range(low, high + 1)))
                    axis.set_ticks([i - 0.5 for i in range(low, high + 2)], minor=True)
                else:
                    axis.set_major_locator(MaxNLocator(nbins=8, integer=True))
            self.ax.grid(which='minor', color='#3498db', alpha=0.35)
            for node in snapshot.nodes:
                self.ax.add_patch(Rectangle((node.x - 0.5, node.y - 0.5), 1, 1,
                    facecolor='#27ae60' if node.visited else '#2980b9', edgecolor='#95a5a6', alpha=0.65))
                self.ax.text(node.x, node.y + 0.12, str(node.marker_id), ha='center', va='center',
                             color='white', fontsize=12, fontweight='bold')
            self.ax.set_title('RoboPico: %d columns x %d rows | %d discovered nodes' % (
                snapshot.columns, snapshot.rows, len(snapshot.nodes)), color='white')
        node = next((node for node in snapshot.nodes if node.marker_id == snapshot.current_node), None)
        self.rover_dot, = self.ax.plot([] if node is None else [node.x],
            [] if node is None else [node.y - 0.22], 'o', color='#f39c12', markersize=8, zorder=10)
        self.canvas.draw_idle()

    def update_grid_map(self):
        """Legacy local grid rendering is disabled; only robot/map owns the view."""

    def submit_vision_observation(self, message):
        """Queue B3's full schema-1 JSON object from an in-process camera worker."""
        self.root.after(0, self.navigator.vision_received, message)

    def update_marker_map(self, data):
        """Camera detections are observations, never authoritative map updates."""

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
        self.log_file.close()
        self.root.destroy()

    def restart(self):
        self.close()
        os.execv(sys.executable, [sys.executable] + sys.argv)

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
            self.last_refused = None
            self.log("Manual E-Stop released. Press a movement key to drive.")
        elif status == "SAFETY_RESET_REFUSED":
            self.log("Manual E-Stop reset refused.")
        else:
            self.log("Manual E-Stop active.")

    def trigger_estop(self):
        """Trigger emergency stop with 3-second lockout"""
        self.navigator.stop("Emergency stop")
        # Immediately send STOP command
        self.send_command("ESTOP")
        
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
                self.send_command("RESET_SAFETY", quiet=True)
                self.estop_status_label.config(text="✓ Ready", fg='#2ecc71', bg='#34495e')
        else:
            self.estop_status_label.config(text="✓ Ready", fg='#2ecc71', bg='#34495e')

    def start_hold(self, cmd):
        """Start driving with cmd and keep resending it while held"""
        if getattr(self, 'heading_zero_pending', False):
            self.log("Wait for heading zero to finish before moving.")
            return
        if cmd == "FORWARD" and self.front_blocked:
            self.log("W disabled: front obstacle. Other movement keys remain available.")
            return
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
                self.send_command("RESET_SAFETY", quiet=True)
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
        if ch == 'w':
            self.forward_requires_release = False

        cmd = KEYMAP.get(ch)
        if cmd is None or cmd != self.active_command:
            return  # not the key that is driving right now: nothing to do

        for other in self.pressed_keys:
            if other in KEYMAP and other not in self.release_jobs:
                if KEYMAP[other] == "FORWARD" and self.front_blocked:
                    continue
                self.start_hold(KEYMAP[other])
                return

        self.stop_hold()

    def update_mission_display(self, value):
        """Display authoritative rover state only; never issue a movement command."""
        try:
            report = parse_mission_report(value)
        except ValueError:
            return False
        previous = self.mission_report
        self.mission_report = report
        self.last_mission_rx = time.time()
        self.mission_state.set(report.state)
        self.mission_reason.set(report.reason)
        self.mission_mode.set(report.mode)
        self.mission_motion.set("Enabled" if report.motion_enabled else "Disabled")
        running = not report.test_mode and report.state == 'EXPLORING'
        self.auto_mode.set(running)
        if running:
            self.last_dfs_reply = None
            leg_phase=report.leg.phase if report.leg is not None else 'NONE'
            message={'HOLDING':'Waiting for fresh camera observations; wheels stopped.',
                'BRAKING':'RoboPico is verifying target arrival.', 'VERIFYING':'RoboPico is verifying target arrival.',
                'SCANNING':'RoboPico is checking ToF exits and planning the next DFS leg.',
                'TURNING':'RoboPico is turning toward the next DFS direction.',
                'SELECTING':'RoboPico is waiting for a reliable marker in the chosen exit.',
                'CONFIGURED':'RoboPico is preparing the next DFS travel leg.',
                'COMPLETE':'Node reached. RoboPico is continuing DFS.'}.get(leg_phase,'RoboPico is approaching the locked target.')
            self.dfs_feedback.set(message)
            self.dfs_feedback_label.config(fg='#2ecc71')
        elif report.state == 'COMPLETE' and report.reason == 'DFS_COMPLETE':
            self.dfs_feedback.set('DFS complete: all reachable mapped exits have been explored.')
            self.dfs_feedback_label.config(fg='#2ecc71')
        elif report.state == 'COMPLETE' and report.reason == 'TARGET_ARRIVED':
            self.dfs_feedback.set('RoboPico confirmed arrival at the locked target.')
            self.dfs_feedback_label.config(fg='#2ecc71')
        elif previous is not None and previous.state == 'EXPLORING' and not previous.test_mode:
            if self.last_dfs_reply is not None and self.last_dfs_reply.error:
                self.dfs_feedback.set(self.last_dfs_reply.message)
                self.dfs_feedback_label.config(fg='#e74c3c')
            else:
                self.dfs_feedback.set('Onboard approach stopped: ' + report.reason)
                self.dfs_feedback_label.config(fg='#f39c12')
        leg_values = report.leg.display_fields if report.leg is not None else {}
        for key, variable in self.mission_leg_fields.items():
            variable.set(leg_values.get(key, 'Unknown' if key == 'phase' else '--'))
        colours = {"IDLE": '#95a5a6', "READY": '#27ae60', "EXPLORING": '#3498db',
                   "PAUSED": '#f39c12', "COMPLETE": '#27ae60', "FAULT": '#e74c3c'}
        self.mission_state_label.config(fg=colours[report.state])
        if previous is None or previous.display_values != report.display_values:
            leg_text = '' if report.leg is None else ' | leg: ' + ', '.join('%s=%s' % item for item in leg_values.items())
            self.log("Mission: %s | %s | %s | mission motion %s%s" % (
                report.state, report.reason, report.mode, self.mission_motion.get().lower(), leg_text))
        return True

    def clear_mission_display(self, state="Unknown"):
        self.mission_report = None
        self.last_mission_rx = None
        self.last_mission_reply = None
        self.mission_state.set(state)
        self.mission_reason.set("--")
        self.mission_mode.set("Unknown")
        self.mission_motion.set("--")
        for key, variable in self.mission_leg_fields.items():
            variable.set('Unknown' if key == 'phase' else '--')
        self.target_selection_report = None
        self.target_selection_status.set('Waiting for RoboPico selection report')
        self.dfs_tracking_text.set('Tracking: waiting for RoboPico')
        self.dfs_tracking_report = None
        self.dfs_planner_report = None
        self.dfs_planner_text.set('DFS: waiting for RoboPico')
        self.last_dfs_reply = None
        self.auto_mode.set(False)
        self.dfs_feedback.set('RoboPico mission status unavailable. Reconnect before starting DFS.')
        self.dfs_feedback_label.config(fg='#e74c3c')
        self.mission_state_label.config(fg='#95a5a6')

    def watchdog_tick(self):
        if self.connected:
            if self.onboard_map.snapshot is not None and not self.onboard_map.is_fresh(time.monotonic()):
                self.onboard_map_status.set('RoboPico map reports stale; displaying last received map')
            last_mission = getattr(self, 'last_mission_rx', None)
            if last_mission is not None and time.time() - last_mission > ROBOT_LOST_S:
                self.clear_mission_display("Stale")
            if self.last_tof_rx is not None and time.time() - self.last_tof_rx > ROBOT_LOST_S:
                self.clear_tof_readings("Telemetry stale")
            # Time since the last telemetry. If none yet, count from connect.
            ref = self.last_telemetry_rx or self.connect_time
            age = time.time() - ref
            colour = '#95a5a6' if age > 0.5 else '#3498db'
            self.heading_label.config(fg=colour)
            self.pitch_label.config(fg=colour)
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
            local_turn_or_settle = (self.navigator.active and
                                   self.navigator.phase in ("turn", "scan_turn", "settle", "scan_settle", "arrival_settle", "input_wait", "align"))
            if local_turn_or_settle:
                self.log("RoboPico telemetry late: waiting for local turn result/fresh inputs; forward travel disabled.")
            else:
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
            
            self.mqtt_client = make_mqtt_client(self.client_id)
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
        if self.heading_zero_pending:
            self.send_command("STOP", quiet=True)
        if self.manual_turn_target is not None:
            self.send_command("STOP", quiet=True)
        self.navigator.stop("Dashboard disconnected")
        if self.active_command:
            self.stop_hold()
        self.stop_link_heartbeat()
        self.cancel_heartbeat()
        if self.mqtt_client:
            client = self.mqtt_client
            self.mqtt_client = None
            # Stop handing new messages to Tk, then stop paho without blocking the window
            client.on_message = None
            client.on_connect = None
            client.on_disconnect = None
            client.disconnect()
            threading.Thread(target=client.loop_stop, daemon=True).start()
        self.connected = False
        self.broker_connected.set("Not Connected")
        self.broker_status_label.config(fg='#e74c3c')
        self.last_telemetry_rx = None
        self.set_robot_link("unknown")
        self.clear_tof_readings("Disconnected")
        self.robot_status.set("Unknown")
        self.robot_status_label.config(fg='#95a5a6')
        self.clear_mission_display()
        self.clear_rover_camera_display()
        self.onboard_map.received_at = None
        self.onboard_map_status.set('Disconnected; retaining the last RoboPico map')
        self.robot_ip.set("N/A")
        self.connect_btn.config(text="Connect", bg='#27ae60')
        self.log("Disconnected from MQTT broker")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(TOPIC_STATUS)
            client.subscribe(TOPIC_TELEMETRY)
            client.subscribe(TOPIC_MARKER)
            client.subscribe(TOPIC_ROVER_CAMERA)
            client.subscribe(TOPIC_ROVER_MAP)
        self.root.after(0, self.handle_mqtt_connect, rc)

    def handle_mqtt_connect(self, rc):
        if rc == 0:
            self.broker_connected.set("Connected")
            self.broker_status_label.config(fg='#27ae60')
            self.log("Connected to MQTT broker")
            self.log(f"Subscribed to {TOPIC_STATUS}, {TOPIC_TELEMETRY}, {TOPIC_MARKER}, {TOPIC_ROVER_CAMERA}, {TOPIC_ROVER_MAP}")
            self.start_link_heartbeat()
            self.send_command('MAP_GET', quiet=True)
        else:
            self.log(f"MQTT connection failed (code {rc})")
            self.broker_connected.set("Failed")
            self.broker_status_label.config(fg='#e74c3c')

    def on_mqtt_message(self, client, userdata, msg):
        # Replayed broker state cannot constitute a live camera/sensor observation.
        if msg.retain and msg.topic in (TOPIC_STATUS, TOPIC_TELEMETRY, TOPIC_MARKER, TOPIC_ROVER_CAMERA, TOPIC_ROVER_MAP):
            return
        payload = msg.payload.decode(errors="replace")
        self.root.after(0, self.handle_mqtt_message, msg.topic, payload)

    def handle_mqtt_message(self, topic, payload):
        ts = datetime.now().strftime("%H:%M:%S")
        self.last_msg_time.set(ts)
        self.msg_count += 1

        if topic == TOPIC_STATUS:
            if payload.startswith('DFS_STATE:'):
                try:
                    planner=parse_dfs_state(payload[len('DFS_STATE:'):])
                except ValueError:
                    return
                if self.dfs_planner_report!=planner:
                    self.log(f'[{ts}] RoboPico ' + planner.display_text)
                self.dfs_planner_report=planner
                self.dfs_planner_text.set(planner.display_text)
                return
            if payload.startswith('DFS_TRACKING:'):
                try:
                    tracking = parse_dfs_tracking(payload[len('DFS_TRACKING:'):])
                except ValueError:
                    return
                previous_tracking = self.dfs_tracking_report
                self.dfs_tracking_report = tracking
                self.dfs_tracking_text.set(tracking.display_text)
                if previous_tracking is None or previous_tracking.change_key != tracking.change_key:
                    self.log(f'[{ts}] RoboPico tracking: ' + tracking.log_text)
                return
            if payload.startswith(('DFS_ACK:', 'DFS_REFUSED:', 'DFS_EVENT:')):
                try:
                    reply = parse_dfs_reply(payload)
                except ValueError:
                    return
                self.auto_mode.set(reply.running)
                self.last_dfs_reply = reply
                self.dfs_feedback.set(reply.message)
                self.dfs_feedback_label.config(fg='#e74c3c' if reply.error else '#2ecc71')
                self.log('RoboPico DFS: ' + reply.message)
                return
            if payload.startswith('TARGET_SELECTION:'):
                try:
                    selection = parse_target_selection(payload[len('TARGET_SELECTION:'):])
                except ValueError:
                    return
                previous = self.target_selection_report
                self.target_selection_report = selection
                self.target_selection_status.set(selection.display_text)
                if previous is None or previous.change_key != selection.change_key:
                    self.log('RoboPico target: ' + selection.display_text)
                return
            if payload.startswith(('TARGET_ACK:', 'TARGET_REFUSED:')):
                self.log(payload)
                return
            if payload.startswith(('MAP_ACK:', 'MAP_REFUSED:')):
                self.log(payload)
                if payload.startswith('MAP_REFUSED:'):
                    self.start_node_status.set(payload)
                    self.pending_start_request = None
                return
            if payload.startswith("MISSION_STATE:"):
                self.update_mission_display(payload)
                return
            if payload.startswith(("MISSION_TEST:", "MISSION_REFUSED:")):
                if payload != self.last_mission_reply:
                    self.log(f"[{ts}] Mission: {payload}")
                    self.last_mission_reply = payload
                return
            if payload.startswith("HEADING_ZERO_") or payload == "HEADING_ZEROED":
                self.handle_heading_zero_status(payload)
            if getattr(self, 'manual_turn_target', None) is not None:
                self.handle_manual_turn_status(payload)
            self.navigator.robot_status_received(payload)
            self.robot_status.set(payload)
            
            if payload in {"FRONT_STOPPED", "FORWARD_BLOCKED"}:
                self.robot_status_label.config(fg='#f39c12')
                # Status messages can arrive after a newer clear range. Never
                # latch W from an old status; ranges own the forward guard.
                if self.active_command == "FORWARD":
                    self.stop_hold()
            elif payload in {"SAFETY_LOCKED", "SAFETY_RESET_REFUSED", "SAFETY_READY"}:
                self.robot_status_label.config(
                    fg='#27ae60' if payload == "SAFETY_READY" else '#e74c3c')
                self.handle_safety_status(payload)
            elif payload == "READY":
                self.navigator.stop("RoboPico restarted")
                self.robot_status_label.config(fg='#27ae60')
                self.log(f"[{ts}] Robot ready")
            elif payload == "STOPPED":
                if self.stop_sent_at is not None:
                    ms = (time.perf_counter() - self.stop_sent_at) * 1000
                    if ms < 2000:
                        self.stop_latencies.append(ms)
                        s_ = sorted(self.stop_latencies)
                        self.log(f"[{ts}] STOP confirmed in {ms:.0f} ms round trip "
                                 f"(n={len(s_)}, min {s_[0]:.0f}, median {s_[len(s_)//2]:.0f}, max {s_[-1]:.0f})")
                if self.robot_status != "SAFETY_LOCKED":
                    self.robot_status_label.config(fg='#f39c12')
                    self.log(f"[{ts}] Robot stopped")
            elif payload == "RUNNING":
                self.robot_status_label.config(fg='#3498db')
            elif payload == "ERROR":
                self.navigator.stop("RoboPico command error")
                self.robot_status_label.config(fg='#e74c3c')
                self.log(f"[{ts}] ⚠ Robot error")
            elif payload.startswith("STOPPED: LINK LOST"):
                self.navigator.stop("RoboPico lost the laptop link")
                self.robot_status_label.config(fg='#e74c3c')
                self.log(f"[{ts}] ⚠ Rover stopped itself: laptop link lost (FR-08)")
            elif payload == "RECONNECTED":
                self.robot_status_label.config(fg='#27ae60')
                self.log(f"[{ts}] Rover reconnected to the broker (FR-12)")
            elif payload.startswith("PITCH:"):
                pass
            elif payload.startswith("SAFE STOP"):
                self.navigator.stop("Motion Control SAFE STOP")
                self.robot_status_label.config(fg='#e74c3c')
                self.log(f"[{ts}] ⚠ {payload}")
            elif payload.startswith("REFUSED"):
                self.robot_status_label.config(fg='#e74c3c')
                if payload != getattr(self, 'last_refused', None):   # log once, not every 300 ms
                    self.log(f"[{ts}] ⚠ {payload}")
                self.last_refused = payload
            elif payload.startswith("AUTO:"):
                # Autonomous mode status messages
                self.robot_status_label.config(fg='#f39c12')
                self.log(f"[{ts}] {payload}")
            elif payload in ["AUTO_ENABLED", "AUTO_DISABLED", "AUTO_STOPPED", "AUTO_MOVING"]:
                # Legacy auto status
                self.robot_status_label.config(fg='#f39c12')
                self.log(f"[{ts}] {payload}")
            else:
                self.log(f"[{ts}] Status: {payload}")
        elif topic == TOPIC_MARKER:
            try:
                data = json.loads(payload)
                if not isinstance(data, dict):
                    raise ValueError("Expected an object")
                if "schema" in data or "markers" in data:
                    self.navigator.vision_received(data)
                    self.update_marker_map(data)
                else:
                    self.navigator.marker_seen(data["marker_id"], data["frame_id"], data.get("age_ms", 0),
                                               data.get("distance_mm"), data.get("bearing_deg"))
            except (KeyError, TypeError, ValueError):
                self.log("Rejected invalid camera marker event")

        elif topic == TOPIC_ROVER_MAP:
            self.receive_onboard_map(payload)
        elif topic == TOPIC_ROVER_CAMERA:
            try:
                data = json.loads(payload)
            except (TypeError, ValueError):
                return
            if self.rover_camera.update(data, time.monotonic()):
                self.update_rover_camera_display()
        elif topic == TOPIC_TELEMETRY:
            try:
                data = json.loads(payload)
                if not isinstance(data, dict):
                    return
                if "mission" in data:
                    self.update_mission_display(data["mission"])
                self.navigator.sensor_received(data)
                self.update_tof_readings(data.get("obstacle"))
                imu = data.get("imu", {})
                ultrasonic = data.get("ultrasonic", {})

                # Heading is clockwise relative to Sensor Pico startup.
                heading = imu.get("heading", None)
                pitch = imu.get("pitch", None)
                roll = imu.get("roll", None)
                imu_ok = imu.get("status", 0) == 0
                
                if heading is not None and imu_ok:
                    with self.odometry_lock:
                        self.robot_heading = float(heading)
                    # Update heading display
                    self.heading_label.config(text=f"{self.robot_heading:.1f}°")
                
                if pitch is not None and imu_ok:
                    self.robot_pitch = float(pitch)
                    # Update pitch display
                    self.pitch_label.config(text=f"{self.robot_pitch:.1f}°")

                if roll is not None and imu_ok:
                    self.roll_label.config(text=f"{float(roll):.1f}°")

                if not imu_ok:
                    self.heading_label.config(text="N/A")
                    self.pitch_label.config(text="N/A")
                    self.roll_label.config(text="N/A")

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
        if self.heading_zero_pending:
            self.cancel_heading_zero()
        if self.manual_turn_target is not None:
            self.cancel_manual_turn()
        self.navigator.stop("MQTT connection lost")
        self.clear_tof_readings("Disconnected")
        self.clear_mission_display()
        self.clear_rover_camera_display()
        self.onboard_map.received_at = None
        self.onboard_map_status.set('Disconnected; retaining the last RoboPico map')
        self.log(f"MQTT disconnected (code {rc})")
        self.broker_connected.set("Disconnected")
        self.broker_status_label.config(fg='#e74c3c')

    def send_command(self, command, quiet=False, auto=False):
        if getattr(self, 'heading_zero_pending', False):
            if command in ("STOP", "ESTOP", "RESET_SAFETY", "AUTO_STOP"):
                self.cancel_heading_zero()
            elif command != "ZERO_HEADING":
                self.log("Wait for heading zero to finish before moving.")
                return False
        if getattr(self, 'manual_turn_target', None) is not None:
            if command in KEYMAP.values():
                # A fresh keyboard command takes over only after stopping the turn.
                if not self.send_command("STOP", quiet=True):
                    return False
            elif command in ("STOP", "ESTOP", "RESET_SAFETY"):
                self.cancel_manual_turn()
        if not self.connected or not self.mqtt_client:
            self.log("Not connected to broker")
            return False
        
        try:
            ts = datetime.now().strftime("%H:%M:%S")
            # Movement speeds are set on the robot. TURN_TO carries its heading.
            # DFS forward goes out as MOVE_FORWARD: an autonomous move (FR-40) that
            # the RoboPico keeps going itself and can tell apart from the W key.
            # The navigator still sees "FORWARD" below for its grid tracking.
            payload = "MOVE_FORWARD" if (auto and command == "FORWARD") else command

            if not quiet:
                self.log(f"[{ts}] → {command}")
            if command == "STOP":
                self.stop_sent_at = time.perf_counter()

            result = self.mqtt_client.publish(TOPIC_COMMAND, payload, qos=0)
            
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                self.log(f"Publish failed (code {result.rc})")
                return False
            observer = getattr(self.navigator, 'motion_received', None)
            if observer is not None:
                observer(command)
            return True
            
        except Exception as e:
            self.log(f"Send failed: {e}")
            return False

    # Removed no-op connection monitor

    def log(self, message):
        self.log_file.write(f"{datetime.now().strftime('%H:%M:%S.%f')[:-3]}  {message}\n")
        self.log_file.flush()
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
