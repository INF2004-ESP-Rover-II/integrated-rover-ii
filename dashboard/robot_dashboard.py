#!/usr/bin/env python3
"""
MQTT Robot Controller Dashboard - With RPLIDAR 2D Visualization
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

HEARTBEAT_MS = 300

# MQTT Topics
TOPIC_COMMAND = "robot/command"
TOPIC_STATUS = "robot/status"
TOPIC_TELEMETRY = "robot/telemetry"
TOPIC_LIDAR = "robot/lidar"

class RobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Mecanum Robot Controller - MQTT + LIDAR")
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
        
        # E-Stop state
        self.estop_active = False
        self.estop_time = 0
        self.estop_cooldown = 3.0  # 3 seconds cooldown

        # Speed control
        self.current_speed = tk.IntVar(value=80)
        
        # Individual wheel speed adjustments (0-100%)
        self.speed_fl = tk.IntVar(value=80)  # Front Left
        self.speed_fr = tk.IntVar(value=80)  # Front Right
        self.speed_bl = tk.IntVar(value=80)  # Back Left
        self.speed_br = tk.IntVar(value=80)  # Back Right

        # Robot odometry state
        self.robot_x = 0.0  # mm
        self.robot_y = 0.0  # mm
        self.robot_heading = 0.0  # degrees (calibrated)
        self.robot_pitch = 0.0  # degrees
        self.last_telemetry_time = time.time()
        self.odometry_lock = threading.Lock()

        # Ultrasonic distances (mm)
        self.ultrasonic = {'front': 0, 'back': 0, 'left': 0, 'right': 0}

        # LIDAR data storage - World coordinates
        self.max_lidar_points = 2000  # Keep more points for persistent map
        self.world_lidar_x = deque(maxlen=self.max_lidar_points)
        self.world_lidar_y = deque(maxlen=self.max_lidar_points)
        self.world_lidar_quality = deque(maxlen=self.max_lidar_points)
        self.world_lidar_angle = deque(maxlen=self.max_lidar_points)  # Store original angle for coloring
        # timestamps for each LIDAR point (seconds since epoch) to allow time-based pruning
        self.world_lidar_ts = deque(maxlen=self.max_lidar_points)
        self.world_lidar_layer = deque(maxlen=self.max_lidar_points)  # Layer index for multi-layer scanning
        self.lidar_lock = threading.Lock()

        # Multi-layer scanning state
        self.scanning_mode = False  # True when in multi-layer scan mode
        self.current_layer = 0  # Current layer being scanned
        self.layer_colors = ['blue', 'green', 'red', 'yellow', 'cyan', 'magenta', 'orange', 'purple']
        
        # Permanent scan storage (not subject to retention time)
        self.scan_points_x = []
        self.scan_points_y = []
        self.scan_points_layer = []
        self.scan_points_angle = []
        
        # View mode: 'livemap' or '3d'
        self.view_mode = 'livemap'

        # How long (seconds) to keep LIDAR points for realtime view
        self.lidar_retention_s = tk.DoubleVar(value=3.0)

        # Map following mode
        self.follow_robot = tk.BooleanVar(value=True)
        
        # Autonomous mode
        self.auto_mode = tk.BooleanVar(value=False)

        # No on-screen control buttons (keyboard-only controls)

        self.setup_ui()

    def setup_ui(self):
        # Header
        header = tk.Frame(self.root, bg='#2c3e50')
        header.pack(fill='x', pady=(10, 0))
        header.grid_columnconfigure(0, weight=1)
        
        tk.Label(header, text="MECANUM ROBOT CONTROLLER - MQTT + LIDAR",
                 font=("Arial", 20, "bold"), fg='white', bg='#2c3e50').grid(row=0, column=0, padx=20, sticky='w')
        
        # Back to Live Map button (top right, hidden initially)
        self.back_to_livemap_btn = tk.Button(header, text="◄ Back to Live Map", 
                                             command=self.back_to_livemap,
                                             bg='#3498db', fg='white', font=("Arial", 12, "bold"))
        # Don't grid yet, will show when entering 3D mode
        
        # Autonomous mode toggle (top right)
        self.auto_btn = tk.Checkbutton(header, text="AUTO EXPLORATION", variable=self.auto_mode,
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

        # LIDAR display
        tk.Label(status_frame, text="LIDAR:", fg='white', bg='#34495e',
                 font=("Arial", 10, "bold")).grid(row=0, column=10, padx=10, pady=8, sticky='w')
        self.lidar_points_label = tk.Label(status_frame, text="0",
                                           fg='#3498db', bg='#34495e', font=("Arial", 10, "bold"))
        self.lidar_points_label.grid(row=0, column=11, padx=5, pady=8, sticky='w')

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
            "[ - Start/Stop Layer Scan\n"
            "] - Next Layer\n"
            "\n"
            "Space/X - Emergency Stop"
        )
        tk.Label(keymap_frame, text=mappings, justify='left', anchor='w', fg='white', bg='#34495e', font=("Consolas", 10)).pack(fill='x', padx=8, pady=8)

        # Control sliders panel
        sliders_frame = tk.LabelFrame(left_col, text="Control Settings", fg='white', bg='#34495e', font=("Arial", 12, "bold"))
        sliders_frame.pack(pady=6, fill='x')
        
        # LIDAR retention slider
        tk.Label(sliders_frame, text="LIDAR Retention (s):", bg='#34495e', fg='white', font=("Arial", 9)).pack(anchor='w', padx=8, pady=(8,2))
        ret_slider = tk.Scale(sliders_frame, from_=0.5, to=10.0, resolution=0.5, orient='horizontal',
                              variable=self.lidar_retention_s, bg='#34495e', fg='white', length=200)
        ret_slider.pack(fill='x', padx=8, pady=(0,8))
        
        # Wheel speed adjustments
        tk.Label(sliders_frame, text="Wheel Speed Adjustment (%):", bg='#34495e', fg='white', font=("Arial", 9, "bold")).pack(anchor='w', padx=8, pady=(8,4))
        
        # Front Left
        tk.Label(sliders_frame, text="Front Left:", bg='#34495e', fg='white', font=("Arial", 8)).pack(anchor='w', padx=8)
        fl_slider = tk.Scale(sliders_frame, from_=0, to=100, resolution=1, orient='horizontal',
                            variable=self.speed_fl, bg='#34495e', fg='white', length=200)
        fl_slider.pack(fill='x', padx=8, pady=(0,4))
        
        # Front Right
        tk.Label(sliders_frame, text="Front Right:", bg='#34495e', fg='white', font=("Arial", 8)).pack(anchor='w', padx=8)
        fr_slider = tk.Scale(sliders_frame, from_=0, to=100, resolution=1, orient='horizontal',
                            variable=self.speed_fr, bg='#34495e', fg='white', length=200)
        fr_slider.pack(fill='x', padx=8, pady=(0,4))
        
        # Back Left
        tk.Label(sliders_frame, text="Back Left:", bg='#34495e', fg='white', font=("Arial", 8)).pack(anchor='w', padx=8)
        bl_slider = tk.Scale(sliders_frame, from_=0, to=100, resolution=1, orient='horizontal',
                            variable=self.speed_bl, bg='#34495e', fg='white', length=200)
        bl_slider.pack(fill='x', padx=8, pady=(0,4))
        
        # Back Right
        tk.Label(sliders_frame, text="Back Right:", bg='#34495e', fg='white', font=("Arial", 8)).pack(anchor='w', padx=8)
        br_slider = tk.Scale(sliders_frame, from_=0, to=100, resolution=1, orient='horizontal',
                            variable=self.speed_br, bg='#34495e', fg='white', length=200)
        br_slider.pack(fill='x', padx=8, pady=(0,8))

        # # Multi-layer scanning panel
        # scan_frame = tk.LabelFrame(left_col, text="Multi-Layer Scan", fg='white', bg='#34495e', font=("Arial", 12, "bold"))
        # scan_frame.pack(pady=6, fill='x')
        
        # self.scan_status_label = tk.Label(scan_frame, text="Status: Idle", bg='#34495e', fg='white', font=("Arial", 10, "bold"))
        # self.scan_status_label.pack(anchor='w', padx=8, pady=(8,2))
        
        # self.layer_label = tk.Label(scan_frame, text="Layer: 0", bg='#34495e', fg='#3498db', font=("Arial", 10, "bold"))
        # self.layer_label.pack(anchor='w', padx=8, pady=(2,4))
        
        # self.points_label = tk.Label(scan_frame, text="Scan Points: 0", bg='#34495e', fg='#3498db', font=("Arial", 9))
        # self.points_label.pack(anchor='w', padx=8, pady=(0,8))
        
        # tk.Button(scan_frame, text="Clear All Layers", command=self.clear_scan_data,
        #          bg='#e74c3c', fg='white', font=("Arial", 9, "bold")).pack(fill='x', padx=8, pady=(0,8))


        # MIDDLE COLUMN - LIDAR Visualization
        middle_col = tk.Frame(main, bg='#2c3e50')
        middle_col.grid(row=0, column=1, sticky='nsew', padx=8)

        self.lidar_frame = tk.LabelFrame(middle_col, text="RPLIDAR A1 - World Map (Robot Centered)",
                                    fg='white', bg='#34495e', font=("Arial", 14, "bold"))
        self.lidar_frame.pack(fill='both', expand=True)

        # Create matplotlib figure for LIDAR (supports both 2D and 3D)
        self.fig = Figure(figsize=(6, 6), dpi=100, facecolor='#2c3e50')
        
        # Create 2D axes (default view)
        self.ax_2d = self.fig.add_subplot(111, facecolor='#1a1a1a')
        
        # Style the 2D plot
        self.ax_2d.set_xlim(-3000, 3000)  # 3 meters range on X axis
        self.ax_2d.set_ylim(-3000, 3000)  # 3 meters range on Y axis
        self.ax_2d.set_aspect('equal')
        self.ax_2d.grid(True, color='#3498db', alpha=0.3, linewidth=0.5)
        self.ax_2d.set_facecolor('#1a1a1a')
        
        # Initial empty scatter plot for LIDAR points
        self.scatter = self.ax_2d.scatter([], [], c=[], s=3, cmap='hot', alpha=0.7, vmin=0, vmax=50)
        
        # Robot marker (always at center when following)
        self.robot_marker, = self.ax_2d.plot(0, 0, 'go', markersize=8, label='Robot', zorder=10)
        # Tiny forward arrow (updated per view)
        self.dir_arrow = self.ax_2d.arrow(0, 0, 0, 150, head_width=60, head_length=80,
                           fc='lime', ec='lime', alpha=0.9, zorder=12)

        # Sector boundary lines (four lines every 90° starting at 45°)
        from matplotlib.lines import Line2D
        self.sector_lines = []
        for _ in range(4):
            ln = Line2D([], [], color='#8e44ad', linewidth=1.0, alpha=0.6)
            self.ax_2d.add_line(ln)
            self.sector_lines.append(ln)
        
        # Configure 2D labels
        self.ax_2d.set_xlabel('X (mm)', color='white', fontsize=10)
        self.ax_2d.set_ylabel('Y (mm)', color='white', fontsize=10)
        self.ax_2d.set_title('LIDAR World Map - Robot Centered View', color='white', pad=15, fontsize=12)
        self.ax_2d.tick_params(colors='white', labelsize=8)
        
        # Legend for sector colors
        try:
            legend_elements = [
                Line2D([0], [0], marker='o', color='w', label='Back (blue)', markerfacecolor='blue', markersize=6),
                Line2D([0], [0], marker='o', color='w', label='Right (green)', markerfacecolor='green', markersize=6),
                Line2D([0], [0], marker='o', color='w', label='Front (red)', markerfacecolor='red', markersize=6),
                Line2D([0], [0], marker='o', color='w', label='Left (yellow)', markerfacecolor='yellow', markersize=6),
            ]
            self.ax_2d.legend(handles=legend_elements, loc='upper right', facecolor='#2c3e50', edgecolor='#2c3e50', labelcolor='white')
        except Exception:
            pass
        
        # 3D axes (created on demand)
        self.ax_3d = None
        self.scatter_3d = None
        
        # Active axes reference
        self.ax = self.ax_2d
        
        # Embed in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.lidar_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill='both', expand=True, padx=10, pady=10)

        # Sector coloring enabled by default (no toggle)
        self.color_sectors = tk.BooleanVar(value=True)

        # RIGHT COLUMN - Log
        right_col = tk.Frame(main, bg='#2c3e50')
        right_col.grid(row=0, column=2, sticky='nsew', padx=(8, 0))
        right_col.grid_rowconfigure(0, weight=1)
        right_col.grid_columnconfigure(0, weight=1)

        log_frame = tk.LabelFrame(right_col, text="Command & Status Log",
                                  fg='white', bg='#34495e')
        log_frame.grid(row=0, column=0, sticky='nsew')
        self.log_text = tk.Text(log_frame, height=30, bg='#2c3e50', fg='#ecf0f1', 
                               font=("Consolas", 9), wrap='word')
        scrollbar = tk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Global key handling
        self.root.bind_all('<KeyPress>', self.on_key_press, add="+")
        self.root.bind_all('<KeyRelease>', self.on_key_release, add="+")
        self.root.focus_set()

        # Start animation loop for LIDAR
        self.animation = FuncAnimation(self.fig, self.update_lidar_plot, 
                                      interval=200, blit=False, cache_frame_data=False)

        # Initialize E-Stop UI
        self.update_estop_ui()

        self.log("Dashboard ready. Enter broker IP and click Connect.")

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

    def transform_lidar_to_world(self, angle_deg, distance_mm):
        """Transform LIDAR point from robot frame to world frame"""
        with self.odometry_lock:
            # Convert LIDAR angle to radians
            lidar_angle_rad = math.radians(angle_deg)
            
            # Robot's heading in radians
            robot_heading_rad = math.radians(self.robot_heading)
            
            # LIDAR point in robot frame
            robot_frame_x = distance_mm * math.sin(lidar_angle_rad)
            robot_frame_y = distance_mm * math.cos(lidar_angle_rad)
            
            # Rotate by robot heading and translate to world frame
            world_x = (robot_frame_x * math.cos(robot_heading_rad) - 
                      robot_frame_y * math.sin(robot_heading_rad) + self.robot_x)
            world_y = (robot_frame_x * math.sin(robot_heading_rad) + 
                      robot_frame_y * math.cos(robot_heading_rad) + self.robot_y)
            
            return world_x, world_y

    def update_lidar_plot(self, frame):
        """Animation callback to update LIDAR visualization"""
        try:
            # Skip animation if in 3D mode
            if self.view_mode == '3d':
                return (self.scatter_3d,) if hasattr(self, 'scatter_3d') and self.scatter_3d else ()
            
            with self.lidar_lock:
                # prune old points based on retention time
                now = time.time()
                cutoff = now - float(self.lidar_retention_s.get())
                while self.world_lidar_ts and self.world_lidar_ts[0] < cutoff:
                    self.world_lidar_x.popleft()
                    self.world_lidar_y.popleft()
                    self.world_lidar_quality.popleft()
                    self.world_lidar_angle.popleft()
                    self.world_lidar_ts.popleft()
                    self.world_lidar_layer.popleft()

                if len(self.world_lidar_x) == 0:
                    # Update robot marker and arrow even if no points
                    if self.follow_robot.get():
                        self.robot_marker.set_data([0], [0])
                        try:
                            if self.dir_arrow:
                                self.dir_arrow.remove()
                        except Exception:
                            pass
                        self.dir_arrow = self.ax.arrow(0, 0, 0, 150, head_width=60, head_length=80,
                                                       fc='lime', ec='lime', alpha=0.9, zorder=12)
                    else:
                        with self.odometry_lock:
                            self.robot_marker.set_data([self.robot_x], [self.robot_y])
                            try:
                                if self.dir_arrow:
                                    self.dir_arrow.remove()
                            except Exception:
                                pass
                            heading_rad = math.radians(self.robot_heading)
                            dx = 150 * math.sin(heading_rad)
                            dy = 150 * math.cos(heading_rad)
                            self.dir_arrow = self.ax.arrow(self.robot_x, self.robot_y, dx, dy,
                                                           head_width=60, head_length=80,
                                                           fc='lime', ec='lime', alpha=0.9, zorder=12)
                    return self.scatter, self.robot_marker
                
                # Get world coordinates
                x_world = np.array(list(self.world_lidar_x))
                y_world = np.array(list(self.world_lidar_y))
                qualities = np.array(list(self.world_lidar_quality))
                angles = np.array(list(self.world_lidar_angle))
                layers = np.array(list(self.world_lidar_layer))
                
                # Color by layer if in scanning mode, otherwise use sector coloring
                if self.scanning_mode or any(layers >= 0):
                    # Multi-layer mode: color by layer
                    colors = []
                    for layer in layers:
                        if layer < 0:
                            colors.append('white')  # Non-scanned points
                        else:
                            color_idx = layer % len(self.layer_colors)
                            colors.append(self.layer_colors[color_idx])
                elif self.color_sectors.get():
                    # Sector coloring (90° sectors) if enabled
                    colors = []
                    for angle in angles:
                        norm_angle = angle % 360
                        if norm_angle <= 45 or norm_angle >= 315:
                            colors.append('blue')   # back
                        elif 45 < norm_angle <= 135:
                            colors.append('yellow') # left (swap)
                        elif 135 < norm_angle <= 225:
                            colors.append('red')    # front
                        else:
                            colors.append('green')  # right (swap)
                else:
                    colors = ['white'] * len(angles)
                
                if self.follow_robot.get():
                    # Robot-centered view: translate to robot and rotate by -heading
                    with self.odometry_lock:
                        rx, ry = self.robot_x, self.robot_y
                        heading_rad = math.radians(self.robot_heading)
                    # Translate world -> robot-local
                    x_local = x_world - rx
                    y_local = y_world - ry
                    # Rotate by -heading so robot faces up (positive Y on plot)
                    cos_h = math.cos(heading_rad)
                    sin_h = math.sin(heading_rad)
                    x_rotated = x_local * cos_h + y_local * sin_h
                    y_rotated = -x_local * sin_h + y_local * cos_h
                    # Flip 180° so the robot's back (blue) appears at the bottom
                    x_rotated = -x_rotated
                    y_rotated = -y_rotated
                    # Robot marker always at origin
                    self.robot_marker.set_data([0], [0])
                    # Refresh tiny forward arrow straight up from origin
                    try:
                        if self.dir_arrow:
                            self.dir_arrow.remove()
                    except Exception:
                        pass
                    self.dir_arrow = self.ax.arrow(0, 0, 0, 150, head_width=60, head_length=80,
                                                   fc='lime', ec='lime', alpha=0.9, zorder=12)
                else:
                    # Fixed world view
                    x_rotated = x_world
                    y_rotated = y_world
                    
                    with self.odometry_lock:
                        # Update robot marker at actual position
                        self.robot_marker.set_data([self.robot_x], [self.robot_y])
                        # Refresh tiny forward arrow in heading direction
                        try:
                            if self.dir_arrow:
                                self.dir_arrow.remove()
                        except Exception:
                            pass
                        heading_rad = math.radians(self.robot_heading)
                        dx = 150 * math.sin(heading_rad)
                        dy = 150 * math.cos(heading_rad)
                        self.dir_arrow = self.ax.arrow(self.robot_x, self.robot_y, dx, dy,
                                                       head_width=60, head_length=80,
                                                       fc='lime', ec='lime', alpha=0.9, zorder=12)
                
                # Update scatter plot with color-coded points
                self.scatter.set_offsets(np.c_[x_rotated, y_rotated])
                self.scatter.set_color(colors)

                # Update sector boundary lines relative to robot-local view
                # In robot-centered view, forward is up (0°). Boundaries at 45°, 135°, 225°, 315°.
                try:
                    r = 3000
                    boundary_degs = [45, 135, 225, 315]
                    for i, deg in enumerate(boundary_degs):
                        rad = math.radians(deg)
                        dx = r * math.sin(rad)
                        dy = r * math.cos(rad)
                        # Lines start at origin; extend to edge
                        self.sector_lines[i].set_data([0, dx], [0, dy])
                except Exception:
                    pass
                # Ensure canvas refresh
                try:
                    self.canvas.draw_idle()
                except Exception:
                    pass
                
                # Update point count only
                self.root.after(0, lambda: self.lidar_points_label.config(
                    text=str(len(x_world))))
            
            return self.scatter, self.robot_marker
            
        except Exception as e:
            return self.scatter, self.robot_marker

    def toggle_auto_mode(self):
        """Toggle autonomous mode on/off"""
        if self.auto_mode.get():
            self.send_command("AUTO_START")
            self.log("Autonomous mode enabled")
        else:
            self.send_command("AUTO_STOP")
            self.log("Autonomous mode disabled")

    def clear_lidar_data(self):
        """Clear live LIDAR data only (not scanned data)"""
        with self.lidar_lock:
            self.world_lidar_x.clear()
            self.world_lidar_y.clear()
            self.world_lidar_quality.clear()
            self.world_lidar_angle.clear()
            self.world_lidar_ts.clear()
            self.world_lidar_layer.clear()
        
        gc.collect()
        self.log("Live LIDAR map cleared")

    def clear_scan_data(self):
        """Clear all scanned data and reset to live map"""
        self.scan_points_x.clear()
        self.scan_points_y.clear()
        self.scan_points_layer.clear()
        self.scan_points_angle.clear()
        
        # Reset scanning state
        self.scanning_mode = False
        self.current_layer = 0
        self.view_mode = 'livemap'
        
        # Back to live map view
        self.back_to_livemap()
        
        self.update_scan_ui()
        gc.collect()
        self.log("All scan data cleared - returned to live map")

    def toggle_scanning_mode(self):
        """Toggle multi-layer scanning mode on/off"""
        self.scanning_mode = not self.scanning_mode
        if self.scanning_mode:
            self.current_layer = 0
            self.log(f"Multi-layer scanning STARTED - Layer {self.current_layer} (front 90° only)")
            self.log("Points will be saved permanently for 3D view")
        else:
            # Scanning stopped - switch to 3D view
            self.log(f"Multi-layer scanning STOPPED - {len(self.scan_points_x)} points collected")
            self.show_3d_view()
        self.update_scan_ui()

    def next_layer(self):
        """Advance to next layer in multi-layer scan"""
        if not self.scanning_mode:
            self.log("Not in scanning mode - press [ to start")
            return
        
        self.current_layer += 1
        if self.current_layer >= len(self.layer_colors):
            self.log(f"Warning: Layer {self.current_layer} exceeds color palette, wrapping colors")
        
        self.log(f"Advanced to Layer {self.current_layer} - Move arm up and continue scanning")
        self.update_scan_ui()

    def update_scan_ui(self):
        """Update the scanning status UI"""
        if self.scanning_mode:
            self.scan_status_label.config(text="Status: SCANNING", fg='#27ae60')
            color_idx = self.current_layer % len(self.layer_colors)
            color = self.layer_colors[color_idx]
            self.layer_label.config(text=f"Layer: {self.current_layer} ({color})")
        else:
            self.scan_status_label.config(text="Status: Idle", fg='white')
            self.layer_label.config(text="Layer: 0")
        
        # Update scan points count
        self.points_label.config(text=f"Scan Points: {len(self.scan_points_x)}")

    def show_3d_view(self):
        """Switch to 3D point cloud view"""
        if len(self.scan_points_x) == 0:
            self.log("No scan data to visualize in 3D")
            return
        
        self.view_mode = '3d'
        self.log("Switching to 3D point cloud view...")
        
        # Clear the figure and create 3D axes
        self.fig.clear()
        self.ax_3d = self.fig.add_subplot(111, projection='3d', facecolor='#1a1a1a')
        self.ax = self.ax_3d
        
        # Style 3D plot
        self.ax_3d.set_facecolor('#1a1a1a')
        self.ax_3d.set_xlabel('X (mm)', color='white', fontsize=10)
        self.ax_3d.set_ylabel('Y (mm)', color='white', fontsize=10)
        self.ax_3d.set_zlabel('Layer (Height)', color='white', fontsize=10)
        self.ax_3d.set_title('3D Multi-Layer Scan Point Cloud', color='white', pad=15, fontsize=12)
        self.ax_3d.tick_params(colors='white', labelsize=8)
        self.ax_3d.grid(True, color='#3498db', alpha=0.3)
        
        # Prepare data
        x = np.array(self.scan_points_x)
        y = np.array(self.scan_points_y)
        z = np.array(self.scan_points_layer)
        
        # Color by layer
        colors = [self.layer_colors[int(layer) % len(self.layer_colors)] for layer in z]
        
        # Plot 3D scatter
        self.scatter_3d = self.ax_3d.scatter(x, y, z, c=colors, s=5, alpha=0.8)
        
        # Set better viewing angle to see points more clearly (not in corner)
        # elev=20 (elevation), azim=-60 (azimuth) positions camera for better view
        self.ax_3d.view_init(elev=20, azim=-60)
        
        # Update frame title
        self.lidar_frame.config(text="3D Point Cloud - Multi-Layer Scan")
        
        # Show back button in header (hide auto button)
        self.auto_btn.grid_forget()
        self.back_to_livemap_btn.grid(row=0, column=1, padx=20, sticky='e')
        
        self.canvas.draw()
        self.log(f"3D view displayed with {len(x)} points across {max(z)+1 if len(z) > 0 else 0} layers")

    def back_to_livemap(self):
        """Return to live map 2D view"""
        self.view_mode = 'livemap'
        self.log("Returning to live map view...")
        
        # Clear 3D scatter and axes references
        if hasattr(self, 'scatter_3d') and self.scatter_3d is not None:
            self.scatter_3d = None
        if hasattr(self, 'ax_3d') and self.ax_3d is not None:
            self.ax_3d = None
        
        # Clear figure and recreate 2D axes
        self.fig.clear()
        self.ax_2d = self.fig.add_subplot(111, facecolor='#1a1a1a')
        self.ax = self.ax_2d
        
        # Restore 2D plot style
        self.ax_2d.set_xlim(-3000, 3000)
        self.ax_2d.set_ylim(-3000, 3000)
        self.ax_2d.set_aspect('equal')
        self.ax_2d.grid(True, color='#3498db', alpha=0.3, linewidth=0.5)
        self.ax_2d.set_facecolor('#1a1a1a')
        
        # Recreate scatter plot
        self.scatter = self.ax_2d.scatter([], [], c=[], s=3, cmap='hot', alpha=0.7, vmin=0, vmax=50)
        
        # Robot marker
        self.robot_marker, = self.ax_2d.plot(0, 0, 'go', markersize=8, label='Robot', zorder=10)
        self.dir_arrow = self.ax_2d.arrow(0, 0, 0, 150, head_width=60, head_length=80,
                           fc='lime', ec='lime', alpha=0.9, zorder=12)
        
        # Sector lines
        from matplotlib.lines import Line2D
        self.sector_lines = []
        for _ in range(4):
            ln = Line2D([], [], color='#8e44ad', linewidth=1.0, alpha=0.6)
            self.ax_2d.add_line(ln)
            self.sector_lines.append(ln)
        
        # Labels
        self.ax_2d.set_xlabel('X (mm)', color='white', fontsize=10)
        self.ax_2d.set_ylabel('Y (mm)', color='white', fontsize=10)
        self.ax_2d.set_title('LIDAR World Map - Robot Centered View', color='white', pad=15, fontsize=12)
        self.ax_2d.tick_params(colors='white', labelsize=8)
        
        # Legend
        try:
            legend_elements = [
                Line2D([0], [0], marker='o', color='w', label='Back (blue)', markerfacecolor='blue', markersize=6),
                Line2D([0], [0], marker='o', color='w', label='Right (green)', markerfacecolor='green', markersize=6),
                Line2D([0], [0], marker='o', color='w', label='Front (red)', markerfacecolor='red', markersize=6),
                Line2D([0], [0], marker='o', color='w', label='Left (yellow)', markerfacecolor='yellow', markersize=6),
            ]
            self.ax_2d.legend(handles=legend_elements, loc='upper right', facecolor='#2c3e50', edgecolor='#2c3e50', labelcolor='white')
        except Exception:
            pass
        
        # Update frame title
        self.lidar_frame.config(text="RPLIDAR A1 - World Map (Robot Centered)")
        
        # Hide back button, restore auto button
        self.back_to_livemap_btn.grid_forget()
        self.auto_btn.grid(row=0, column=1, padx=20, sticky='e')
        
        self.canvas.draw()
        self.log("Live map view restored")

    def reset_odometry(self):
        """Reset robot position to origin"""
        with self.odometry_lock:
            self.robot_x = 0.0
            self.robot_y = 0.0
            self.robot_heading = 0.0
        # reset ultrasonic readings too
        self.ultrasonic = {'front': 0, 'back': 0, 'left': 0, 'right': 0}
        self.log("Robot position reset to origin")

    def trigger_estop(self):
        """Trigger emergency stop with 3-second lockout"""
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

    # Removed unused button helpers and pitch setter (keyboard-only UI)

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
            
            self.send_command(self.active_command)
            self.schedule_heartbeat()

    def cancel_heartbeat(self):
        if self.heartbeat_job:
            self.root.after_cancel(self.heartbeat_job)
            self.heartbeat_job = None

    # Removed reset_all_buttons (no on-screen movement buttons to manage)

    def on_key_press(self, event):
        ch = event.char.lower() if event.char else ''
        
        # Always allow emergency stop (triggers E-Stop button)
        if ch in {'x', ' '}:
            if ch in self.pressed_keys:
                return "break"
            self.pressed_keys.add(ch)
            self.trigger_estop()
            return "break"
        
        # Check E-Stop lockout for movement commands
        if self.estop_active:
            time_since_estop = time.time() - self.estop_time
            if time_since_estop < 3.0:
                # Movement blocked during lockout
                if ch in {'w', 'a', 's', 'd', 'q', 'e', 'z', 'c', 'r', 't', 'f', 'g', 'o', 'i', 'k'}:
                    self.log(f"Movement blocked - E-Stop lockout ({3.0 - time_since_estop:.1f}s remaining)")
                    return "break"
            else:
                # Lockout expired, clear E-Stop
                self.estop_active = False
                self.update_estop_ui()
                self.log("E-Stop lockout expired - movement enabled")
        
        # Multi-layer scanning controls (work in any mode)
        if ch == '[':
            if ch in self.pressed_keys:
                return "break"
            self.pressed_keys.add(ch)
            self.toggle_scanning_mode()
            return "break"
        
        if ch == ']':
            if ch in self.pressed_keys:
                return "break"
            self.pressed_keys.add(ch)
            self.next_layer()
            return "break"
        
        # Block all other keys when autonomous mode is active
        if self.auto_mode.get():
            return "break"
        
        keymap = {
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
        
        if ch in keymap:
            if ch in self.pressed_keys:
                return "break"
            self.pressed_keys.add(ch)
            cmd = keymap[ch]
            if self.active_command and self.active_command != cmd:
                self.stop_hold()
            self.active_command = cmd
            self.last_command_time = time.time()
            self.send_command(cmd)
            self.schedule_heartbeat()
            return "break"

    def on_key_release(self, event):
        ch = event.char.lower() if event.char else ''
        if ch in self.pressed_keys:
            self.pressed_keys.remove(ch)
        # Only stop movement keys (including diagonals), not arm/claw (those are momentary)
        if ch in {'w','a','s','d','q','e','r','t','f','g', 'o','c','i','k'}:
            self.stop_hold()
            return "break"

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
            self.connect_btn.config(text="Disconnect", bg='#e74c3c')
            self.root.focus_set()
            
        except Exception as e:
            self.log(f"Connection failed: {e}")
            messagebox.showerror("Connection Error", f"Failed to connect:\n{e}")
            self.connected = False

    def disconnect(self):
        self.cancel_heartbeat()
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            self.mqtt_client = None
        self.connected = False
        self.broker_connected.set("Not Connected")
        self.broker_status_label.config(fg='#e74c3c')
        self.robot_connected.set("Unknown")
        self.robot_conn_label.config(fg='#95a5a6')
        self.robot_status.set("Unknown")
        self.robot_status_label.config(fg='#95a5a6')
        self.robot_ip.set("N/A")
        self.connect_btn.config(text="Connect", bg='#27ae60')
        self.log("Disconnected from MQTT broker")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.broker_connected.set("Connected")
            self.broker_status_label.config(fg='#27ae60')
            self.log("Connected to MQTT broker")
            client.subscribe(TOPIC_STATUS)
            client.subscribe(TOPIC_TELEMETRY)
            client.subscribe(TOPIC_LIDAR)
            # Wildcard subscribe to catch any robot/* topics (defensive)
            client.subscribe("robot/#")
            self.log(f"Subscribed to {TOPIC_STATUS}, {TOPIC_TELEMETRY}, {TOPIC_LIDAR} and robot/#")
        else:
            self.log(f"MQTT connection failed (code {rc})")
            self.broker_connected.set("Failed")
            self.broker_status_label.config(fg='#e74c3c')

    def on_mqtt_message(self, client, userdata, msg):
        payload = msg.payload.decode()
        ts = datetime.now().strftime("%H:%M:%S")
        self.last_msg_time.set(ts)
        self.msg_count += 1
        
        if msg.topic == TOPIC_STATUS:
            self.robot_status.set(payload)
            self.robot_connected.set("Connected")
            self.robot_conn_label.config(fg='#27ae60')
            
            if payload == "READY":
                self.robot_status_label.config(fg='#27ae60')
                self.log(f"[{ts}] Robot ready")
            elif payload == "STOPPED":
                self.robot_status_label.config(fg='#f39c12')
                self.log(f"[{ts}] Robot stopped")
            elif payload == "RUNNING":
                self.robot_status_label.config(fg='#3498db')
            elif payload == "ERROR":
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
                
        elif msg.topic == TOPIC_TELEMETRY:
            try:
                data = json.loads(payload)
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

                self.robot_connected.set("Connected")
                self.robot_conn_label.config(fg="#27ae60")

            except json.JSONDecodeError:
                pass
                
        elif msg.topic == TOPIC_LIDAR:
            try:
                lidar_data = json.loads(payload)
                # Debug: log number of points received
                # pts = len(lidar_data.get("points", [])) if isinstance(lidar_data, dict) else 0
                # self.log(f"[{ts}] LIDAR rx: {pts} points")
                self.process_lidar_data(lidar_data)
            except json.JSONDecodeError:
                self.log(f"[{ts}] LIDAR rx: JSON decode error")
            except Exception as e:
                self.log(f"[{ts}] LIDAR rx: error {e}")

    def process_lidar_data(self, data):
        """Process incoming LIDAR data and transform to world coordinates"""
        try:
            with self.lidar_lock:
                if "points" not in data:
                    return
                
                points = data["points"]
                if not points:
                    return
                
                # Transform each point to world coordinates
                for point in points:
                    angle = point.get("angle", 0)
                    distance = point.get("distance", 0)
                    quality = point.get("quality", 20)
                    
                    # Filter out invalid readings
                    if distance > 10 and distance < 8000:
                        # If in scanning mode, only capture front 50° sector (155° to 205°)
                        # Front sector is centered at 180°, so 155° to 205° covers the front 50°
                        norm_angle = angle % 360
                        in_front_sector = (155 <= norm_angle <= 205)
                        
                        # If scanning, only process front sector points
                        if self.scanning_mode:
                            if in_front_sector:
                                # Save to permanent scan storage
                                world_x, world_y = self.transform_lidar_to_world(angle, distance)
                                self.scan_points_x.append(world_x)
                                self.scan_points_y.append(world_y)
                                self.scan_points_layer.append(self.current_layer)
                                self.scan_points_angle.append(angle)
                        
                        # Always add to live view (unless in 3D mode)
                        if self.view_mode == 'livemap':
                            world_x, world_y = self.transform_lidar_to_world(angle, distance)
                            self.world_lidar_x.append(world_x)
                            self.world_lidar_y.append(world_y)
                            self.world_lidar_quality.append(quality)
                            self.world_lidar_angle.append(angle)
                            self.world_lidar_ts.append(time.time())
                            self.world_lidar_layer.append(self.current_layer if self.scanning_mode else -1)
                
        except Exception as e:
            self.log(f"[ERROR] LIDAR processing failed: {e}")

    def on_mqtt_disconnect(self, client, userdata, rc):
        if rc != 0:
            self.log(f"MQTT disconnected (code {rc})")
            self.broker_connected.set("Disconnected")
            self.broker_status_label.config(fg='#e74c3c')
            self.robot_connected.set("Unknown")
            self.robot_conn_label.config(fg='#95a5a6')

    def send_command(self, command):
        if not self.connected or not self.mqtt_client:
            self.log("Not connected to broker")
            return
        
        try:
            ts = datetime.now().strftime("%H:%M:%S")
            speed = self.current_speed.get()
            # Include individual wheel speed adjustments
            fl = self.speed_fl.get()
            fr = self.speed_fr.get()
            bl = self.speed_bl.get()
            br = self.speed_br.get()
            payload = f"{command}:{speed}:{fl}:{fr}:{bl}:{br}"
            
            self.log(f"[{ts}] → {command} @ {speed}% [FL:{fl} FR:{fr} BL:{bl} BR:{br}]")
            result = self.mqtt_client.publish(TOPIC_COMMAND, payload, qos=0)
            
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                self.log(f"Publish failed (code {result.rc})")
            
        except Exception as e:
            self.log(f"Send failed: {e}")
            self.disconnect()

    # Removed no-op connection monitor

    def log(self, message):
        self.log_text.insert(tk.END, message + '\n')
        self.log_text.see(tk.END)
        self.root.update_idletasks()

def main():
    root = tk.Tk()
    app = RobotGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()