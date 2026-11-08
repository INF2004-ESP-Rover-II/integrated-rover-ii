"""Original blue dashboard layout, with an onboard-only map panel.

The window layout follows the supplied RobotGUI reference. No laptop navigator,
DFS engine, occupancy inference or route executor is instantiated here.
"""
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.patches import Circle
from matplotlib.ticker import MultipleLocator
import math

BG = "#2c3e50"
PANEL = "#34495e"


class RestoredDashboardUI:
    def setup_ui(self):
        # Header
        header = tk.Frame(self.root, bg='#2c3e50')
        header.pack(fill='x', pady=(4, 0))
        header.grid_columnconfigure(0, weight=1)
        
        tk.Label(header, text="ROVER II CONTROLLER - MQTT + TOF",
                 font=("Arial", 20, "bold"), fg='white', bg='#2c3e50').grid(row=0, column=0, padx=20, sticky='w')
                
        # Autonomous mode toggle (top right)
        self.auto_btn = tk.Checkbutton(header, text="ONBOARD DFS", variable=self.auto_mode,
                      command=self.toggle_auto_mode,
                      bg='#2c3e50', fg='yellow', selectcolor='#1a1a1a',
                      font=("Arial", 12, "bold"))
        self.auto_btn.grid(row=0, column=1, padx=20, sticky='e')

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
                ("Current camera marker:", self.current_marker_text),
                ("Lens distance:", self.marker_distance_text),
                ("Bearing:", self.marker_bearing_text),
                ("Last reached:", self.last_reached_marker_text),
                ("DFS current node:", self.dfs_current_node_text))):
            tk.Label(marker_frame, text=title, fg='white', bg='#34495e').grid(
                row=0, column=column * 2, padx=(8, 4), sticky='w')
            tk.Label(marker_frame, textvariable=value, fg='#3498db', bg='#34495e',
                     font=("Arial", 10, "bold")).grid(
                row=0, column=column * 2 + 1, padx=(0, 12), sticky='w')
        self.marker_confirmation_label = tk.Label(
            marker_frame, textvariable=self.marker_confirmation_text,
            fg='#bdc3c7', bg='#34495e', anchor='w')
        self.marker_confirmation_label.grid(row=1, column=0, columnspan=10,
                                            padx=8, pady=(0, 2), sticky='w')
        tk.Label(marker_frame, textvariable=self.node_confirmation_text,
                 fg='#bdc3c7', bg='#34495e', anchor='w', justify='left', wraplength=1100).grid(
                     row=2, column=0, columnspan=10, padx=8, pady=(0, 2), sticky='w')

        dfs_frame = tk.Frame(self.root, bg='#34495e')
        dfs_frame.pack(padx=20, pady=2, fill='x')
        tk.Label(dfs_frame, text="Start node ID:", fg='white', bg='#34495e').pack(side='left', padx=6)
        tk.Entry(dfs_frame, textvariable=self.start_node_id, width=4).pack(side='left')
        tk.Button(dfs_frame, text="Set start node", command=self.set_start_node,
                  bg='#2980b9', fg='white').pack(side='left', padx=6)
        tk.Label(dfs_frame, text="DFS target markers (1–36):", fg='white', bg='#34495e').pack(side='left', padx=6)
        tk.Entry(dfs_frame, textvariable=self.dfs_target, width=4).pack(side='left')
        tk.Label(dfs_frame, text="Side opening ≥ (mm):", fg='white', bg='#34495e').pack(side='left', padx=6)
        tk.Entry(dfs_frame, textvariable=self.dfs_threshold, width=6).pack(side='left')
        tk.Label(dfs_frame, textvariable=self.dfs_status, fg='yellow', bg='#34495e').pack(side='left', padx=12)

        turn_frame = tk.Frame(self.root, bg='#34495e')
        turn_frame.pack(padx=20, pady=2, fill='x')
        tk.Label(turn_frame, text="Target heading (0–359°):", fg='white', bg='#34495e').pack(side='left', padx=6)
        turn_entry = tk.Entry(turn_frame, textvariable=self.turn_heading, width=6)
        turn_entry.pack(side='left')
        turn_entry.bind('<Return>', lambda event: self.request_manual_turn())
        tk.Button(turn_frame, text="Turn to heading", command=self.request_manual_turn, state="disabled",
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
        main.grid_columnconfigure(1, weight=10, minsize=480)   # the map is the main focus
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
        self.map_tabs.add(live_tab, text="Live view")
        self.mapping = OnboardMapPanel(self.map_tabs, self)
        self.map_tabs.add(self.mapping, text="Mapping & path")
        self.map_tabs.select(self.mapping)

        # Onboard map snapshots are authoritative in both views.
        self.fig = Figure(figsize=(6, 6), dpi=100, facecolor='#2c3e50')
        self.ax = self.fig.add_subplot(111, facecolor='#1a1a1a')
        # Initial 6x6 grid of 60 cm cells; expand for markers beyond its edges.
        self.ax.set_xlim(-.5, 5.5)
        self.ax.set_ylim(-.5, 5.5)
        self.ax.set_aspect('equal')
        self.ax.set_xticks(range(6))
        self.ax.set_yticks(range(6))
        self.ax.grid(True, color='#3498db', alpha=0.6, linewidth=1)
        self.ax.set_xlabel('Column (60 cm cells)', color='white', fontsize=10)
        self.ax.set_ylabel('Row (60 cm cells)', color='white', fontsize=10)
        self.ax.set_title('Set start node ID to begin grid mapping', color='white', pad=15, fontsize=12)
        self.ax.tick_params(colors='white', labelsize=8)
        self.rover_dot, = self.ax.plot(0, 0,
                                      'go', markersize=10, zorder=10)

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
        self.log_text = tk.Text(log_frame, height=30, width=32, bg='#2c3e50', fg='#ecf0f1',
                               font=("Consolas", 9), wrap='word', state='disabled')
        scrollbar = tk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Global key handling
        self.root.bind_all('<KeyPress>', self.on_key_press, add="+")
        self.root.bind_all('<KeyRelease>', self.on_key_release, add="+")
        self.root.focus_set()
        self.root.bind("<FocusOut>", self.on_focus_out, add="+")

        # Initialize E-Stop UI
        self.estop_status_label.config(text="Awaiting rover status")

        self.log("Dashboard ready. Enter broker IP and click Connect.")

        # Start watchdog timer


class OnboardMapPanel(tk.Frame):
    """Same map/path tab layout, displaying only the map received from the Pico."""

    def __init__(self, parent, dashboard):
        super().__init__(parent, bg=PANEL)
        self.dashboard = dashboard
        self.status = dashboard.location
        self.selected = None
        tk.Label(self, textvariable=self.status, fg="#f1c40f", bg=PANEL,
                 font=("Arial", 11, "bold"), anchor="w", wraplength=780).pack(
                     fill="x", padx=8, pady=(6, 2))
        row = tk.Frame(self, bg=PANEL)
        row.pack(fill="x", padx=6, pady=2)
        self.target_var = tk.StringVar()
        tk.Label(row, text="Target marker (or click one)", fg="white", bg=PANEL).pack(side="left", padx=4)
        tk.Entry(row, textvariable=self.target_var, width=4).pack(side="left", padx=(0, 8))
        for text, colour, command, enabled in (
                ("Finish mapping", "#2980b9", None, False),
                ("Go to target", "#e67e22", None, False),
                ("Stop", "#e74c3c", lambda: dashboard.send("stop"), True),
                ("Reset fault", "#8e44ad", lambda: dashboard.send("reset_fault"), True),
                ("Reset map", "#7f8c8d", lambda: dashboard.send("reset_map"), True),
                ("Obstacles: off", "#16a085", None, False)):
            tk.Button(row, text=text, bg=colour, fg="white", font=("Arial", 10, "bold"),
                      command=command, state="normal" if enabled else "disabled").pack(side="left", padx=3)
        self.fig = Figure(figsize=(8, 8), dpi=100, facecolor=BG)
        self.ax = self.fig.add_subplot(111)
        self.fig.subplots_adjust(left=.10, right=.79, top=.94, bottom=.18)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.mpl_connect("button_press_event", self.on_click)
        # Setup is kept in this tab so the original top-level layout is preserved.
        geometry = tk.Frame(self, bg=PANEL)
        geometry.pack(side="bottom", fill="x", padx=6, pady=2)
        for label, variable in (("Lens height mm", dashboard.height),
                                ("Forward offset mm", dashboard.forward),
                                ("Downward tilt deg", dashboard.down)):
            tk.Label(geometry, text=label, fg="white", bg=PANEL).pack(side="left", padx=3)
            tk.Entry(geometry, textvariable=variable, width=6).pack(side="left")
        tk.Button(geometry, text="Set camera geometry", bg="#2980b9", fg="white",
                  command=lambda: dashboard.send("geometry")).pack(side="left", padx=6)
        controls = tk.Frame(self, bg=PANEL)
        controls.pack(side="bottom", fill="x", padx=6, pady=(2, 5))
        dashboard.start_button = tk.Button(controls, text="Start DFS", bg="#27ae60", fg="white",
                                           command=lambda: dashboard.send("start"), state="disabled")
        dashboard.start_button.pack(side="left", padx=3)
        for text, action, colour in (("Resume", "resume", "#2980b9"),
                                     ("Refresh map", "snapshot", "#7f8c8d")):
            tk.Button(controls, text=text, bg=colour, fg="white",
                      command=lambda a=action: dashboard.send(a)).pack(side="left", padx=3)
        tk.Label(controls, text="Path planner pending integration", fg="#bdc3c7", bg=PANEL).pack(side="left", padx=8)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=6, pady=6)

    def on_click(self, event):
        snap = self.dashboard.model.snapshot
        if not snap or event.inaxes is not self.ax or event.xdata is None or event.ydata is None:
            return
        pitch = snap.get("pitch_mm", 600) / 10
        nearest = min(snap["markers"], default=None,
                      key=lambda m: (m["x"]*pitch-event.xdata)**2 + (m["y"]*pitch-event.ydata)**2)
        if nearest and ((nearest["x"]*pitch-event.xdata)**2 +
                        (nearest["y"]*pitch-event.ydata)**2) <= (pitch*.5)**2:
            self.selected = nearest["id"]
            self.target_var.set(str(self.selected))
            self.dashboard.draw()


def draw_onboard_map(ax, snap, *, live=False, selected=None):
    """Render a received snapshot in the original dark map style; infer no topology."""
    from matplotlib.lines import Line2D

    ax.clear()
    for legend in list(ax.figure.legends):
        legend.remove()
    ax.set_facecolor("#1a1a1a")
    ax.set_aspect("equal", adjustable="box")
    ax.tick_params(colors="white", labelsize=8)
    for spine in ax.spines.values():
        spine.set_color("#555555")
    pitch = (snap or {}).get("pitch_mm", 600) / 10
    # Live view keeps the original grid-cell units; mapping tab uses centimetres.
    if live:
        pitch = 1
    markers = (snap or {}).get("markers", [])
    positions = {m["id"]: (m["x"]*pitch, m["y"]*pitch) for m in markers}
    margin = .8*pitch
    if positions:
        xs, ys = zip(*positions.values())
        ax.set_xlim(min(xs)-margin, max(xs)+margin)
        ax.set_ylim(min(ys)-margin, max(ys)+margin)
    else:
        ax.set_xlim((-.5 if live else -1)*pitch, (5.5 if live else 1)*pitch)
        ax.set_ylim((-.5 if live else -1)*pitch, (5.5 if live else 1)*pitch)
        ax.text(.5, .5, "No map yet.\nSet the starting marker to begin.", transform=ax.transAxes,
                color="#ecf0f1", ha="center", va="center", fontsize=11)
    ax.xaxis.set_major_locator(MultipleLocator(pitch*max(1, math.ceil((ax.get_xlim()[1]-ax.get_xlim()[0])/pitch/9))))
    ax.yaxis.set_major_locator(MultipleLocator(pitch*max(1, math.ceil((ax.get_ylim()[1]-ax.get_ylim()[0])/pitch/9))))
    ax.grid(True, color="#3498db", alpha=.6 if live else .18, linewidth=1 if live else .6)
    ax.set_xlabel("Column (grid cells)" if live else "x east (cm)   -   start marker = (0, 0)", color="white", fontsize=9)
    ax.set_ylabel("Row (grid cells)" if live else "y north (cm)", color="white", fontsize=9)
    if live:
        ax.set_title("Onboard map" if markers else "Set start node ID to begin grid mapping",
                     color="white", pad=15, fontsize=12)
    # Only links explicitly provided by the rover are displayed as driven links.
    for a, b, blocked in (snap or {}).get("edges", []):
        if a in positions and b in positions:
            ax.plot([positions[a][0], positions[b][0]], [positions[a][1], positions[b][1]],
                    color="#e74c3c" if blocked else "#3498db", lw=2 if blocked else 3,
                    ls="--" if blocked else "-", zorder=2)
    radius = .1833*pitch
    for marker in markers:
        x, y = positions[marker["id"]]
        for direction, state in enumerate(marker["exits"]):
            if state not in (1, 3):
                continue
            dx, dy = ((0, 1), (1, 0), (0, -1), (-1, 0))[direction]
            colour = "#f39c12" if state == 1 else "#e74c3c"
            ax.plot([x+dx*radius, x+dx*.46*pitch], [y+dy*radius, y+dy*.46*pitch],
                    color=colour, lw=2, ls=":" if state == 1 else "--", zorder=2)
            ax.text(x+dx*.55*pitch, y+dy*.55*pitch, "?" if state == 1 else "x",
                    color=colour, ha="center", va="center", fontsize=9)
        current = marker["id"] == snap.get("current")
        colour = "#f1c40f" if marker.get("junction", False) else "#95a5a6"
        ax.add_patch(Circle((x, y), radius, facecolor=colour,
                            edgecolor="#27ae60" if current else "#2c3e50",
                            linewidth=3.5 if current else 1.5, zorder=5))
        ax.text(x, y, str(marker["id"]), color="black", ha="center", va="center",
                fontsize=10, fontweight="bold", zorder=7)
        if marker["id"] == snap.get("start"):
            ax.text(x-radius, y+radius, "S", color="white", ha="right", va="bottom", zorder=7)
        for match, scale, colour in ((snap.get("target"), 1.5, "#e67e22"), (selected, 1.3, "white")):
            if marker["id"] == match:
                ax.add_patch(Circle((x, y), radius*scale, fill=False, edgecolor=colour, lw=2, zorder=4))
    pose = (snap or {}).get("pose", {})
    if pose.get("valid"):
        scale = (snap.get("pitch_mm", 600) if live else 10)
        ax.plot(pose["x_mm"]/scale, pose["y_mm"]/scale, "x", color="white", markersize=9, zorder=8)
    if not live:
        handles = [Line2D([0], [0], color=colour, lw=2, ls=style, label=label)
                   for colour, style, label in (("#3498db", "-", "driven link"),
                                                ("#f39c12", ":", "? unexplored exit"),
                                                ("#e74c3c", "--", "x blocked"))]
        ax.figure.legend(handles=handles, loc="center right", bbox_to_anchor=(.995, .5),
                         fontsize=8, frameon=False, labelcolor="#ecf0f1", handlelength=2.4)

