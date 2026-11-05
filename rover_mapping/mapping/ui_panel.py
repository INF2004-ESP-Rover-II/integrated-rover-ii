"""Tk panel for the dashboard's "Mapping & path" tab (Subsystem 4).

Everything runs on the Tk thread. The dashboard calls `sync()` once per navigation tick (100 ms):
  * Phase 1: reads Buddy 1's DFS graph (dfs_adapter) so the map fills in as the rover explores.
  * Phase 2: "Go to target" plans an A* route and drives it through her navigator (route_exec).
Drawing lives in ui_draw.py (matplotlib only, tested headless); this file is only Tk glue."""
import tkinter as tk
from typing import Callable, Optional

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from .bridge import MappingBridge
from .dfs_adapter import DfsAdapter
from .pose import PoseFeed
from .route_exec import RouteExplorer, RouteRunner
from .ui_draw import draw_map, nearest_marker

BG, FG = "#34495e", "white"


class MappingPanel(tk.Frame):
    def __init__(self, parent, navigator, get_threshold: Optional[Callable[[], int]] = None,
                 log: Optional[Callable[[str], None]] = None, publish=None):
        super().__init__(parent, bg=BG)
        self.nav = navigator
        self.get_threshold = get_threshold or (lambda: 0)
        self._log = log or print
        self.bridge = MappingBridge(notify=lambda m: self._log(f"[Mapping] {m}"), publish=publish)
        self.adapter = DfsAdapter(self.bridge)
        self.runner = RouteRunner(self.bridge)
        self.pose_feed = PoseFeed(navigator, self.bridge)
        self.pose_feed.attach()                # feeds every ToF sample into the occupancy grid
        self.show_occ = True
        self._last_occ_draw = 0.0
        self.selected: Optional[int] = None
        self._drawn = -1
        self._occ_drawn = -1
        self._build()

    # ---- layout ------------------------------------------------------------------
    def _build(self):
        self.status = tk.StringVar(value="IDLE  (enable LAPTOP DFS to start mapping)")
        tk.Label(self, textvariable=self.status, fg="#f1c40f", bg=BG,
                 font=("Arial", 11, "bold")).pack(anchor="w", padx=8, pady=(6, 2))
        row = tk.Frame(self, bg=BG)
        row.pack(fill="x", padx=6, pady=2)
        self.target_var = tk.StringVar()
        tk.Label(row, text="Target marker (or click one)", fg=FG, bg=BG).pack(side="left", padx=(2, 4))
        tk.Entry(row, textvariable=self.target_var, width=4).pack(side="left", padx=(0, 8))
        self._btn(row, "Go to target", "#e67e22", self._go)
        self._btn(row, "Stop", "#e74c3c", lambda: self.nav.stop("Mapping panel stop"))
        self._btn(row, "Reset fault", "#8e44ad", lambda: self.bridge.reset_fault())
        self._btn(row, "Reset map", "#7f8c8d", lambda: self.bridge.reset_map())
        self.occ_btn = self._btn(row, "Obstacles: on", "#16a085", self._toggle_occ)
        self.fig = Figure(figsize=(8, 8), dpi=100, facecolor="#2c3e50")
        self.ax = self.fig.add_subplot(111)
        self.fig.subplots_adjust(left=0.08, right=0.80, top=0.98, bottom=0.10)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=6, pady=6)
        self.canvas.mpl_connect("button_press_event", self._on_click)
        self.canvas.mpl_connect("resize_event", lambda e: setattr(self, "_drawn", -1))   # redraw to rescale
        self._redraw()

    @staticmethod
    def _btn(parent, text, colour, cmd):
        # Label-based: macOS Tk ignores bg/fg on tk.Button (white text on white = blank)
        lbl = tk.Label(parent, text=text, bg=colour, fg="white", padx=8, pady=3,
                       font=("Arial", 10, "bold"), cursor="hand2")
        lbl.bind("<Button-1>", lambda e: cmd())
        lbl.pack(side="left", padx=3)
        return lbl

    def _toggle_occ(self):
        self.show_occ = not self.show_occ
        self.occ_btn.config(text="Obstacles: on" if self.show_occ else "Obstacles: off")
        self._drawn = -1

    # ---- actions -----------------------------------------------------------------
    def _go(self):
        try:
            target = int(self.target_var.get())
            self.runner.begin(self.nav, target, int(self.get_threshold() or 0))
            self._log(f"[Mapping] Driving to marker {target}")
        except ValueError as exc:
            self._log(f"[Mapping] Cannot start route: {exc}")

    def _on_click(self, event):
        if event.inaxes is None or event.xdata is None:
            return
        found = nearest_marker(self.bridge.snapshot(), event.xdata, event.ydata)
        if found is not None:
            self.selected = found
            self.target_var.set(str(found))
            self._drawn = -1

    # ---- called by the dashboard every 100 ms ---------------------------------------
    def sync(self):
        if not isinstance(self.nav.explorer, RouteExplorer):
            self.adapter.sync(self.nav.explorer)
        self.runner.sync(self.nav)
        now = self.nav.clock()
        occ_changed = self.show_occ and self.bridge.grid.version != self._occ_drawn and now - self._last_occ_draw > 1.0
        if self.bridge.version != self._drawn or occ_changed:
            self._last_occ_draw = now
            self._redraw()

    def _redraw(self):
        self._drawn = self.bridge.version
        snap = self.bridge.snapshot()
        last = f"   last route: {snap['last_result']}" if snap.get("last_result") else ""
        self.status.set(f"{snap['state']}   markers {snap['count']}/{snap['n'] or '-'}{last}")
        self._occ_drawn = self.bridge.grid.version
        with self.bridge.lock:
            occ = self.bridge.grid.render() if self.show_occ else None
        draw_map(self.ax, snap, self.selected, occ, title=False)
        self.canvas.draw_idle()
