"""Draws a map snapshot with matplotlib only (no Tk), so it can be tested headless.

TO SCALE: all coordinates are centimetres (grid position x MARKER_PITCH_CM), the axes keep an equal
aspect ratio, and the axis ticks sit on the 60 cm marker lattice. Marker discs have a fixed
real-world radius (MARKER_DRAW_RADIUS_CM) so the gap between two markers is obvious, and the label
font follows the disc size in pixels, so it stays readable when the window is resized."""
from typing import Optional

from matplotlib.lines import Line2D
from matplotlib.patches import Circle

from .config import DIR_VECTORS, MARKER_PITCH_CM as PITCH

BG = "#1a1a1a"
TEXT = "#ecf0f1"
EDGE = "#3498db"
BLOCKED = "#e74c3c"
UNEXPLORED = "#f39c12"
ROUTE = "#2ecc71"
JUNCTION = "#f1c40f"
PLAIN = "#95a5a6"
CURRENT = "#27ae60"
TARGET = "#e67e22"
LIKELY = "#5dade2"
POSSIBLE = "#7f8c8d"

UNEXPLORED_STATUS = 1
MARKER_DRAW_RADIUS_CM = 11.0       # disc radius on the map (a 22 cm disc on a 60 cm pitch)


def _px_per_cm(ax) -> float:
    x0, x1 = ax.get_xlim()
    return ax.get_window_extent().width / max(x1 - x0, 1e-9)


def _pts(ax, cm: float) -> float:
    """centimetres -> points at the current axes size."""
    return cm * _px_per_cm(ax) * 72.0 / ax.figure.dpi


def _cm(m) -> tuple:
    return m["x"] * PITCH, m["y"] * PITCH


def draw_map(ax, snap: dict, selected: Optional[int] = None, occ=None, title: bool = True) -> None:
    """title=False when the caller already shows the status line. occ = (rgba array, [x0, x1, y0, y1] in cm) from OccupancyGrid.render(), or None."""
    ax.clear()
    for old in list(ax.figure.legends):          # the legend lives on the figure (to the right of the map)
        old.remove()
    ax.set_facecolor(BG)
    ax.set_aspect("equal", adjustable="box")
    ax.tick_params(colors=TEXT, labelsize=8)
    for spine in ax.spines.values():
        spine.set_color("#555")

    markers = {m["id"]: m for m in snap["markers"]}
    n = snap["n"]
    text = f"{snap['state']}"
    if n:
        text += f"  |  markers {snap['count']}/{n}"
    if snap.get("last_result"):
        text += f"  |  last route: {snap['last_result']}"
    if title:
        ax.set_title(text, color=TEXT, fontsize=11, pad=8)

    if not markers:
        ax.set_xlim(-PITCH, PITCH); ax.set_ylim(-PITCH, PITCH)
        ax.text(0, 0, "No map yet.\nEnable LAPTOP DFS and start a mission.",
                color=TEXT, ha="center", va="center", fontsize=11)
        ax.set_xticks([]); ax.set_yticks([])
        return

    pos = {i: _cm(m) for i, m in markers.items()}
    xs = [p[0] for p in pos.values()]
    ys = [p[1] for p in pos.values()]
    margin = 0.8 * PITCH
    ax.set_xlim(min(xs) - margin, max(xs) + margin)
    ax.set_ylim(min(ys) - margin, max(ys) + margin)
    if occ is not None:                       # keep the grid inside the view
        ax.set_xlim(min(ax.get_xlim()[0], occ[1][0] - 5), max(ax.get_xlim()[1], occ[1][1] + 5))
        ax.set_ylim(min(ax.get_ylim()[0], occ[1][2] - 5), max(ax.get_ylim()[1], occ[1][3] + 5))
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    lat = lambda lo, hi: [PITCH * k for k in range(int(lo // PITCH), int(hi // PITCH) + 1)]
    ax.set_xticks(lat(x0, x1)); ax.set_yticks(lat(y0, y1))
    ax.grid(True, color=EDGE, alpha=0.18, linewidth=0.6)
    ax.set_xlabel("x east (cm)   -   start marker = (0, 0)", color=TEXT, fontsize=9)
    ax.set_ylabel("y north (cm)", color=TEXT, fontsize=9)

    if occ is not None:
        ax.imshow(occ[0], origin="lower", extent=occ[1], interpolation="nearest", zorder=1, aspect="equal")
        ax.set_xlim(x0, x1); ax.set_ylim(y0, y1)

    r = MARKER_DRAW_RADIUS_CM
    rp = _pts(ax, r)                                   # disc radius in points
    font = max(5.0, min(18.0, rp * 0.95))

    # probable links first (under everything): never driven, guessed from grid adjacency
    for a, b, strong in snap.get("probable", []):
        if a in pos and b in pos:
            ax.plot([pos[a][0], pos[b][0]], [pos[a][1], pos[b][1]], color=LIKELY if strong else POSSIBLE,
                    lw=1.8 if strong else 1.3, ls=(0, (4, 4)) if strong else (0, (1, 4)), zorder=2)

    for a, b, blocked in snap["edges"]:
        (xa, ya), (xb, yb) = pos[a], pos[b]
        if blocked:
            ax.plot([xa, xb], [ya, yb], color=BLOCKED, lw=2, ls="--", zorder=2)
            ax.text((xa + xb) / 2, (ya + yb) / 2, "x", color=BLOCKED, ha="center", va="center",
                    fontsize=font * 1.3, fontweight="bold", zorder=6)
        else:
            ax.plot([xa, xb], [ya, yb], color=EDGE, lw=3, zorder=2)

    for m in markers.values():                          # unexplored exits: amber stubs with a ?
        x, y = pos[m["id"]]
        for d, status in enumerate(m["exits"]):
            if status == UNEXPLORED_STATUS:
                dx, dy = DIR_VECTORS[d]
                ax.plot([x + dx * r, x + dx * (r + 0.28 * PITCH)], [y + dy * r, y + dy * (r + 0.28 * PITCH)],
                        color=UNEXPLORED, lw=2, ls=":", zorder=2)
                ax.text(x + dx * (r + 0.4 * PITCH), y + dy * (r + 0.4 * PITCH), "?", color=UNEXPLORED,
                        ha="center", va="center", fontsize=font * 0.9, zorder=6)

    route = [i for i in snap["route"] if i in pos]       # planned route, finished hops dimmed
    hop = snap.get("hop", 0)
    for k, (a, b) in enumerate(zip(route, route[1:])):
        done = k < hop
        ax.annotate("", xy=pos[b], xytext=pos[a], zorder=3,
                    arrowprops=dict(arrowstyle="-|>", color=ROUTE, lw=4 if not done else 2,
                                    alpha=0.4 if done else 0.95, shrinkA=rp + 2, shrinkB=rp + 2))

    for m in markers.values():                           # marker discs, true size on the map
        x, y = pos[m["id"]]
        cur = m["id"] == snap["current"]
        ax.add_patch(Circle((x, y), r, facecolor=JUNCTION if m["junction"] else PLAIN,
                            edgecolor=CURRENT if cur else "#2c3e50", linewidth=3.5 if cur else 1.5, zorder=5))
        ax.text(x, y, str(m["id"]), color="black", ha="center", va="center",
                fontsize=font, fontweight="bold", zorder=7)
        if m["id"] == snap["start"]:
            ax.text(x - r * 1.05, y + r * 1.05, "S", color=TEXT, fontsize=font * 0.9,
                    fontweight="bold", ha="right", va="bottom", zorder=7)
    if snap["target"] in pos:
        ax.add_patch(Circle(pos[snap["target"]], r * 1.5, fill=False, edgecolor=TARGET, linewidth=2.5, zorder=4))
    if selected in pos:
        ax.add_patch(Circle(pos[selected], r * 1.3, fill=False, edgecolor="white", linewidth=1.5,
                            linestyle="--", zorder=4))

    handles = [Line2D([0], [0], color=EDGE, lw=3, label="driven link"),
               Line2D([0], [0], color=LIKELY, lw=1.8, ls=(0, (4, 4)), label="probable (ToF saw it open)"),
               Line2D([0], [0], color=POSSIBLE, lw=1.3, ls=(0, (1, 4)), label="probable (never checked)"),
               Line2D([0], [0], color=UNEXPLORED, lw=2, ls=":", label="? unexplored exit"),
               Line2D([0], [0], color=BLOCKED, lw=2, ls="--", label="x blocked"),
               Line2D([0], [0], color=ROUTE, lw=4, label="planned route")]
    if occ is not None:
        handles += [Line2D([0], [0], marker="s", color="none", markerfacecolor="#f2594a", markersize=7,
                           label="occupied cell (5 cm)")]
    leg = ax.figure.legend(handles=handles, loc="center right", bbox_to_anchor=(0.995, 0.5), ncol=1,
                           fontsize=8, frameon=False, labelcolor=TEXT, handlelength=2.4)
    for t in leg.get_texts():
        t.set_color(TEXT)


def nearest_marker(snap: dict, x_cm: float, y_cm: float,
                   radius_cm: float = 0.5 * PITCH) -> Optional[int]:
    """Marker whose centre is within radius_cm of the clicked point (cm coordinates)."""
    best, best_d = None, radius_cm
    for m in snap["markers"]:
        mx, my = _cm(m)
        d = ((mx - x_cm) ** 2 + (my - y_cm) ** 2) ** 0.5
        if d <= best_d:
            best, best_d = m["id"], d
    return best
