"""Smoke-test the Tk glue (mapping/ui_panel.py) WITHOUT Tk: tkinter and the Tk matplotlib backend are
replaced by tiny stand-ins, so typos / wrong calls in the panel show up even on a machine with no
display. It does not test how the window looks (see the PNG renders for that)."""
import sys
import types
import unittest

from tests import _paths  # noqa: F401


class Widget:
    def __init__(self, *a, **k):
        self.a, self.k, self.bound = a, k, {}

    def pack(self, *a, **k): return self
    def grid(self, *a, **k): return self
    def bind(self, event, fn): self.bound[event] = fn
    def config(self, **k): self.k.update(k)
    def get_tk_widget(self): return self


class Var:
    def __init__(self, value=""): self.v = value
    def get(self): return self.v
    def set(self, v): self.v = v


class FakeCanvas(Widget):
    def __init__(self, fig, master=None):
        super().__init__()
        self.draws = 0
    def draw_idle(self): self.draws += 1
    def mpl_connect(self, name, fn): self.cb = fn


def install_stubs():
    tk = types.ModuleType("tkinter")
    tk.Frame = type("Frame", (Widget,), {"__init__": lambda self, parent=None, **k: Widget.__init__(self)})
    tk.Label, tk.Entry, tk.StringVar = Widget, Widget, Var
    sys.modules["tkinter"] = tk
    be = types.ModuleType("matplotlib.backends.backend_tkagg")
    be.FigureCanvasTkAgg = FakeCanvas
    sys.modules["matplotlib.backends.backend_tkagg"] = be


class PanelSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import matplotlib
        matplotlib.use("Agg")
        cls._saved = {k: sys.modules.get(k) for k in ("tkinter", "matplotlib.backends.backend_tkagg")}
        install_stubs()
        sys.modules.pop("mapping.ui_panel", None)
        from mapping import ui_panel
        cls.ui_panel = ui_panel

    @classmethod
    def tearDownClass(cls):
        for k, v in cls._saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
        sys.modules.pop("mapping.ui_panel", None)

    def test_panel_follows_dfs_and_drives_a_route(self):
        from tests.test_b1_integration import Rig, maze36, DState
        rig = Rig(maze36())
        logs = []
        panel = self.ui_panel.MappingPanel(rig.nav, rig.nav, get_threshold=lambda: 500, log=logs.append)
        rig.sync = panel.sync                               # the dashboard calls this every tick
        rig.start_dfs(36)
        self.assertTrue(rig.run(lambda: rig.nav.explorer.state == DState.COMPLETE))
        rig.run(lambda: not rig.nav.active)
        rig.step()
        self.assertEqual(panel.bridge.snapshot()["count"], 36)
        self.assertGreater(panel.canvas.draws, 5)           # it redrew as the map grew
        self.assertIn("MAP_COMPLETE", panel.status.get())
        for _ in range(5):
            rig.step()
        target = 36 if panel.bridge.snapshot()["current"] != 36 else 1
        panel.target_var.set(str(target))
        panel._go()                                         # what the "Go to target" label calls
        self.assertTrue(rig.run(lambda: not panel.runner.active, limit=60000))
        self.assertEqual(panel.bridge.snapshot()["current"], target)
        self.assertEqual(panel.bridge.snapshot()["last_result"], "REACHED")

    def test_go_with_bad_input_logs_instead_of_crashing(self):
        from tests.test_b1_integration import Rig, maze36
        rig = Rig(maze36())
        logs = []
        panel = self.ui_panel.MappingPanel(rig.nav, rig.nav, log=logs.append)
        panel.target_var.set("abc")
        panel._go()
        panel.target_var.set("5")
        panel._go()                                         # no map yet
        self.assertEqual(len(logs), 2)
        self.assertTrue(all("Cannot start route" in m for m in logs))

    def test_click_selects_nearest_marker(self):
        from tests.test_b1_integration import Rig, maze36, DState
        rig = Rig(maze36())
        panel = self.ui_panel.MappingPanel(rig.nav, rig.nav, log=lambda m: None)
        rig.sync = panel.sync
        rig.start_dfs(6)
        rig.run(lambda: rig.nav.explorer.state == DState.COMPLETE)
        rig.step()
        m = panel.bridge.snapshot()["markers"][-1]
        panel._on_click(types.SimpleNamespace(inaxes=object(), xdata=m["x"] * 60 + 3, ydata=m["y"] * 60))
        self.assertEqual(panel.target_var.get(), str(m["id"]))


if __name__ == "__main__":
    unittest.main()
