"""Offline checks of restored controls; no broker or rover is contacted."""
import copy
import json
import time
import tkinter as tk
import unittest
from types import SimpleNamespace

from onboard_dashboard import OnboardDashboard
from test_onboard_state import snapshot


class FakeClient:
    def __init__(self):
        self.messages = []

    def is_connected(self):
        return True

    def publish(self, topic, payload, **kwargs):
        self.messages.append((topic, payload))
        return SimpleNamespace(rc=0)

    def disconnect(self):
        pass

    def loop_stop(self):
        pass


class RestoredDashboardTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.app = OnboardDashboard(self.root, replay=[])
        self.client = FakeClient()
        self.app.client = self.client

    def tearDown(self):
        self.app.close()

    def ready(self, anchored=True):
        value = snapshot()
        value.update(start=7 if anchored else 0, geometry_ready=True, confirmed=True)
        self.app.model.update(value)
        self.app.replay_mode = False
        self.app.last_sensor = time.monotonic()
        return value

    def commands(self):
        return [payload for topic, payload in self.client.messages if topic == "robot/command"]

    def test_start_controls_send_onboard_commands_only(self):
        self.ready()
        self.app.tick()
        self.app.start_button.invoke()
        self.app.start_id.set("12")
        self.app.set_start_node()
        self.assertEqual(self.commands(), ["DFS_START:36:500", "SET_START:12"])
        self.assertEqual(self.app.map_tabs.tab(0, "text"), "Live view")
        self.assertEqual(self.app.map_tabs.tab(1, "text"), "Mapping & path")

    def test_replay_never_sends_even_after_replay_finishes(self):
        self.app.enqueue("robot/map", json.dumps(snapshot()))
        self.app.tick()
        self.app.trigger_estop()
        self.app.send("start_node")
        self.app.send_command("FORWARD")
        self.assertEqual(self.client.messages, [])
        self.assertEqual(str(self.app.start_button["state"]), "disabled")

    def test_manual_keys_respect_anchor_and_entry_focus(self):
        self.ready()
        event = SimpleNamespace(keysym="w", widget=self.root)
        self.app.on_key_press(event)
        self.assertEqual(self.commands(), [])
        self.app.pressed_keys.clear()
        self.ready(anchored=False)
        self.app.on_key_press(SimpleNamespace(keysym="w", widget=self.app.ip_entry))
        self.assertEqual(self.commands(), [])
        self.app.on_key_press(event)
        self.assertEqual(self.commands(), ["FORWARD"])
        self.app.on_focus_out(None)
        self.assertEqual(self.commands(), ["FORWARD", "STOP"])

    def test_missing_geometry_disables_both_start_controls(self):
        value = self.ready()
        value["geometry_ready"] = False
        self.app.model.update(value)
        self.app.tick()
        self.app.auto_mode.set(True)
        self.app.toggle_auto_mode()
        self.assertEqual(str(self.app.start_button["state"]), "disabled")
        self.assertFalse(self.app.auto_mode.get())
        self.assertEqual(self.commands(), [])

    def test_received_map_replaces_wait_message_and_does_not_infer_nodes(self):
        value = snapshot()
        original = copy.deepcopy(value)
        self.app.enqueue("robot/map", json.dumps(value))
        self.app.tick()
        self.assertEqual(self.app.model.snapshot, original)
        self.assertEqual(self.app.status.get(), "Receiving onboard map snapshots")
        self.assertEqual(self.app.dfs_current_node_text.get(), "7")
        self.assertEqual(self.client.messages, [])


if __name__ == "__main__":
    unittest.main()
