import unittest

from mapping import events as ev
from mapping.config import E, N, S, W
from mapping.controller import MappingController, State


def mapped(n=3):
    """Controller in MAP_COMPLETE with markers 0-1-2 in a row (n must be 3)."""
    c = MappingController()
    c.start(n, 0)
    c.on_rover_event(ev.MarkerReached(1, E, seq=1))
    c.on_rover_event(ev.MarkerReached(2, E, seq=2))
    return c


class TestPhase1(unittest.TestCase):
    def test_start_guards(self):
        c = MappingController()
        self.assertFalse(c.start(0, 0))
        self.assertTrue(c.start(3, 0))
        self.assertEqual(c.state, State.MAPPING)
        self.assertFalse(c.start(3, 0))            # already running

    def test_reaching_n_completes_and_notifies(self):
        c = mapped()
        self.assertEqual(c.state, State.MAP_COMPLETE)
        self.assertEqual(c.sent_log, [ev.CountReached(3)])

    def test_n_of_one_completes_immediately(self):
        c = MappingController(); c.start(1, 0)
        self.assertEqual(c.state, State.MAP_COMPLETE)

    def test_exhausted_when_fewer_than_n(self):
        c = MappingController(); c.start(5, 0)
        c.on_rover_event(ev.MarkerReached(1, E, seq=1))
        c.on_rover_event(ev.ExploreDoneLocal(seq=2))
        self.assertEqual(c.state, State.MAP_EXHAUSTED)
        self.assertEqual(c.sent_log, [ev.MapExhausted(2, 5)])

    def test_exhausted_only_left_by_operator_reset_to_idle(self):
        c = MappingController(); c.start(5, 0)
        c.on_rover_event(ev.ExploreDoneLocal(seq=1))
        self.assertFalse(c.reset_map())
        self.assertTrue(c.operator_reset())
        self.assertEqual(c.state, State.IDLE)
        self.assertEqual(c.graph.count(), 0)

    def test_repeated_and_stale_events_are_dropped(self):
        c = MappingController(); c.start(9, 0)
        self.assertTrue(c.on_rover_event(ev.MarkerReached(1, E, seq=10)))
        self.assertFalse(c.on_rover_event(ev.MarkerReached(1, E, seq=10)))   # repeat
        self.assertFalse(c.on_rover_event(ev.MarkerReached(2, E, seq=9)))    # older
        self.assertEqual(c.graph.count(), 2)

    def test_seq_wrap_accepted(self):
        c = MappingController(); c.start(9, 0)
        c.on_rover_event(ev.MarkerReached(1, E, seq=255))
        self.assertTrue(c.on_rover_event(ev.MarkerReached(2, E, seq=0)))

    def test_exit_events_update_map(self):
        c = MappingController(); c.start(9, 0)
        rep = (ev.ExitStatus.UNEXPLORED, ev.ExitStatus.BLOCKED, ev.ExitStatus.BLOCKED, ev.ExitStatus.BLOCKED)
        c.on_rover_event(ev.ExitReport(0, rep, seq=1))
        self.assertTrue(c.graph.has_unexplored())
        c.on_rover_event(ev.ExitBlocked(0, N, seq=2))
        self.assertFalse(c.graph.has_unexplored())

    def test_stop_pause_resume_guards(self):
        c = MappingController(); c.start(9, 0)
        c.operator_stop()
        self.assertEqual(c.state, State.PAUSED)
        self.assertFalse(c.operator_resume(self_check_ok=True))       # position not confirmed
        c.on_marker_seen(0)
        self.assertFalse(c.operator_resume(self_check_ok=False))      # self-check failed
        self.assertTrue(c.operator_resume(self_check_ok=True))
        self.assertEqual(c.state, State.MAPPING)

    def test_phase1_fault_pauses_and_keeps_map(self):
        c = MappingController(); c.start(9, 0)
        c.on_rover_event(ev.MarkerReached(1, E, seq=1))
        c.mission_fault()
        self.assertEqual(c.state, State.PAUSED)
        self.assertEqual(c.graph.count(), 2)
        self.assertFalse(c.on_rover_event(ev.MarkerReached(2, E, seq=2)))   # ignored while paused

    def test_reset_map_from_paused(self):
        c = MappingController(); c.start(9, 0)
        c.operator_stop()
        self.assertTrue(c.reset_map())
        self.assertEqual(c.state, State.IDLE)

    def test_exit_dashboard_only_from_idle(self):
        c = MappingController()
        self.assertTrue(c.exit_dashboard())
        self.assertEqual(c.state, State.ENDED)
        d = MappingController(); d.start(3, 0)
        self.assertFalse(d.exit_dashboard())


class TestPhase2(unittest.TestCase):
    def test_route_sent_one_segment_at_a_time(self):
        c = mapped(); c.drain()
        self.assertTrue(c.set_target(0))                       # rover is at marker 2
        segs = c.drain()
        self.assertEqual(len(segs), 1)                         # never the whole route at once
        self.assertEqual((segs[0].target_marker, segs[0].heading, segs[0].last), (1, W, False))
        self.assertEqual(c.state, State.WAITING_SEGMENT)
        c.on_rover_event(ev.SegmentDone(segs[0].seq))
        seg2 = c.drain()[0]
        self.assertEqual((seg2.target_marker, seg2.last), (0, True))
        c.on_rover_event(ev.SegmentDone(seg2.seq))
        self.assertEqual(c.state, State.MAP_COMPLETE)
        self.assertEqual(c.sent_log[-1], ev.RouteResult("REACHED", 0))
        self.assertIn(State.TARGET_REACHED, c.history)

    def test_target_is_current_marker(self):
        c = mapped()
        c.set_target(2)                                        # rover already at 2
        self.assertEqual(c.sent_log[-1], ev.RouteResult("REACHED", 2))
        self.assertEqual([m for m in c.sent_log if isinstance(m, ev.Segment)], [])

    def test_planning_guarded_by_confirmed_marker(self):
        c = mapped()
        c.position_confirmed = False
        self.assertFalse(c.set_target(0))                      # stale position: refuse to plan
        c.on_marker_seen(2)
        self.assertTrue(c.set_target(0))

    def test_unknown_target_rejected(self):
        c = mapped()
        self.assertFalse(c.set_target(42))

    def test_stale_and_repeated_segment_done_ignored(self):
        c = mapped(); c.drain(); c.set_target(0)
        seg = c.drain()[0]
        self.assertFalse(c.on_rover_event(ev.SegmentDone(seg.seq + 5)))      # wrong seq
        self.assertTrue(c.on_rover_event(ev.SegmentDone(seg.seq)))
        self.assertFalse(c.on_rover_event(ev.SegmentDone(seg.seq)))          # duplicate
        self.assertEqual(len(c.drain()), 1)                                  # only one next segment

    def test_blocked_exit_replans_and_no_route(self):
        c = mapped(); c.drain(); c.set_target(0)
        seg = c.drain()[0]
        c.on_rover_event(ev.ExitBlocked(2, W, ref_seq=seg.seq))
        self.assertIn(State.REPLANNING, c.history)
        self.assertEqual(c.sent_log[-1], ev.RouteResult("NO_ROUTE", 0))      # only route was blocked
        self.assertEqual(c.state, State.MAP_COMPLETE)
        self.assertEqual(c.graph.count(), 3)                                 # map kept

    def test_phase2_fault_goes_to_fault_wait_then_reset_to_map_complete(self):
        c = mapped(); c.drain(); c.set_target(0)
        c.mission_fault()
        self.assertEqual(c.state, State.FAULT_WAIT)
        self.assertFalse(c.on_rover_event(ev.SegmentDone(1)))               # no sending blind
        self.assertFalse(c.operator_reset(self_check_ok=False))
        self.assertTrue(c.operator_reset(self_check_ok=True))
        self.assertEqual(c.state, State.MAP_COMPLETE)
        self.assertEqual(c.graph.count(), 3)
        self.assertFalse(c.set_target(0))                                    # marker not re-confirmed yet
        c.on_marker_seen(2)
        self.assertTrue(c.set_target(0))

    def test_operator_stop_in_phase2_returns_to_map_complete(self):
        c = mapped(); c.drain(); c.set_target(0)
        c.operator_stop()
        self.assertEqual(c.state, State.MAP_COMPLETE)

    def test_reset_map_from_map_complete(self):
        c = mapped()
        self.assertTrue(c.reset_map())
        self.assertEqual(c.state, State.IDLE)


if __name__ == "__main__":
    unittest.main()
