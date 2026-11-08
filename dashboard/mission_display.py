"""Decode rover mission reports for display; no navigation or command output."""
from dataclasses import dataclass


MISSION_STATES = frozenset({"IDLE", "READY", "EXPLORING", "PAUSED", "COMPLETE", "FAULT"})


@dataclass(frozen=True)
class MissionReport:
    state: str
    reason: str
    test_mode: bool
    motion_enabled: bool
    revision: int

    @property
    def mode(self):
        if self.test_mode:
            return "Test (stationary)" if not self.motion_enabled else "Test"
        return "Inactive" if self.state in {"IDLE", "FAULT"} and not self.motion_enabled else "Onboard"

    @property
    def display_values(self):
        # Revision/periodic retransmission alone should not create a new log row.
        return self.state, self.reason, self.test_mode, self.motion_enabled


def parse_mission_report(value):
    """Accept the existing MISSION_STATE text or telemetry's mission object."""
    if isinstance(value, str):
        parts = value.split()
        if not parts or not parts[0].startswith("MISSION_STATE:"):
            raise ValueError("Expected MISSION_STATE report")
        fields = {}
        for token in parts[1:]:
            key, separator, item = token.partition("=")
            if not separator or key in fields:
                raise ValueError("Invalid mission field")
            fields[key] = item
        if not {"test", "motion", "reason", "revision"} <= fields.keys():
            raise ValueError("Incomplete mission report")
        if fields["test"] not in {"0", "1"} or fields["motion"] not in {"disabled", "enabled"}:
            raise ValueError("Invalid mission flags")
        if not fields["revision"].isascii() or not fields["revision"].isdecimal():
            raise ValueError("Invalid mission revision")
        value = {
            "state": parts[0][len("MISSION_STATE:"):], "reason": fields["reason"],
            "test_mode": fields["test"] == "1", "motion_enabled": fields["motion"] == "enabled",
            "revision": int(fields["revision"]),
        }
    if not isinstance(value, dict):
        raise ValueError("Expected mission object")
    state, reason = value.get("state"), value.get("reason")
    test, motion, revision = value.get("test_mode"), value.get("motion_enabled"), value.get("revision")
    if not isinstance(state, str) or state not in MISSION_STATES:
        raise ValueError("Invalid mission state")
    if (not isinstance(reason, str) or not 1 <= len(reason) <= 64 or
            any(char not in "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_" for char in reason)):
        raise ValueError("Invalid mission reason")
    if type(test) is not bool or type(motion) is not bool:
        raise ValueError("Invalid mission flags")
    if type(revision) is not int or not 0 <= revision <= 0xffffffff:
        raise ValueError("Invalid mission revision")
    return MissionReport(state, reason, test, motion, revision)
