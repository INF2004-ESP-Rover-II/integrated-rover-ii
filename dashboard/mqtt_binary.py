"""Binary MQTT payloads for the Robo Pico link (see docs/MQTT_BINARY.md).

decode_telemetry() returns the same dictionary the old JSON telemetry produced, so
nothing downstream (ToF panel, watchdog, laptop DFS, mapping) has to change.
Accepts both formats during the switch-over: JSON starts with '{', binary does not.
"""

import json
import math

TELEMETRY_VERSION = 1
TELEMETRY_BYTES = 23

# (name, width in bits), packed least-significant bit first, in this order.
TELEMETRY_FIELDS = (
    ("version", 4),
    ("timestamp_ms", 32),
    ("heading_d10", 12),
    ("pitch_d10", 12),
    ("roll_d10", 12),
    ("imu_fault", 1),
    ("us_front_cm", 9),
    ("us_back_cm", 9),
    ("tof_bits", 30),
    ("obstacle_flags", 10),
    ("age_ms", 10),
    ("heading_epoch", 8),          # change counter for "Set heading to 0", mod 256
    ("heading_offset_mdeg", 20),   # signed thousandths of a degree (JSON sent %.3f)
    ("forward_threshold_mm", 10),  # firmware FRONT_STOP_DISTANCE_MM, 1..1000
)
SIGNED_FIELDS = {"pitch_d10", "roll_d10", "heading_offset_mdeg"}
OFFSET_LIMIT_MDEG = (1 << 19) - 1   # +/-524.287 degrees
assert sum(width for _, width in TELEMETRY_FIELDS) <= TELEMETRY_BYTES * 8

COMMAND_CODES = {
    "STOP": 1, "ESTOP": 2, "RESET_SAFETY": 3,
    "FORWARD": 4, "BACKWARD": 5, "LEFT": 6, "RIGHT": 7,
    "ROTATE_LEFT": 8, "ROTATE_RIGHT": 9,
    "FRONT_LEFT": 10, "FRONT_RIGHT": 11, "BACK_LEFT": 12, "BACK_RIGHT": 13,
    "CLAW_OPEN": 14, "CLAW_CLOSE": 15, "ARM_UP": 16, "ARM_DOWN": 17,
    "ZERO_HEADING": 18, "TURN_TO": 19, "AUTO_STOP": 20, "AUTO_START": 21,
}
COMMAND_NAMES = {code: name for name, code in COMMAND_CODES.items()}
CODE_MASK = 0x1F       # 5 bits
TURN_TO_SHIFT = 5      # 9-bit heading in bits 5..13


def _clamp(value, low, high):
    return max(low, min(high, value))


def _round_d10(value):
    """Degrees -> integer tenths of a degree (round half away from zero, like the C side)."""
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("IMU angle must be finite")
    tenths = value * 10.0
    return int(tenths + 0.5) if tenths >= 0 else -int(-tenths + 0.5)


def _round_milli(value):
    """Degrees -> integer thousandths (round half away from zero, like the C side)."""
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("Heading offset must be finite")
    milli = value * 1000.0
    return int(milli + 0.5) if milli >= 0 else -int(-milli + 0.5)


# ---------------------------------------------------------------------------
# Telemetry
# ---------------------------------------------------------------------------
def pack_fields(fields):
    """Pack a dict of already-ranged integer fields into TELEMETRY_BYTES bytes."""
    word, offset = 0, 0
    for name, width in TELEMETRY_FIELDS:
        value = int(fields[name])
        if name in SIGNED_FIELDS:
            low, high = -(1 << (width - 1)), (1 << (width - 1)) - 1
            if not low <= value <= high:
                raise ValueError(f"{name}={value} does not fit in {width} signed bits")
            value &= (1 << width) - 1
        elif not 0 <= value < (1 << width):
            raise ValueError(f"{name}={value} does not fit in {width} bits")
        word |= value << offset
        offset += width
    return word.to_bytes(TELEMETRY_BYTES, "little")


def unpack_fields(payload):
    """Bytes -> dict of integer fields. Raises ValueError for a wrong size or version."""
    if len(payload) != TELEMETRY_BYTES:
        raise ValueError(f"Binary telemetry must be {TELEMETRY_BYTES} bytes, got {len(payload)}")
    word, offset, fields = int.from_bytes(payload, "little"), 0, {}
    for name, width in TELEMETRY_FIELDS:
        value = (word >> offset) & ((1 << width) - 1)
        if name in SIGNED_FIELDS and value >> (width - 1):
            value -= 1 << width
        fields[name] = value
        offset += width
    if fields["version"] != TELEMETRY_VERSION:
        raise ValueError(f"Unsupported telemetry version {fields['version']}")
    return fields


def encode_telemetry(data):
    """Old-style telemetry dict (as the JSON had it) -> binary payload.

    Used by fake_rover.py and the tests; the Robo Pico does the same in C.
    """
    imu, ultrasonic, obstacle = data["imu"], data.get("ultrasonic", {}), data["obstacle"]
    distances = obstacle["distance_mm"]
    tof_bits = 0
    for shift, side in ((0, "left"), (10, "front"), (20, "right")):
        tof_bits |= _clamp(int(distances.get(side, 0)), 0, 1023) << shift
    flags = ((int(obstacle.get("blocked_mask", 0)) & 7)
             | (int(obstacle.get("urgent_mask", 0)) & 7) << 3
             | (int(obstacle.get("fault_mask", 0)) & 7) << 6
             | (1 << 9 if obstacle.get("stop_required") else 0))
    return pack_fields({
        "version": TELEMETRY_VERSION,
        "timestamp_ms": int(data["timestamp"]) & 0xFFFFFFFF,
        "heading_d10": _round_d10(float(imu.get("heading", 0.0)) % 360.0) % 3600,
        "pitch_d10": _clamp(_round_d10(imu.get("pitch", 0.0)), -1800, 1800),
        "roll_d10": _clamp(_round_d10(imu.get("roll", 0.0)), -1800, 1800),
        "imu_fault": 0 if imu.get("status", 0) == 0 else 1,
        "us_front_cm": _clamp(int(ultrasonic.get("front", 0)), 0, 511),
        "us_back_cm": _clamp(int(ultrasonic.get("back", 0)), 0, 511),
        "tof_bits": tof_bits,
        "obstacle_flags": flags,
        "age_ms": _clamp(int(obstacle.get("age_ms", 0)), 0, 1023),
        "heading_epoch": int(data.get("heading_epoch", 0)) & 0xFF,
        "heading_offset_mdeg": _clamp(_round_milli(data.get("heading_offset_deg", 0.0)),
                                      -OFFSET_LIMIT_MDEG, OFFSET_LIMIT_MDEG),
        "forward_threshold_mm": _clamp(int(obstacle.get("forward_threshold_mm", 150)), 1, 1000),
    })


def fields_to_telemetry(fields):
    """Integer fields -> the dictionary shape the JSON telemetry used to have."""
    tof, flags = fields["tof_bits"], fields["obstacle_flags"]
    return {
        "timestamp": fields["timestamp_ms"],
        "heading_epoch": fields["heading_epoch"],
        "heading_offset_deg": fields["heading_offset_mdeg"] / 1000.0,
        "imu": {
            "heading": fields["heading_d10"] / 10.0,
            "pitch": fields["pitch_d10"] / 10.0,
            "roll": fields["roll_d10"] / 10.0,
            "status": fields["imu_fault"],
        },
        "ultrasonic": {"front": fields["us_front_cm"], "back": fields["us_back_cm"]},
        "obstacle": {
            "distance_mm": {"left": tof & 0x3FF, "front": (tof >> 10) & 0x3FF,
                            "right": (tof >> 20) & 0x3FF},
            "blocked_mask": flags & 7,
            "urgent_mask": (flags >> 3) & 7,
            "fault_mask": (flags >> 6) & 7,
            "stop_required": bool(flags & (1 << 9)),
            "age_ms": fields["age_ms"],
            "forward_threshold_mm": fields["forward_threshold_mm"],
        },
    }


def decode_telemetry(payload):
    """robot/telemetry payload (bytes) -> telemetry dict. Accepts JSON or binary.

    Raises ValueError (including json.JSONDecodeError) for anything malformed.
    """
    payload = bytes(payload)
    if payload[:1] == b"{":
        data = json.loads(payload.decode("utf-8", errors="replace"))
        if not isinstance(data, dict):
            raise ValueError("Telemetry JSON must be an object")
        return data
    return fields_to_telemetry(unpack_fields(payload))


def telemetry_format(payload):
    """Short label for logging, e.g. 'binary v1, 23 bytes' or 'JSON, 612 bytes'."""
    payload = bytes(payload)
    kind = "JSON" if payload[:1] == b"{" else f"binary v{payload[0] & 0x0F}" if payload else "empty"
    return f"{kind}, {len(payload)} bytes"


# ---------------------------------------------------------------------------
# Commands and heartbeat
# ---------------------------------------------------------------------------
def encode_command(command):
    """'FORWARD' -> b'\\x04';  'TURN_TO:90' -> 2 bytes. Unknown commands raise ValueError."""
    name, _, argument = command.partition(":")
    code = COMMAND_CODES.get(name)
    if code is None:
        raise ValueError(f"No binary code for command {command!r}")
    if name == "TURN_TO":
        heading = int(argument)
        if not 0 <= heading <= 359:
            raise ValueError("TURN_TO heading must be 0..359")
        return (code | heading << TURN_TO_SHIFT).to_bytes(2, "little")
    if argument:
        raise ValueError(f"{name} takes no argument")
    return bytes([code])


def decode_command(payload):
    """Binary command -> the old text form ('FORWARD', 'TURN_TO:90'). Raises ValueError."""
    payload = bytes(payload)
    if len(payload) not in (1, 2):
        raise ValueError("Binary commands are 1 or 2 bytes")
    word = int.from_bytes(payload, "little")
    name = COMMAND_NAMES.get(word & CODE_MASK)
    if name is None:
        raise ValueError(f"Unknown command code {word & CODE_MASK}")
    if name == "TURN_TO":
        heading = (word >> TURN_TO_SHIFT) & 0x1FF
        if len(payload) != 2 or heading > 359 or word >> 14:
            raise ValueError("Bad TURN_TO payload")
        return f"TURN_TO:{heading}"
    if len(payload) != 1 or word >> 5:
        raise ValueError(f"{name} must be a single byte with bits 5..7 clear")
    return name


def is_binary_command(payload):
    """Text commands are at least 4 bytes ('STOP'); binary ones are 1 or 2."""
    return len(payload) <= 2


def encode_heartbeat(sequence):
    return bytes([sequence & 0xFF])
