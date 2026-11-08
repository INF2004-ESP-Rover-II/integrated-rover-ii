"""Decode rover mission reports for display; no navigation or command output."""
from dataclasses import dataclass
import json


MISSION_STATES = frozenset({"IDLE", "READY", "EXPLORING", "PAUSED", "COMPLETE", "FAULT"})
LEG_PHASES = frozenset({'NONE', 'SCANNING', 'SELECTING', 'CONFIGURED', 'TURNING', 'APPROACHING', 'HOLDING', 'BRAKING', 'VERIFYING', 'COMPLETE', 'CANCELLED'})
LEG_FIELDS = ('leg_id', 'departure_node', 'next_target_node', 'arrival_reference_node',
              'target_cell', 'direction', 'phase', 'configured')

DFS_REFUSAL_MESSAGES = {
    'START_NODE_NOT_SET': 'Set the start node first, then start DFS.',
    'CURRENT_NODE_UNKNOWN': 'Current position is unknown. Set the physical start node before starting DFS.',
    'NO_LOCKED_TARGET': 'RoboPico has not locked a target yet. Wait for a target, then try Start DFS again.',
    'SENSOR_STALE': 'Sensor Pico data is stale. Wait for fresh sensor data before starting DFS.',
    'FRONT_SENSOR_UNAVAILABLE': 'Front ToF data is unavailable. Restore it before starting DFS.',
    'FRONT_SENSOR_FAULT': 'Front ToF sensor reports a fault or result timeout. Restore sensor health before starting DFS.',
    'CAMERA_STALE_OR_TARGET_LOST': 'The target camera observation is stale or missing. Reacquire the target.',
    'HEADING_UNAVAILABLE': 'A fresh healthy IMU heading is required before starting DFS.',
    'FRONT_BLOCKED': 'The front path is blocked. Clear it before starting DFS.',
    'TARGET_NOT_ALIGNED': 'The target must be within 5 degrees of camera centre before starting DFS.',
    'TARGET_TOO_CLOSE': 'Target is inside the approach stop range. Arrival has not been confirmed.',
    'SAFETY_LOCKED': 'Safety is locked. Clear the safety fault before starting DFS.',
    'MISSION_NOT_IDLE': 'Reset the mission and confirm the current node before starting DFS.',
    'MOTOR_BUSY': 'Stop the rover before starting DFS.',
    'LINK_LOST': 'RoboPico has no live dashboard link. Reconnect before starting DFS.',
    'MOTOR_REFUSED': 'Motion Control refused the drive. Check safety and heading status.'}
DFS_REFUSAL_MESSAGES.update({
    'DFS_EXITS_UNRESOLVED': 'DFS paused: exits remain unknown, inconsistent or unreachable. The map is not complete.',
    'DFS_TARGET_NOT_VISIBLE': 'DFS paused: no reliable marker is visible in the chosen exit direction.',
    'DFS_INPUT_UNAVAILABLE': 'DFS paused: fresh sensor, heading and link data are required.',
    'DFS_TURN_REFUSED': 'Motion Control refused the next DFS turn.',
    'DFS_TURN_FAILED': 'The DFS turn stopped or timed out before reaching its heading.',
    'DFS_TURN_LEASE_EXPIRED': 'The DFS turn lease expired; motion was stopped.',
    'DFS_ARRIVAL_MISMATCH': 'DFS arrival did not match the planned stack/parent node.',
    'DFS_STACK_MISMATCH': 'DFS stack/map consistency check failed.',
    'DFS_POSITION_UNKNOWN': 'DFS requires a confirmed current node before choosing an exit.',
    'INVALID_OPENING_THRESHOLD': 'Enter an opening threshold from 151 to 1000 mm.',
    'STOP_DFS_BEFORE_CONFIG_CHANGE': 'Stop DFS before changing the opening threshold.',
    'REFERENCE_BASELINE_UNAVAILABLE': 'Target is out of view and no synchronized reference baseline is available.',
    'REFERENCE_NOT_ALIGNED': 'The locked reference is outside the forward alignment limit.',
    'REFERENCE_RANGE_INCONSISTENT': 'Reference range changed inconsistently. Recheck the marker observations.',
    'REFERENCE_GEOMETRY_INCONSISTENT': 'Target/reference observations do not have consistent ordering or alignment.',
    'TRACKING_ID_CHANGED': 'The target/reference identity changed. The approach was cancelled.',
    'GRID_HEADING_UNAVAILABLE': 'The onboard grid direction needs a new confirmed start anchor.',
    'GRID_DIRECTION_NOT_ALIGNED': 'Face a grid direction within 15 degrees before starting.',
    'TARGET_MAP_CONFLICT': 'The planned target conflicts with the onboard map.',
    'MAP_FULL': 'The onboard map has reached its node capacity.',
    'TARGET_CELL_OVERFLOW': 'The planned target lies outside the supported coordinates.',
    'MAP_CHANGED_DURING_LEG': 'The onboard map changed during this travel leg.',
    'ARRIVAL_VERIFICATION_TIMEOUT': 'Stopped, but fresh post-brake observations did not confirm arrival.',
    'ARRIVAL_ESTIMATE_NOT_CONFIRMED': 'Stopped, but the reference estimate did not confirm arrival.',
    'TARGET_OVERSHOOT_UNCONFIRMED': 'The estimated target position was overshot. Arrival is unconfirmed.',
    'ARRIVAL_MAP_COMMIT_FAILED': 'Arrival could not be committed to the onboard map.'})
DFS_REFUSAL_MESSAGES.update({
    'CAMERA_UART_MISSING': 'No camera UART packet has been received.',
    'CAMERA_NO_MARKERS': 'The camera reported no markers in this image.',
    'CAMERA_PROCESSING_STALLED': 'The camera reported stalled image processing.',
    'CAMERA_LOST': 'The camera reported that its image source is lost.',
    'CAMERA_PACKET_STALE': 'The latest camera UART packet exceeded the receive-age limit.',
    'CAMERA_IMAGE_STALE': 'The camera image exceeded the total age limit.',
    'CAMERA_ACCEPTED_IMAGE_STALE': 'No new accepted camera image arrived within the receive-age limit.',
    'CAMERA_IMAGE_UNAVAILABLE': 'No distinct accepted camera image is available.',
    'CAMERA_DATA_UNUSABLE': 'The latest camera input is unusable.',
    'TARGET_NOT_VISIBLE_AT_START': 'The locked target must be visible when starting so RoboPico can capture its reference baseline.',
    'TRACKING_MARKERS_MISSING': 'Neither locked tracking marker is usable in the latest image.',
    'CAMERA_RECOVERY_TIMEOUT': 'Camera recovery timed out while stopped. See the tracking row for the exact input reason and ages.',
    'LEG_TIME_LIMIT': 'The leg reached its overall 10-second limit, including camera waits.'})


@dataclass(frozen=True)
class DfsReply:
    running: bool
    error: bool
    message: str


def parse_dfs_reply(payload):
    if not isinstance(payload, str):
        raise ValueError('Expected DFS reply')
    kind, separator, code = payload.partition(':')
    if not separator or kind not in {'DFS_ACK', 'DFS_REFUSED', 'DFS_EVENT'} or not 1 <= len(code) <= 64 or any(c not in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ_' for c in code):
        raise ValueError('Invalid DFS reply')
    if kind == 'DFS_ACK':
        if code not in {'STARTED', 'ALREADY_RUNNING', 'STOPPED'}:
            raise ValueError('Invalid DFS acknowledgement')
        running = code != 'STOPPED'
        return DfsReply(running, False, 'Onboard DFS is running: scanning exits and planning travel.' if running else 'DFS stopped.')
    if kind == 'DFS_REFUSED':
        return DfsReply(False, True, DFS_REFUSAL_MESSAGES.get(code, 'RoboPico refused DFS: ' + code))
    if code == 'ARRIVED':
        return DfsReply(False, False, 'RoboPico confirmed arrival at the locked target.')
    if code == 'NODE_ARRIVED':
        return DfsReply(True, False, 'Node reached. RoboPico will check exits and continue DFS.')
    if code == 'COMPLETE':
        return DfsReply(False, False, 'DFS complete: all reachable mapped exits have been resolved and explored.')
    message = {'NEAR_TARGET_UNCONFIRMED': 'Approach stopped near the target. Arrival is not confirmed.',
               'DRIVE_TIME_LIMIT': 'Approach stopped after 3 seconds of driving. Arrival is not confirmed.',
               'CANCELLED': 'Onboard approach cancelled.'}.get(code, DFS_REFUSAL_MESSAGES.get(code, 'Onboard approach stopped: ' + code))
    return DfsReply(False, True, message)


@dataclass(frozen=True)
class DfsTrackingReport:
    leg_id: int
    active: bool
    phase: str
    source: str
    remaining_mm: object
    reference_progress_mm: int
    reference_model_valid: bool
    verification_frames: int
    result: str
    input_reason: str = 'NONE'
    camera_present: bool = False
    camera_status: int = 0
    camera_packet_age_ms: int = 0
    camera_image_age_ms: int = 0
    accepted_image_age_ms: int = 0
    recovery_frames: int = 0
    driving_ms: int = 0

    @property
    def change_key(self):
        # Record decisions/counts, not every changing age/distance value.
        return (self.leg_id, self.active, self.phase, self.source, self.reference_model_valid,
                self.verification_frames, self.recovery_frames, self.input_reason, self.result)

    @property
    def log_text(self):
        if not self.camera_present:
            return self.display_text + ' | camera ages: unavailable/unreported'
        return self.display_text + ' | camera ages: packet=%d ms, image=%d ms, accepted=%d ms' % (
            self.camera_packet_age_ms, self.camera_image_age_ms, self.accepted_image_age_ms)

    @property
    def display_text(self):
        remaining = '--' if self.remaining_mm is None else '~%d mm' % self.remaining_mm
        text = 'Tracking: %s | estimated remaining: %s | reference progress: %d mm | verification: %d/3 | %s' % (
            self.source, remaining, self.reference_progress_mm, self.verification_frames, self.result)
        if self.input_reason != 'NONE' or self.phase == 'HOLDING':
            text += ' | input: %s | UART %d ms / image %d ms / accepted %d ms | recovery %d/2' % (
                self.input_reason, self.camera_packet_age_ms, self.camera_image_age_ms,
                self.accepted_image_age_ms, self.recovery_frames)
        return text


def parse_dfs_tracking(payload):
    try:
        value = json.loads(payload) if isinstance(payload, str) else payload
    except (TypeError, ValueError) as error:
        raise ValueError('Invalid tracking JSON') from error
    fields = tuple(DfsTrackingReport.__dataclass_fields__)
    optional = {'input_reason': 'NONE', 'camera_present': False, 'camera_status': 0,
        'camera_packet_age_ms': 0, 'camera_image_age_ms': 0, 'accepted_image_age_ms': 0,
        'recovery_frames': 0, 'driving_ms': 0}
    if not isinstance(value, dict) or not (set(fields) - optional.keys()) <= value.keys():
        raise ValueError('Incomplete tracking report')
    value = {**optional, **value}
    for key, low, high in (('leg_id', 0, 0xffffffff), ('reference_progress_mm', -65535, 65535),
                           ('verification_frames', 0, 3), ('camera_status', 0, 255), ('recovery_frames', 0, 2),
                           ('camera_packet_age_ms', 0, 0xffffffff), ('camera_image_age_ms', 0, 0xffffffff),
                           ('accepted_image_age_ms', 0, 0xffffffff), ('driving_ms', 0, 0xffffffff)):
        if type(value[key]) is not int or not low <= value[key] <= high:
            raise ValueError('Invalid tracking value')
    remaining = value['remaining_mm']
    if remaining is not None and (type(remaining) is not int or not -65535 <= remaining <= 65535):
        raise ValueError('Invalid remaining distance')
    if any(type(value[key]) is not bool for key in ('active', 'reference_model_valid', 'camera_present')):
        raise ValueError('Invalid tracking flag')
    if not isinstance(value['phase'], str) or value['phase'] not in LEG_PHASES or not isinstance(value['source'], str) or value['source'] not in {'NONE', 'TARGET', 'REFERENCE'}:
        raise ValueError('Invalid tracking source/phase')
    if not isinstance(value['result'], str) or not 1 <= len(value['result']) <= 64 or any(c not in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ_' for c in value['result']):
        raise ValueError('Invalid tracking result')
    if not isinstance(value['input_reason'], str) or not 1 <= len(value['input_reason']) <= 64 or any(c not in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ_' for c in value['input_reason']):
        raise ValueError('Invalid tracking input reason')
    return DfsTrackingReport(**{key: value[key] for key in fields})


@dataclass(frozen=True)
class DfsStateReport:
    active: bool
    stage: str
    current_node: int
    depth: int
    direction: str
    expected_node: int
    detail: str

    @property
    def display_text(self):
        return 'DFS: %s | node: %s | stack depth: %d | direction: %s | expected marker: %s | %s' % (
            self.stage, self.current_node or '--', self.depth, self.direction, self.expected_node or '--', self.detail)


def parse_dfs_state(payload):
    try:
        value = json.loads(payload) if isinstance(payload, str) else payload
    except (TypeError, ValueError) as error:
        raise ValueError('Invalid DFS state JSON') from error
    fields = tuple(DfsStateReport.__dataclass_fields__)
    if not isinstance(value, dict) or not set(fields) <= value.keys() or type(value['active']) is not bool:
        raise ValueError('Incomplete DFS state')
    if not isinstance(value['stage'], str) or value['stage'] not in {'IDLE','AT_NODE','TURNING','SCANNING','WAIT_TARGET','TRAVELLING','COMPLETE','STOPPED'}:
        raise ValueError('Invalid DFS stage')
    for key in ('current_node','expected_node','depth'):
        if type(value[key]) is not int or not 0 <= value[key] <= 255:
            raise ValueError('Invalid DFS node/depth')
    if not isinstance(value['direction'], str) or value['direction'] not in {'NORTH','EAST','SOUTH','WEST','UNKNOWN'}:
        raise ValueError('Invalid DFS direction')
    if not isinstance(value['detail'], str) or not 1 <= len(value['detail']) <= 64 or any(c not in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ_' for c in value['detail']):
        raise ValueError('Invalid DFS detail')
    return DfsStateReport(**{key:value[key] for key in fields})


@dataclass(frozen=True)
class TargetSelectionReport:
    state: str
    request: str
    leg_id: int
    role: str
    direction: object
    marker: object
    confirmations: int
    available: bool
    range_mm: int
    bearing_cdeg: int
    detail: str
    reference_marker: object = None
    reference_confirmations: int = 0
    reference_available: bool = False
    reference_range_mm: int = 0
    reference_bearing_cdeg: int = 0

    @property
    def display_text(self):
        marker = '--' if self.marker is None else str(self.marker)
        observation = ('%d mm, %+.2f°' % (self.range_mm, self.bearing_cdeg / 100)
                       if self.available else 'observation unavailable')
        if self.role == 'AUTOMATIC':
            reference = '--' if self.reference_marker is None else str(self.reference_marker)
            reference_observation = ('%d mm, %+.2f°' % (self.reference_range_mm, self.reference_bearing_cdeg / 100)
                                     if self.reference_available else 'unavailable/unconfirmed')
            return '%s | target %s (%s) | reference %s (%s) | %s' % (
                self.state, marker, observation, reference, reference_observation, self.detail)
        return '%s | %s marker %s | %s | %s' % (self.state, self.role, marker, observation, self.detail)

    @property
    def change_key(self):
        # Measurements can refresh without flooding the command log.
        return (self.state, self.request, self.leg_id, self.role, self.direction,
                self.marker, self.confirmations, self.available, self.detail,
                self.reference_marker, self.reference_confirmations, self.reference_available)


def parse_target_selection(payload):
    try:
        value = json.loads(payload) if isinstance(payload, str) else payload
    except (TypeError, ValueError) as error:
        raise ValueError('Invalid target-selection JSON') from error
    fields = tuple(TargetSelectionReport.__dataclass_fields__)
    optional = {'reference_marker': None, 'reference_confirmations': 0, 'reference_available': False,
                'reference_range_mm': 0, 'reference_bearing_cdeg': 0}
    if not isinstance(value, dict) or not (set(fields) - optional.keys()) <= value.keys():
        raise ValueError('Incomplete target selection')
    value = {**optional, **value}
    for key, choices in (('state', {'NONE', 'WAITING', 'LOCKED', 'TIMED_OUT', 'CANCELLED'}),
                         ('role', {'AUTOMATIC', 'DESTINATION', 'REFERENCE'})):
        if not isinstance(value[key], str) or value[key] not in choices:
            raise ValueError('Invalid selection state/role')
    request = value['request']
    if not isinstance(request, str) or (request and (len(request) != 32 or any(c not in '0123456789abcdef' for c in request))):
        raise ValueError('Invalid selection request')
    if value['direction'] is not None and (not isinstance(value['direction'], str) or value['direction'] not in {'NORTH', 'EAST', 'SOUTH', 'WEST'}):
        raise ValueError('Invalid selection direction')
    if value['marker'] is not None and (type(value['marker']) is not int or not 1 <= value['marker'] <= 255):
        raise ValueError('Invalid selection marker')
    if value['reference_marker'] is not None and (type(value['reference_marker']) is not int or not 1 <= value['reference_marker'] <= 255):
        raise ValueError('Invalid reference marker')
    for key, low, high in (('leg_id', 0, 0xffffffff), ('confirmations', 0, 3),
                           ('range_mm', 0, 65535), ('bearing_cdeg', -32768, 32767),
                           ('reference_confirmations', 0, 3), ('reference_range_mm', 0, 65535),
                           ('reference_bearing_cdeg', -32768, 32767)):
        if type(value[key]) is not int or not low <= value[key] <= high:
            raise ValueError('Invalid selection measurement/identity')
    detail = value['detail']
    if type(value['available']) is not bool or type(value['reference_available']) is not bool or not isinstance(detail, str) or not 1 <= len(detail) <= 64 or any(c not in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ_' for c in detail):
        raise ValueError('Invalid selection availability/detail')
    return TargetSelectionReport(**{key: value[key] for key in fields})


@dataclass(frozen=True)
class TravelLegReport:
    leg_id: int
    departure_node: object
    next_target_node: object
    arrival_reference_node: object
    target_cell: object
    direction: object
    phase: str
    configured: bool

    @property
    def display_fields(self):
        return {'leg_id': str(self.leg_id) if self.leg_id else '--',
                'departure_node': '--' if self.departure_node is None else str(self.departure_node),
                'next_target_node': '--' if self.next_target_node is None else str(self.next_target_node),
                'arrival_reference_node': '--' if self.arrival_reference_node is None else str(self.arrival_reference_node),
                'target_cell': '--' if self.target_cell is None else '(%d, %d)' % self.target_cell,
                'direction': self.direction or 'Unknown', 'phase': self.phase,
                'configured': 'Yes' if self.configured else 'No'}


def parse_travel_leg(value):
    if value is None:
        return None # Older firmware did not report leg storage.
    if not isinstance(value, dict) or not set(LEG_FIELDS) <= value.keys():
        raise ValueError('Incomplete travel leg')
    leg_id = value['leg_id']
    if type(leg_id) is not int or not 0 <= leg_id <= 0xffffffff:
        raise ValueError('Invalid leg ID')
    nodes = []
    for key in ('departure_node', 'next_target_node', 'arrival_reference_node'):
        node = value[key]
        if node is not None and (type(node) is not int or not 1 <= node <= 255):
            raise ValueError('Invalid leg node')
        nodes.append(node)
    cell = value['target_cell']
    if cell is not None:
        if not isinstance(cell, dict) or any(type(cell.get(key)) is not int or not -32768 <= cell[key] <= 32767 for key in ('x', 'y')):
            raise ValueError('Invalid target cell')
        cell = cell['x'], cell['y']
    direction = value['direction']
    if direction is not None and (not isinstance(direction, str) or direction not in {'NORTH', 'EAST', 'SOUTH', 'WEST'}):
        raise ValueError('Invalid leg direction')
    if not isinstance(value['phase'], str) or value['phase'] not in LEG_PHASES or type(value['configured']) is not bool:
        raise ValueError('Invalid leg phase/configured flag')
    return TravelLegReport(leg_id, *nodes, cell, direction, value['phase'], value['configured'])


@dataclass(frozen=True)
class MissionReport:
    state: str
    reason: str
    test_mode: bool
    motion_enabled: bool
    revision: int
    leg: object = None

    @property
    def mode(self):
        if self.test_mode:
            return "Test (stationary)" if not self.motion_enabled else "Test"
        return "Inactive" if self.state in {"IDLE", "FAULT"} and not self.motion_enabled else "Onboard"

    @property
    def display_values(self):
        # Revision/periodic retransmission alone should not create a new log row.
        return self.state, self.reason, self.test_mode, self.motion_enabled, self.leg


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
        leg_keys = {'leg_id', 'leg_configured', 'departure_node', 'next_target_node',
                    'arrival_reference_node', 'target_cell', 'direction', 'leg_phase'}
        leg = None
        if leg_keys & fields.keys():
            if not leg_keys <= fields.keys() or fields['leg_configured'] not in {'0', '1'}:
                raise ValueError('Incomplete travel-leg status')
            try:
                cell = None if fields['target_cell'] == 'UNKNOWN' else fields['target_cell'].split(',')
                if cell is not None:
                    if len(cell) != 2:
                        raise ValueError('Invalid target-cell status')
                    cell = {'x': int(cell[0]), 'y': int(cell[1])}
                leg = {'leg_id': int(fields['leg_id']), 'configured': fields['leg_configured'] == '1',
                       'target_cell': cell, 'direction': None if fields['direction'] == 'UNKNOWN' else fields['direction'],
                       'phase': fields['leg_phase']}
                for key in ('departure_node', 'next_target_node', 'arrival_reference_node'):
                    node = int(fields[key])
                    leg[key] = None if node == 0 else node
            except (TypeError, ValueError) as error:
                raise ValueError('Invalid travel-leg status') from error
        value = {
            "state": parts[0][len("MISSION_STATE:"):], "reason": fields["reason"],
            "test_mode": fields["test"] == "1", "motion_enabled": fields["motion"] == "enabled",
            "revision": int(fields["revision"]),
            "leg": leg,
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
    return MissionReport(state, reason, test, motion, revision, parse_travel_leg(value.get('leg')))
