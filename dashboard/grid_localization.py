"""Grid estimates from ordered look-ahead markers and fresh ToF events.

IDs are labels, never coordinates. The operator anchors the starting node;
consecutive, collinear camera markers establish the next two grid positions.
"""
import math
from statistics import median

VECTORS = ((0, 1), (1, 0), (0, -1), (-1, 0))


class GridLocalizer:
    CONFIRM_FRAMES = 5
    MAX_AGE_S = 0.5
    BEARING_DEG = 20.0
    MAX_BEARING_SPREAD_DEG = 10.0
    GRID_PITCH_MM = 600
    HEADING_TOLERANCE_DEG = 15.0
    WALL_APPROACH_MM = 600
    WALL_STOP_MM = 150
    JUNCTION_SAMPLES = 3
    NODE_TOLERANCE_MM = 120.0

    def __init__(self, marker_id, heading, camera_height_mm=325.0, camera_forward_mm=120.0):
        if type(marker_id) is not int or not 1 <= marker_id <= 36:
            raise ValueError("Starting node ID must be 1..36")
        self.cells = {marker_id: (0, 0)}
        self.visited = {marker_id}
        self.fault = None
        self.current = marker_id
        self.zero = heading % 360
        self.camera_height_mm = camera_height_mm
        self.camera_forward_mm = camera_forward_mm
        self._pose_marker = None
        self._pose_count = 0
        self._pose_at = -math.inf
        self._estimate_reason = 'straight look-ahead'
        self._sighting_direction = None
        self._sighting_ids = ()
        self.confirmation_note = 'Waiting for a forward command and fresh marker observations'
        self.forward = False
        self.direction = None
        self.latest = None
        self.terminal = None
        self.approaching_wall = False
        self.wall_stop = False
        self.junction = False
        self.estimate = None
        self.estimate_at = -math.inf
        self.camera_at = -math.inf
        self.pair = None
        self.count = 0
        self._pair_at = -math.inf
        self._leg_has_pair = False
        self._corridor_seen = False
        self._open_count = 0
        self._clear_wall_count = 0
        self._sensor_at = -math.inf
        self.message = "Starting node set; waiting for two aligned markers ahead"

    def rebase_heading(self, delta):
        self.zero = (self.zero - delta) % 360

    def motion(self, command, heading):
        if command == 'FORWARD':
            direction = round(((heading - self.zero) % 360) / 90) % 4
            if not self.forward and direction != self.direction:
                self.direction = direction
                self.latest = self.terminal = self.estimate = None
                self.approaching_wall = self.wall_stop = self.junction = False
                self._leg_has_pair = self._corridor_seen = False
                self._open_count = self._clear_wall_count = self.count = 0
                self.pair = None
            self.forward = True
            self._remember_ahead()
        elif command in ('STOP', 'ESTOP', 'AUTO_STOP', 'RESET_SAFETY'):
            self.forward = False
        elif command.startswith('TURN_TO:') or command in (
                'ROTATE_LEFT', 'ROTATE_RIGHT', 'BACKWARD', 'LEFT', 'RIGHT',
                'FRONT_LEFT', 'FRONT_RIGHT', 'BACK_LEFT', 'BACK_RIGHT'):
            self.forward = False
            self.direction = None
            self.latest = self.terminal = self.estimate = None
            self.approaching_wall = self.wall_stop = self.junction = False
            self._corridor_seen = self._leg_has_pair = False
            self._open_count = self._clear_wall_count = self.count = 0
            self.pair = None
            self._pose_count = 0

    def camera_lost(self):
        self.count = 0
        self.pair = None
        self.estimate = None
        self._pose_count = 0
        self.message = "Waiting for fresh look-ahead markers"
        self.confirmation_note = 'Confirmation reset: waiting for fresh camera/IMU data'

    def observe(self, markers, now, heading):
        self.camera_at = now
        self.record_sightings(markers, heading)
        if self.direction is None:
            self.confirmation_note = 'Position held: rover is stopped or turning'
            return
        target = self.zero + self.direction * 90
        error = (heading - target + 180) % 360 - 180
        if abs(error) > self.HEADING_TOLERANCE_DEG:
            self.camera_lost()
            self.message = "Grid estimate paused: rover is not travelling straight"
            self.confirmation_note = 'Heading error %.1f° exceeds straight-travel limit %.1f°' % (
                abs(error), self.HEADING_TOLERANCE_DEG)
            return
        if self.forward:
            self._remember_ahead()
        aligned = sorted((m for m in markers
                          if m.marker_id != self.current and
                          abs(m.camera_bearing_deg) <= self.BEARING_DEG),
                         key=lambda m: m.camera_distance_mm)
        if self.forward:
            self._observe_pair(aligned, now)
            self.confirmation_note = self.message
        else:
            # A STOP after straight travel does not invalidate position. Use
            # fixed-marker ranges, not the moving two-markers-ahead assumption.
            self.estimate = None
            self.confirmation_note = 'Stopped after straight travel; checking fixed marker position'
        if self.fault is None:
            self._observe_known_pose(aligned, now)

    def _ground_forward(self, marker):
        ground = math.sqrt(max(0, marker.camera_distance_mm ** 2 - self.camera_height_mm ** 2))
        return ground * math.cos(math.radians(marker.camera_bearing_deg)) + self.camera_forward_mm

    def _pin(self, marker, cell):
        if marker in self.cells:
            return self.cells[marker] == cell
        if cell in self.cells.values():
            return False  # Never stack different IDs in the same cell.
        self.cells[marker] = cell
        return True

    def record_sightings(self, markers, heading):
        """Pin first sightings; later ranges can locate the rover, never move a node."""
        ordered = sorted((m for m in markers if m.marker_id != self.current and
                          abs(m.camera_bearing_deg) <= self.BEARING_DEG),
                         key=lambda m: m.camera_distance_mm)
        quarter = round(((heading - self.zero) % 360) / 90) % 4
        vx, vy = VECTORS[quarter]
        origin = self.cells[self.current]
        if self._leg_has_pair and ordered and not any(m.marker_id in self.cells for m in ordered):
            return  # A lost run of IDs cannot establish new grid positions.
        if (len(ordered) == 2 and ordered[0].marker_id != ordered[1].marker_id and
                abs(ordered[0].camera_bearing_deg - ordered[1].camera_bearing_deg) <= self.MAX_BEARING_SPREAD_DEG and
                self.GRID_PITCH_MM * .5 <= ordered[1].camera_distance_mm - ordered[0].camera_distance_mm
                <= self.GRID_PITCH_MM * 1.5 and
                abs((heading - self.zero - quarter * 90 + 180) % 360 - 180) <= self.HEADING_TOLERANCE_DEG):
            near, far = (m.marker_id for m in ordered)
            if near in self.cells:
                first = self.cells[near]
            elif far in self.cells:
                first = self.cells[far][0] - vx, self.cells[far][1] - vy
            else:
                first = origin[0] + vx, origin[1] + vy
            self._pin(near, first)
            self._pin(far, (first[0] + vx, first[1] + vy))
        angle = math.radians((heading - self.zero) % 360)
        for marker in markers:
            if marker.marker_id in self.cells or marker.camera_distance_mm > 6000:
                continue
            ground = math.sqrt(max(0, marker.camera_distance_mm ** 2 - self.camera_height_mm ** 2))
            right = ground * math.sin(math.radians(marker.camera_bearing_deg))
            ahead = self._ground_forward(marker)
            cell = (round(origin[0] + (right * math.cos(angle) + ahead * math.sin(angle)) / self.GRID_PITCH_MM),
                    round(origin[1] + (ahead * math.cos(angle) - right * math.sin(angle)) / self.GRID_PITCH_MM))
            self._pin(marker.marker_id, cell)
        if abs((heading - self.zero - quarter * 90 + 180) % 360 - 180) <= self.HEADING_TOLERANCE_DEG:
            self._sighting_direction = quarter
            self._sighting_ids = tuple(m.marker_id for m in ordered if m.marker_id in self.cells)

    def _remember_ahead(self):
        """Remember seen IDs on this leg without requiring a five-frame arrival."""
        if self.direction is None or self._sighting_direction != self.direction:
            return
        vx, vy = VECTORS[self.direction]
        origin = self.cells[self.current]
        candidates = []
        for marker in self._sighting_ids + ((self.latest,) if self.latest is not None else ()):
            cell = self.cells.get(marker)
            if cell is None:
                continue
            dx, dy = cell[0] - origin[0], cell[1] - origin[1]
            distance = dx * vx + dy * vy
            if distance > 0 and dx * vy == dy * vx:
                candidates.append((distance, marker))
        if candidates:
            self.latest = max(candidates)[1]
            if self.approaching_wall and self.terminal is None:
                self.terminal = self.latest

    def confirmation_summary(self, now):
        if self.approaching_wall and self.wall_stop:
            if self.terminal is None:
                return 'Wall stop detected, but no forward terminal marker was remembered'
            return 'Wall stop: terminal node %d; path %s' % (self.terminal, self.path_to(self.terminal))
        if self.estimate is not None and self.estimate != self.current and now - self.estimate_at <= self.MAX_AGE_S:
            return 'Candidate node %d confirmed by %s; awaiting stop/settle' % (
                self.estimate, self._estimate_reason)
        return self.confirmation_note

    def _observe_known_pose(self, aligned, now):
        """Use pinned forward anchors when the last corridor has only one marker visible."""
        if (len(aligned) > 1 and max(m.camera_bearing_deg for m in aligned) -
                min(m.camera_bearing_deg for m in aligned) > self.MAX_BEARING_SPREAD_DEG):
            self._pose_count = 0
            self.confirmation_note = 'Visible marker bearings differ by more than %.0f°' % self.MAX_BEARING_SPREAD_DEG
            return
        vx, vy = VECTORS[self.direction]
        origin = self.cells[self.current]
        current_s = origin[0] * vx + origin[1] * vy
        samples = []
        for marker in aligned:
            cell = self.cells.get(marker.marker_id)
            if cell is None or (cell[0] - origin[0]) * vy != (cell[1] - origin[1]) * vx:
                continue
            samples.append((cell[0] * vx + cell[1] * vy) * self.GRID_PITCH_MM - self._ground_forward(marker))
        if not samples or max(samples) - min(samples) > self.NODE_TOLERANCE_MM * 2:
            self._pose_count = 0
            self.confirmation_note = ('No mapped forward marker on the current grid line' if not samples
                                      else 'Camera anchors disagree by %.0f mm (limit %.0f mm)' %
                                      (max(samples) - min(samples), self.NODE_TOLERANCE_MM * 2))
            return
        position_mm = median(samples)
        step = round(position_mm / self.GRID_PITCH_MM) - current_s
        cell = origin[0] + step * vx, origin[1] + step * vy
        marker = next((label for label, pos in self.cells.items() if pos == cell), None)
        error_mm = abs(position_mm - (current_s + step) * self.GRID_PITCH_MM)
        if step <= 0 or marker is None or error_mm > self.NODE_TOLERANCE_MM:
            self._pose_count = 0
            if step <= 0:
                self.confirmation_note = 'Range still places rover at/before node %d (progress %.0f mm)' % (
                    self.current, position_mm - current_s * self.GRID_PITCH_MM)
            elif marker is None:
                self.confirmation_note = 'Estimated rover cell %s has no mapped marker ID' % (cell,)
            else:
                self.confirmation_note = 'Node %d position error %.0f mm exceeds ±%.0f mm' % (
                    marker, error_mm, self.NODE_TOLERANCE_MM)
            return
        if self._pose_marker == marker and now - self._pose_at <= self.MAX_AGE_S:
            self._pose_count += 1
        else:
            self._pose_marker, self._pose_count = marker, 1
        self._pose_at = now
        self.confirmation_note = 'Node %d: %d/%d position frames | error %.0f mm / ±%.0f mm' % (
            marker, min(self._pose_count, self.CONFIRM_FRAMES), self.CONFIRM_FRAMES,
            error_mm, self.NODE_TOLERANCE_MM)
        if self._pose_count >= self.CONFIRM_FRAMES:
            self.estimate, self.estimate_at = marker, now
            self._estimate_reason = 'position from fixed forward marker'
            self.message = 'Fixed-marker position confirms node %d (%d frames)' % (marker, self._pose_count)

    def _observe_pair(self, aligned, now):
        # Require two consecutive labels along one forward ray. The range-gap
        # band allows slant-range/calibration error without accepting a skipped grid.
        if (len(aligned) != 2 or aligned[0].marker_id == aligned[1].marker_id or
                abs(aligned[0].camera_bearing_deg - aligned[1].camera_bearing_deg) > self.MAX_BEARING_SPREAD_DEG or
                not self.GRID_PITCH_MM * .5 <= aligned[1].camera_distance_mm - aligned[0].camera_distance_mm
                <= self.GRID_PITCH_MM * 1.5):
            self.count = 0
            self.pair = None
            self.estimate = None
            self.message = "Need two distinct aligned markers ahead for a grid estimate"
            return
        ids = tuple(m.marker_id for m in aligned)
        if self.pair == ids and now - self._pair_at <= self.MAX_AGE_S:
            self.count += 1
        else:
            self.pair, self.count = ids, 1
            self.estimate = None
        self._pair_at = now
        if self.count < self.CONFIRM_FRAMES:
            self.message = "Look-ahead markers %s → %s: %d/%d frames" % (
                *ids, self.count, self.CONFIRM_FRAMES)
            return
        near, far = ids
        vx, vy = VECTORS[self.direction]
        origin = self.cells[self.current]
        if near in self.cells:
            first = self.cells[near]
        elif far in self.cells:
            first = (self.cells[far][0] - vx, self.cells[far][1] - vy)
        elif not self._leg_has_pair:
            first = (origin[0] + vx, origin[1] + vy)
        else:
            self.message = "Lost marker overlap; cannot infer skipped marker IDs"
            self.fault = self.message
            return
        second = first[0] + vx, first[1] + vy
        proposed = dict(self.cells)
        for marker, cell in ((near, first), (far, second)):
            if ((marker in proposed and proposed[marker] != cell) or
                    any(other != marker and pos == cell for other, pos in proposed.items())):
                self.estimate = None
                self.message = "Marker/grid conflict; stop and reset the starting node"
                self.fault = self.message
                return
            proposed[marker] = cell
        cell = second[0] - 2 * vx, second[1] - 2 * vy
        progress = (cell[0] - origin[0]) * vx + (cell[1] - origin[1]) * vy
        if ((cell[0] - origin[0]) * vy != (cell[1] - origin[1]) * vx or progress < 0):
            self.estimate = None
            self.message = "Ignoring markers behind the current grid estimate"
            return
        marker = next((label for label, pos in proposed.items() if pos == cell), None)
        if marker is None:
            self.estimate = None
            self.message = "Two-grids-back position has no known marker ID"
            return
        self.cells = proposed
        self.fault = None
        self._leg_has_pair = True
        self.estimate, self.estimate_at = marker, now
        self._estimate_reason = 'straight look-ahead'
        self.message = "Latest ahead: %d | Two grids behind: %d" % (far, marker)

    @staticmethod
    def reading(sensor, name):
        bit = {'left': 0, 'front': 1, 'right': 2}[name]
        value = sensor.distances.get(name)
        if sensor.faults & (1 << bit) or type(value) is not int:
            return None
        return value if 1 <= value <= 1000 or value == 1023 else None

    def sensors(self, sensor, now, threshold):
        if (self.direction is None or sensor.age_s + now - sensor.received_at > self.MAX_AGE_S
                or sensor.received_at <= self._sensor_at):
            return
        self._sensor_at = sensor.received_at
        target = self.zero + self.direction * 90
        if abs((sensor.heading - target + 180) % 360 - 180) > self.HEADING_TOLERANCE_DEG:
            self.wall_stop = self.junction = False
            return
        front = self.reading(sensor, 'front')
        self.wall_stop = front is not None and front < self.WALL_STOP_MM
        if self.forward and front is not None and front < self.WALL_APPROACH_MM:
            self._clear_wall_count = 0
            if not self.approaching_wall or self.terminal is None:
                self.terminal = self.latest
            self.approaching_wall = True
        elif self.forward and front is not None and front >= 800:
            self._clear_wall_count += 1
            if self._clear_wall_count >= 3:
                self.approaching_wall = False
                self.terminal = None
        if not self.forward:
            return
        left, right = (self.reading(sensor, name) for name in ('left', 'right'))
        if left is None or right is None:
            self._open_count = 0
            self.junction = False
            return
        if left < threshold and right < threshold:
            self._corridor_seen = True
            self._open_count = 0
            self.junction = False
        elif left >= threshold and right >= threshold and self._corridor_seen:
            self._open_count += 1
            self.junction = self._open_count >= self.JUNCTION_SAMPLES
        else:
            self._open_count = 0
            self.junction = False

    def candidate(self, now):
        if self.approaching_wall and self.wall_stop:
            return self.terminal, 'terminal wall stop'
        if self.estimate is not None and now - self.estimate_at <= self.MAX_AGE_S:
            return self.estimate, 'cross junction' if self.junction else self._estimate_reason
        return None, None

    def path_to(self, target):
        if target not in self.cells or self.direction is None:
            return None
        vx, vy = VECTORS[self.direction]
        here, dest = self.cells[self.current], self.cells[target]
        steps = (dest[0] - here[0]) * vx + (dest[1] - here[1]) * vy
        if steps < 0 or steps > 35 or (here[0] + steps * vx, here[1] + steps * vy) != dest:
            return None
        by_cell = {pos: marker for marker, pos in self.cells.items()}
        path = [by_cell.get((here[0] + step * vx, here[1] + step * vy))
                for step in range(1, steps + 1)]
        return path if all(marker is not None for marker in path) else None
