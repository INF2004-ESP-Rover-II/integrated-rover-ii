"""Read-only, validated mirror of RoboPico's authoritative sparse grid map."""
from dataclasses import dataclass


def bounded_int(value, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError('Invalid map integer')
    return value


@dataclass(frozen=True)
class MapNode:
    marker_id: int
    x: int
    y: int
    visited: bool
    exits: tuple


@dataclass(frozen=True)
class MapSnapshot:
    boot_id: str
    revision: int
    capacity: int
    last_request: str
    current_node: object
    bounds: object
    columns: int
    rows: int
    nodes: tuple
    observed_nodes: tuple = ()

    @property
    def display_nodes(self):
        return self.nodes + self.observed_nodes

    @property
    def display_bounds(self):
        nodes = self.display_nodes
        return None if not nodes else (min(n.x for n in nodes), max(n.x for n in nodes),
                                      min(n.y for n in nodes), max(n.y for n in nodes))


def decode_map(data):
    if not isinstance(data, dict) or type(data.get('schema')) is not int or data['schema'] != 1 or data.get('source') != 'robo_pico':
        raise ValueError('Expected RoboPico map snapshot')
    boot = data['boot_id']
    request = data['last_request']
    if not isinstance(boot, str) or len(boot) != 16 or any(c not in '0123456789abcdef' for c in boot):
        raise ValueError('Invalid map boot ID')
    if not isinstance(request, str) or (request and (len(request) != 32 or any(c not in '0123456789abcdef' for c in request))):
        raise ValueError('Invalid map request ID')
    revision = bounded_int(data['revision'], 0, 0xffffffff)
    capacity = bounded_int(data['capacity'], 1, 255)
    raw_nodes = data['nodes']
    if not isinstance(raw_nodes, list) or len(raw_nodes) > capacity:
        raise ValueError('Invalid map node count')
    nodes, ids, cells = [], set(), set()
    for item in raw_nodes:
        if not isinstance(item, dict):
            raise ValueError('Invalid map node')
        marker_id = bounded_int(item['marker_id'], 1, 255)
        x, y = bounded_int(item['x'], -32768, 32767), bounded_int(item['y'], -32768, 32767)
        if type(item['visited']) is not bool or not isinstance(item['exits'], list) or len(item['exits']) != 4:
            raise ValueError('Invalid node state')
        exits = tuple(bounded_int(exit_state, 0, 3) for exit_state in item['exits'])
        if marker_id in ids or (x, y) in cells:
            raise ValueError('Conflicting node ID or grid coordinate')
        ids.add(marker_id)
        cells.add((x, y))
        nodes.append(MapNode(marker_id, x, y, item['visited'], exits))
    current = data['current_node']
    if current is not None:
        bounded_int(current, 1, 255)
        if not any(node.marker_id == current and node.visited for node in nodes):
            raise ValueError('Current node must reference a visited node')
    columns, rows = bounded_int(data['columns'], 0, 65536), bounded_int(data['rows'], 0, 65536)
    bounds = data['bounds']
    if nodes:
        expected = (min(n.x for n in nodes), max(n.x for n in nodes), min(n.y for n in nodes), max(n.y for n in nodes))
        if not isinstance(bounds, dict):
            raise ValueError('Missing map bounds')
        actual = tuple(bounded_int(bounds[key], -32768, 32767) for key in ('min_x', 'max_x', 'min_y', 'max_y'))
        if actual != expected or columns != expected[1] - expected[0] + 1 or rows != expected[3] - expected[2] + 1:
            raise ValueError('Inconsistent map dimensions')
        bounds = expected
    elif bounds is not None or columns != 0 or rows != 0 or current is not None:
        raise ValueError('Empty map must have empty bounds/current node')
    raw_observed = data.get('observed_nodes', [])
    if not isinstance(raw_observed, list) or len(nodes) + len(raw_observed) > capacity or (raw_observed and not nodes):
        raise ValueError('Invalid observed node count')
    observed = []
    for item in raw_observed:
        if not isinstance(item, dict) or item.get('visited', False) is not False:
            raise ValueError('Camera observations cannot confirm visits')
        marker_id = bounded_int(item['marker_id'], 1, 255)
        x, y = bounded_int(item['x'], -32768, 32767), bounded_int(item['y'], -32768, 32767)
        if marker_id in ids or (x, y) in cells:
            raise ValueError('Conflicting observed marker or coordinate')
        ids.add(marker_id)
        cells.add((x, y))
        observed.append(MapNode(marker_id, x, y, False, (0, 0, 0, 0)))
    return MapSnapshot(boot, revision, capacity, request, current, bounds, columns, rows,
                       tuple(sorted(nodes, key=lambda node: node.marker_id)),
                       tuple(sorted(observed, key=lambda node: node.marker_id)))


class OnboardMapCache:
    def __init__(self):
        self.snapshot = None
        self.received_at = None
        self.last_confirmed_node = None
        self.retired_boots = set()

    def update(self, data, now):
        try:
            incoming = decode_map(data)
        except (KeyError, TypeError, ValueError):
            return False
        if incoming.boot_id in self.retired_boots:
            return False
        previous = self.snapshot
        if previous is not None and incoming.boot_id == previous.boot_id:
            delta = (incoming.revision - previous.revision) & 0xffffffff
            if delta >= 0x80000000 or (delta == 0 and incoming != previous):
                return False
        elif previous is not None:
            self.retired_boots.add(previous.boot_id)
            self.last_confirmed_node = None
        if incoming.current_node is not None:
            self.last_confirmed_node = incoming.current_node
        elif not incoming.nodes:
            self.last_confirmed_node = None
        self.snapshot = incoming
        self.received_at = now
        return True

    def is_fresh(self, now):
        return self.received_at is not None and now - self.received_at <= 5.0
