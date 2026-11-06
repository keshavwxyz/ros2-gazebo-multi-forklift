"""
Mission configuration: load, validation and QR payload check (no ROS).

The YAML file (config/warehouse_mission.yaml) is the only source of the boxes, their
destinations and the warehouse places. All items that use these data use this module (the
perception node, the Gazebo scenario spawner and the task manager). Thus, there is exactly
one definition of a valid box, place or QR payload.
"""

from dataclasses import dataclass
import math
import re

import yaml

ID_PATTERN = re.compile(r'^[A-Z][A-Z0-9_]{0,31}$')
PAYLOAD_SEPARATOR = ':'
QUIET_ZONE_MODULES = 4
V1_MODULES = 21


class MissionConfigError(ValueError):
    """The mission configuration is not valid."""


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Destination:
    dest_id: str
    pose: Pose2D
    surface_height: float
    face_offset: float


@dataclass(frozen=True)
class Box:
    box_id: str
    destination: str
    pickup: str

    @property
    def payload(self):
        return f'{self.box_id}{PAYLOAD_SEPARATOR}{self.destination}'


@dataclass(frozen=True)
class LoadGeometry:
    pallet_depth: float
    pallet_width: float
    pallet_height: float
    deck_thickness: float
    stringer_width: float
    box_depth: float
    box_width: float
    box_height: float
    mass: float

    @property
    def qr_center(self):
        """QR symbol center in the load frame: the middle of the front (+x) face of the box."""
        return (self.box_depth / 2.0, 0.0, self.pallet_height + self.box_height / 2.0)

    @property
    def stringer_y(self):
        half = self.pallet_width / 2.0 - self.stringer_width / 2.0
        return (-half, 0.0, half)

    @property
    def pocket_width(self):
        """Clear width of each fork pocket between two stringers."""
        return self.pallet_width / 2.0 - 1.5 * self.stringer_width

    @property
    def pocket_height(self):
        return self.pallet_height - self.deck_thickness


@dataclass(frozen=True)
class PayloadResult:
    """Result of the validation of a decoded QR payload against the mission."""

    valid: bool
    status: str           # ok | empty | invalid_characters | malformed | too_long |
    #                       unknown_box | unknown_destination | destination_mismatch
    reason: str           # text for persons, empty if valid
    box_id: str = ''
    destination_id: str = ''


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class MissionConfig:
    qr_edge_length: float
    max_payload_length: int
    load: LoadGeometry
    staging_gap: float
    fork_tip_offset: float
    home: Pose2D
    pickups: dict
    destinations: dict
    boxes: tuple          # mission sequence

    def box(self, box_id):
        for b in self.boxes:
            if b.box_id == box_id:
                return b
        raise KeyError(box_id)

    @property
    def box_ids(self):
        return tuple(b.box_id for b in self.boxes)

    def staging_pose(self, place):
        """
        Robot base pose in front of a load pose (pickup or destination).

        The +x axis of the load (QR face) points to the aisle. The robot points in the opposite
        direction. Its fork tips are `staging_gap` in front of the front face of the pallet.
        """
        d = self.load.pallet_depth / 2.0 + self.staging_gap + self.fork_tip_offset
        return Pose2D(place.x + d * math.cos(place.yaw), place.y + d * math.sin(place.yaw),
                      wrap_angle(place.yaw + math.pi))

    def pickup_staging(self, box_id):
        return self.staging_pose(self.pickups[self.box(box_id).pickup])

    def destination_staging(self, destination_id):
        return self.staging_pose(self.destinations[destination_id].pose)

    def validate_payload(self, payload):
        """Examine a decoded payload. It never raises. Bad input gives valid=False."""
        if payload is None or payload == '':
            return PayloadResult(False, 'empty', 'empty payload (QR not decoded)')
        if len(payload) > self.max_payload_length:
            return PayloadResult(False, 'too_long',
                                 f'payload longer than {self.max_payload_length} characters')
        if not payload.isascii() or not payload.isprintable() or any(c.isspace()
                                                                     for c in payload):
            return PayloadResult(False, 'invalid_characters',
                                 'payload contains non-ASCII, whitespace or control characters')
        parts = payload.split(PAYLOAD_SEPARATOR)
        if len(parts) != 2 or not all(ID_PATTERN.match(p) for p in parts):
            return PayloadResult(False, 'malformed',
                                 f'expected "<BOX_ID>:<DESTINATION_ID>", got {payload!r}')
        box_id, dest_id = parts
        known = {b.box_id: b for b in self.boxes}
        if box_id not in known:
            return PayloadResult(False, 'unknown_box', f'box {box_id} is not in the mission',
                                 box_id, dest_id)
        if dest_id not in self.destinations:
            return PayloadResult(False, 'unknown_destination',
                                 f'destination {dest_id} does not exist', box_id, dest_id)
        if known[box_id].destination != dest_id:
            return PayloadResult(
                False, 'destination_mismatch',
                f'{box_id} is assigned to {known[box_id].destination}, QR says {dest_id}',
                box_id, dest_id)
        return PayloadResult(True, 'ok', '', box_id, dest_id)


# ---------------------------------------------------------------------------- loading

def _require(mapping, key, where):
    if not isinstance(mapping, dict) or key not in mapping:
        raise MissionConfigError(f'{where}: missing required key "{key}"')
    return mapping[key]


def _number(value, where, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise MissionConfigError(f'{where}: expected a finite number, got {value!r}')
    if positive and value <= 0:
        raise MissionConfigError(f'{where}: must be > 0, got {value}')
    return float(value)


def _id(value, where):
    if not isinstance(value, str) or not ID_PATTERN.match(value):
        raise MissionConfigError(f'{where}: invalid id {value!r} (expected {ID_PATTERN.pattern})')
    return value


def _pose(data, where):
    return Pose2D(*(_number(_require(data, k, where), f'{where}.{k}') for k in ('x', 'y', 'yaw')))


def parse_mission(data):
    """Validate a mission dict (from YAML) and return a MissionConfig."""
    if not isinstance(data, dict):
        raise MissionConfigError('mission file must contain a mapping')
    if _require(data, 'version', 'mission') != 1:
        raise MissionConfigError(f'unsupported mission version {data["version"]!r}')

    qr = _require(data, 'qr', 'mission')
    edge = _number(_require(qr, 'edge_length', 'qr'), 'qr.edge_length', positive=True)
    max_len = _require(qr, 'max_payload_length', 'qr')
    if not isinstance(max_len, int) or max_len < 3:
        raise MissionConfigError('qr.max_payload_length must be an integer >= 3')

    ld = _require(data, 'load', 'mission')
    pallet, box = _require(ld, 'pallet', 'load'), _require(ld, 'box', 'load')
    load = LoadGeometry(
        *(_number(_require(pallet, k, 'load.pallet'), f'load.pallet.{k}', positive=True)
          for k in ('depth', 'width', 'height', 'deck_thickness')),
        _number(_require(ld, 'stringer_width', 'load'), 'load.stringer_width', positive=True),
        *(_number(_require(box, k, 'load.box'), f'load.box.{k}', positive=True)
          for k in ('depth', 'width', 'height')),
        _number(_require(ld, 'mass', 'load'), 'load.mass', positive=True))
    if load.deck_thickness >= load.pallet_height:
        raise MissionConfigError('load: deck_thickness must be smaller than pallet height')
    if load.pocket_width <= 0:
        raise MissionConfigError('load: stringers leave no room for fork pockets')
    if load.box_depth > load.pallet_depth or load.box_width > load.pallet_width:
        raise MissionConfigError('load: box must fit on the pallet')
    # The printed label (symbol + quiet zone) must fit on the front face of the box. A
    # version-1 symbol (21 modules) is the smallest. Longer payloads make the quiet zone thinner.
    label = edge * (V1_MODULES + 2 * QUIET_ZONE_MODULES) / V1_MODULES
    if label > min(load.box_width, load.box_height):
        raise MissionConfigError(f'qr: label ({label:.3f} m incl. quiet zone) does not fit '
                                 f'on the {load.box_width} x {load.box_height} m box face')

    approach = _require(data, 'approach', 'mission')
    staging_gap = _number(_require(approach, 'staging_gap', 'approach'), 'approach.staging_gap',
                          positive=True)
    fork_tip = _number(_require(approach, 'fork_tip_offset', 'approach'),
                       'approach.fork_tip_offset', positive=True)

    home = _pose(_require(data, 'home', 'mission'), 'home')
    pickups = {_id(k, 'pickups'): _pose(v, f'pickups.{k}')
               for k, v in _require(data, 'pickups', 'mission').items()}
    destinations = {}
    for k, v in _require(data, 'destinations', 'mission').items():
        destinations[_id(k, 'destinations')] = Destination(
            k, _pose(v, f'destinations.{k}'),
            _number(_require(v, 'surface_height', f'destinations.{k}'),
                    f'destinations.{k}.surface_height'),
            _number(_require(v, 'face_offset', f'destinations.{k}'),
                    f'destinations.{k}.face_offset', positive=True))
    if set(pickups) & set(destinations):
        raise MissionConfigError('pickup and destination ids must be distinct')

    raw_boxes = _require(data, 'boxes', 'mission')
    if not isinstance(raw_boxes, list) or not raw_boxes:
        raise MissionConfigError('boxes: must be a non-empty list')
    boxes, used_pickups = [], set()
    for i, entry in enumerate(raw_boxes):
        where = f'boxes[{i}]'
        b = Box(_id(_require(entry, 'id', where), f'{where}.id'),
                _id(_require(entry, 'destination', where), f'{where}.destination'),
                _id(_require(entry, 'pickup', where), f'{where}.pickup'))
        if b.box_id in {x.box_id for x in boxes}:
            raise MissionConfigError(f'{where}: duplicate box id {b.box_id}')
        if b.box_id in pickups or b.box_id in destinations:
            raise MissionConfigError(f'{where}: box id {b.box_id} collides with a place id')
        if b.destination not in destinations:
            raise MissionConfigError(f'{where}: unknown destination {b.destination}')
        if b.pickup not in pickups:
            raise MissionConfigError(f'{where}: unknown pickup {b.pickup}')
        if b.pickup in used_pickups:
            raise MissionConfigError(f'{where}: pickup {b.pickup} already holds another box')
        if len(b.payload) > max_len:
            raise MissionConfigError(f'{where}: payload {b.payload!r} exceeds '
                                     f'qr.max_payload_length')
        used_pickups.add(b.pickup)
        boxes.append(b)

    return MissionConfig(edge, max_len, load, staging_gap, fork_tip, home, pickups,
                         destinations, tuple(boxes))


def load_mission(path):
    """Load and validate a mission YAML file."""
    with open(path, encoding='utf-8') as f:
        try:
            data = yaml.safe_load(f)
        except yaml.YAMLError as exc:
            raise MissionConfigError(f'{path}: invalid YAML: {exc}') from exc
    return parse_mission(data)


def default_mission_path():
    """Installed path of the default mission file."""
    from ament_index_python.packages import get_package_share_directory
    import os
    return os.path.join(get_package_share_directory('forklift_task_manager'), 'config',
                        'warehouse_mission.yaml')
