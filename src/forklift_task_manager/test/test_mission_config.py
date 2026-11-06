"""Unit tests for the load of the mission configuration and the QR payload validation."""

import copy
import math
import pathlib

from forklift_task_manager.mission_config import (load_mission, MissionConfigError,
                                                  parse_mission)
import pytest
import yaml

MISSION_FILE = pathlib.Path(__file__).resolve().parents[1] / 'config' / 'warehouse_mission.yaml'
RAW = yaml.safe_load(MISSION_FILE.read_text())


@pytest.fixture(scope='module')
def mission():
    return load_mission(MISSION_FILE)


def mutated(fn):
    data = copy.deepcopy(RAW)
    fn(data)
    return data


# ---------------------------------------------------------------- the mission in the package

def test_shipped_mission_is_valid(mission):
    assert mission.box_ids == ('BOX_A', 'BOX_B', 'BOX_C')
    assert [b.payload for b in mission.boxes] == [
        'BOX_A:SHELF_01', 'BOX_B:SHELF_03', 'BOX_C:SHELF_02']
    assert set(mission.destinations) == {'SHELF_01', 'SHELF_02', 'SHELF_03'}
    assert mission.qr_edge_length == pytest.approx(0.30)


def test_load_geometry_admits_the_forks(mission):
    """The fork tines (0.10 m wide, 0.40 m apart, 0.04 m thick) fit in the pallet pockets."""
    load = mission.load
    fork_inner, fork_outer = 0.20 - 0.05, 0.20 + 0.05
    stringer_outer_edge_of_centre = load.stringer_width / 2
    stringer_inner_edge_of_side = load.stringer_y[2] - load.stringer_width / 2
    assert fork_inner > stringer_outer_edge_of_centre + 0.05     # side clearance of 5 cm or more
    assert fork_outer < stringer_inner_edge_of_side - 0.05
    # the lowered fork top at 0.05 m must go below the deck with clearance
    assert 0.05 + 0.03 < load.pocket_height


def test_qr_centre_is_on_the_box_front_face(mission):
    x, y, z = mission.load.qr_center
    assert x == pytest.approx(mission.load.box_depth / 2)
    assert y == 0.0
    assert z == pytest.approx(0.15 + 0.25)


# ---------------------------------------------------------------- staging poses

def test_pickup_staging_faces_the_load(mission):
    s = mission.pickup_staging('BOX_B')           # load at (0, -4.6), pointed to +y
    d = 0.40 + 0.8 + 1.38
    # The YAML file gives yaw as 1.5708 (not exactly pi/2): compare to 1e-4
    assert (s.x, s.y) == pytest.approx((0.0, -4.6 + d), abs=1e-4)
    assert s.yaw == pytest.approx(-math.pi / 2, abs=1e-4)


def test_destination_staging_faces_the_platform(mission):
    s = mission.destination_staging('SHELF_02')   # load pose (0, 5.3), pointed to -y
    assert (s.x, s.y) == pytest.approx((0.0, 5.3 - (0.40 + 0.8 + 1.38)), abs=1e-4)
    assert s.yaw == pytest.approx(math.pi / 2, abs=1e-4)


def test_fork_tips_stop_short_of_the_pallet(mission):
    for box in mission.boxes:
        s, p = mission.pickup_staging(box.box_id), mission.pickups[box.pickup]
        tip = (s.x + 1.38 * math.cos(s.yaw), s.y + 1.38 * math.sin(s.yaw))
        pallet_front = (p.x + 0.4 * math.cos(p.yaw), p.y + 0.4 * math.sin(p.yaw))
        assert math.dist(tip, pallet_front) == pytest.approx(mission.staging_gap)


# ---------------------------------------------------------------- payload validation

@pytest.mark.parametrize('payload,box,dest', [
    ('BOX_A:SHELF_01', 'BOX_A', 'SHELF_01'),
    ('BOX_B:SHELF_03', 'BOX_B', 'SHELF_03'),
    ('BOX_C:SHELF_02', 'BOX_C', 'SHELF_02'),
])
def test_valid_payloads(mission, payload, box, dest):
    r = mission.validate_payload(payload)
    assert r.valid and r.status == 'ok' and r.reason == ''
    assert (r.box_id, r.destination_id) == (box, dest)


@pytest.mark.parametrize('payload,status', [
    ('', 'empty'),
    (None, 'empty'),
    ('BOX_A', 'malformed'),
    ('BOX_A:SHELF_01:X', 'malformed'),
    ('box_a:shelf_01', 'malformed'),
    (':SHELF_01', 'malformed'),
    ('BOX_A:', 'malformed'),
    ('BOX-A:SHELF_01', 'malformed'),
    ('https://example.com', 'malformed'),
    ('BOX_A :SHELF_01', 'invalid_characters'),
    ('BOX_A:SHELF_01\n', 'invalid_characters'),
    ('BOX_Ä:SHELF_01', 'invalid_characters'),
    ('BOX_A:SHELF_01\x00', 'invalid_characters'),
    ('A' * 65, 'too_long'),
    ('BOX_Z:SHELF_01', 'unknown_box'),
    ('BOX_A:SHELF_99', 'unknown_destination'),
    ('BOX_A:SHELF_02', 'destination_mismatch'),
])
def test_invalid_payloads_rejected_with_reason(mission, payload, status):
    r = mission.validate_payload(payload)
    assert not r.valid
    assert r.status == status
    assert r.reason


# ---------------------------------------------------------------- config validation

@pytest.mark.parametrize('mutate,message', [
    (lambda d: d.update(version=2), 'unsupported mission version'),
    (lambda d: d['boxes'].append(dict(d['boxes'][0])), 'duplicate box id'),
    (lambda d: d['boxes'][0].update(destination='SHELF_09'), 'unknown destination'),
    (lambda d: d['boxes'][0].update(pickup='PICK_09'), 'unknown pickup'),
    (lambda d: d['boxes'][1].update(pickup='PICK_01'), 'already holds another box'),
    (lambda d: d['boxes'][0].update(id='box_a'), 'invalid id'),
    (lambda d: d['boxes'][0].pop('destination'), 'missing required key "destination"'),
    (lambda d: d.update(boxes=[]), 'non-empty list'),
    (lambda d: d['qr'].update(edge_length=-0.3), 'must be > 0'),
    (lambda d: d['qr'].update(edge_length=0.5), 'does not fit'),
    (lambda d: d['qr'].update(edge_length='big'), 'finite number'),
    (lambda d: d['pickups']['PICK_01'].update(x=float('nan')), 'finite number'),
    (lambda d: d['load']['pallet'].update(deck_thickness=0.2), 'deck_thickness'),
    (lambda d: d['load']['box'].update(width=1.0), 'box must fit'),
    (lambda d: d['destinations'].update(PICK_01=d['destinations']['SHELF_01']), 'distinct'),
    (lambda d: d['approach'].update(staging_gap=0), 'must be > 0'),
    (lambda d: d['destinations']['SHELF_01'].pop('face_offset'), 'missing required key'),
    (lambda d: d.pop('approach'), 'missing required key "approach"'),
])
def test_invalid_configs_rejected(mutate, message):
    with pytest.raises(MissionConfigError, match=message):
        parse_mission(mutated(mutate))


def test_invalid_yaml_rejected(tmp_path):
    f = tmp_path / 'bad.yaml'
    f.write_text('boxes: [unclosed')
    with pytest.raises(MissionConfigError, match='invalid YAML'):
        load_mission(f)
