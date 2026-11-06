"""
The QR load models in the repository must agree with the generator output for the mission.

This test finds an old texture (a box with a printed QR code that does not agree with its
configured destination). It also finds texture names that are the same in the renderer
cache of Gazebo.
"""

import filecmp
import importlib.util
import pathlib
import xml.etree.ElementTree as ET

import cv2
from forklift_task_manager.mission_config import load_mission
import numpy as np
import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
MODELS = PKG / 'models'

spec = importlib.util.spec_from_file_location('gen', PKG / 'scripts' / 'generate_load_models.py')
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)
MISSION = load_mission(gen.DEFAULT_MISSION)


@pytest.fixture(scope='module')
def regenerated(tmp_path_factory):
    out = tmp_path_factory.mktemp('models')
    return out, gen.generate(out_dir=out)


def test_committed_models_are_up_to_date(regenerated):
    out, written = regenerated
    for name in written:
        for rel in ('model.sdf', 'model.config'):
            assert filecmp.cmp(out / name / rel, MODELS / name / rel, shallow=False), \
                f'{name}/{rel} is stale: run scripts/generate_load_models.py'
        png = f'materials/textures/{name}.png'
        fresh = cv2.imread(str(out / name / png), 0)
        committed = cv2.imread(str(MODELS / name / png), 0)
        assert committed is not None and np.array_equal(fresh, committed), f'{name}: stale'


def test_every_mission_box_has_a_model_with_its_payload(regenerated):
    _, written = regenerated
    for box in MISSION.boxes:
        name = gen.model_name(box.box_id)
        assert written.get(name) == box.payload
        texture = cv2.imread(str(MODELS / name / 'materials' / 'textures' / f'{name}.png'), 0)
        assert cv2.QRCodeDetector().detectAndDecode(texture)[0] == box.payload


def test_texture_file_names_are_unique():
    names = [p.name for p in MODELS.glob('*/materials/textures/*.png')]
    assert names and len(names) == len(set(names))


def test_decoys_are_not_valid_mission_payloads(regenerated):
    _, written = regenerated
    mission_models = {gen.model_name(b.box_id) for b in MISSION.boxes}
    for name, payload in written.items():
        if name not in mission_models:
            assert not MISSION.validate_payload(payload).valid, f'decoy {name} is valid'


def test_model_geometry_matches_load_config():
    sdf = ET.parse(MODELS / gen.model_name(MISSION.boxes[0].box_id) / 'model.sdf').getroot()
    cols = {c.get('name'): c for c in sdf.iter('collision')}
    assert set(cols) == {'deck', 'stringer_0', 'stringer_1', 'stringer_2', 'cargo'}
    load = MISSION.load
    ys = sorted(float(cols[f'stringer_{i}'].findtext('pose').split()[1]) for i in range(3))
    assert ys == pytest.approx(sorted(load.stringer_y))
    mass = float(sdf.find('.//inertial/mass').text)
    assert mass == pytest.approx(load.mass)
    label = [v for v in sdf.iter('visual') if v.get('name') == 'qr_label'][0]
    x, _, z = (float(v) for v in label.findtext('pose').split()[:3])
    assert x == pytest.approx(load.qr_center[0] + 0.001)
    assert z == pytest.approx(load.qr_center[2])
