"""Unit tests for QR generation, detection, payload validation and pose estimation."""

import math
import pathlib

import cv2
from forklift_perception import synthetic_scene as scene
from forklift_perception.pose_estimation import (confidence, estimate_qr_pose, object_points,
                                                 rotation_to_quaternion)
from forklift_perception.qr_codec import label_edge, qr_modules, render_qr_label
from forklift_perception.qr_detection import QrDetector
from forklift_task_manager.mission_config import load_mission
import numpy as np
import pytest

K = scene.camera_matrix(1280, 720, 1.2217)          # the same as the Gazebo camera
EDGE = 0.30


def mission_path():
    src = pathlib.Path(__file__).resolve().parents[2] / 'forklift_task_manager' / 'config'
    if (src / 'warehouse_mission.yaml').exists():
        return src / 'warehouse_mission.yaml'
    from forklift_task_manager.mission_config import default_mission_path
    return default_mission_path()


MISSION = load_mission(mission_path())


def forklift_view(distance, yaw_deg=0.0, pitch_deg=40.0, lateral=0.0):
    """Label as the forklift camera sees a box face: from above and possibly at an angle."""
    pitch, yaw = math.radians(pitch_deg), math.radians(yaw_deg)
    normal = np.array([math.sin(yaw) * math.cos(pitch), -math.sin(pitch),
                       -math.cos(pitch) * math.cos(yaw)])
    return scene.look_at_rotation(normal), np.array([lateral, 0.0, distance])


def rotation_error_deg(a, b):
    return math.degrees(math.acos(np.clip((np.trace(a.T @ b) - 1) / 2, -1, 1)))


# ---------------------------------------------------------------- generation

def test_mission_payloads_fit_version_1():
    for box in MISSION.boxes:
        assert qr_modules(box.payload).shape == (21, 21)


def test_label_geometry():
    img, n = render_qr_label('BOX_A:SHELF_01', px_per_module=10)
    assert n == 21 and img.shape == (290, 290)
    assert label_edge(0.30, 21) == pytest.approx(0.30 * 29 / 21)
    # the quiet zone is white
    assert img[:40, :].min() == 255 and img[:, :40].min() == 255


def test_label_round_trip():
    img, _ = render_qr_label('BOX_C:SHELF_02', px_per_module=8)
    assert cv2.QRCodeDetector().detectAndDecode(img)[0] == 'BOX_C:SHELF_02'


def test_empty_payload_rejected_by_encoder():
    with pytest.raises(ValueError):
        qr_modules('')


# ---------------------------------------------------------------- detection

@pytest.mark.parametrize('payload', [b.payload for b in MISSION.boxes])
def test_each_box_code_detected_and_decoded(payload):
    rot, t = forklift_view(2.4)
    found = QrDetector().detect(scene.render([(payload, rot, t)], K))
    assert [c.payload for c in found] == [payload]


def test_multiple_codes_in_one_frame():
    labels = []
    for box, x in zip(MISSION.boxes, (-0.8, 0.0, 0.8)):
        rot, t = forklift_view(2.6, lateral=x)
        labels.append((box.payload, rot, t))
    found = QrDetector().detect(scene.render(labels, K))
    assert sorted(c.payload for c in found) == sorted(b.payload for b in MISSION.boxes)


def test_no_detection_in_empty_image():
    assert QrDetector().detect(scene.render([], K)) == []


def test_damaged_code_is_not_decoded():
    rot, t = forklift_view(2.0)
    img = scene.render([('BOX_A:SHELF_01', rot, t)], K)
    found = QrDetector().detect(img)
    c = found[0].centre_px.astype(int)
    img[c[1] - 25:c[1] + 25, c[0] - 25:c[0] + 25] = 255     # smear on the data area
    damaged = QrDetector().detect(img)
    assert all(d.payload != 'BOX_A:SHELF_01' for d in damaged)


def test_too_far_code_is_not_reported_as_valid():
    rot, t = forklift_view(9.0)
    for d in QrDetector().detect(scene.render([('BOX_A:SHELF_01', rot, t)], K)):
        assert d.payload in ('', 'BOX_A:SHELF_01')   # never an incorrect decode


def test_detector_rejects_colour_input():
    with pytest.raises(ValueError):
        QrDetector().detect(np.zeros((10, 10, 3), np.uint8))


# ---------------------------------------------------------------- payload validation

@pytest.mark.parametrize('payload,status', [
    ('hello world', 'invalid_characters'),
    ('PALLET-42', 'malformed'),
    ('BOX_Z:SHELF_01', 'unknown_box'),
    ('BOX_A:SHELF_09', 'unknown_destination'),
    ('BOX_B:SHELF_01', 'destination_mismatch'),
])
def test_decoded_but_invalid_payloads_are_rejected(payload, status):
    rot, t = forklift_view(2.2)
    found = QrDetector().detect(scene.render([(payload, rot, t)], K))
    assert [c.payload for c in found] == [payload]       # the decode is successful...
    result = MISSION.validate_payload(found[0].payload)
    assert not result.valid and result.status == status  # ...but the validation rejects it


# ---------------------------------------------------------------- pose estimation

@pytest.mark.parametrize('distance', [1.2, 1.8, 2.4, 3.0])
@pytest.mark.parametrize('yaw_deg', [-25.0, 0.0, 25.0])
def test_pose_accuracy_over_operating_range(distance, yaw_deg):
    rot, t = forklift_view(distance, yaw_deg=yaw_deg, lateral=0.15)
    found = QrDetector().detect(scene.render([('BOX_B:SHELF_03', rot, t)], K))
    assert len(found) == 1
    pose = estimate_qr_pose(found[0].corners, EDGE, K)
    assert pose is not None
    pos_err = np.linalg.norm(pose.tvec - t)
    assert pos_err < 0.01 + 0.01 * distance, f'position error {pos_err * 1000:.1f} mm'
    assert rotation_error_deg(rot, pose.rotation) < 3.0
    assert pose.reprojection_error_px < 1.0


def test_pose_robust_to_noise_and_blur():
    rot, t = forklift_view(2.4, yaw_deg=10.0)
    img = scene.render([('BOX_A:SHELF_01', rot, t)], K, noise_sigma=6.0, blur_sigma=1.0)
    found = QrDetector().detect(img)
    assert [c.payload for c in found] == ['BOX_A:SHELF_01']
    pose = estimate_qr_pose(found[0].corners, EDGE, K)
    assert np.linalg.norm(pose.tvec - t) < 0.05
    assert rotation_error_deg(rot, pose.rotation) < 5.0


def test_pose_from_exact_projection_is_exact():
    rot, t = forklift_view(2.0, yaw_deg=20.0)
    corners = (K @ ((rot @ object_points(EDGE).T).T + t).T).T
    corners = corners[:, :2] / corners[:, 2:]
    pose = estimate_qr_pose(corners, EDGE, K)
    np.testing.assert_allclose(pose.tvec, t, atol=1e-6)
    assert rotation_error_deg(rot, pose.rotation) < 1e-4
    assert pose.viewing_angle_deg == pytest.approx(
        math.degrees(math.acos(rot[:, 2] @ (-t / np.linalg.norm(t)))), abs=1e-3)


def test_distance_scales_with_assumed_edge():
    """If the printed size is incorrect, the estimated distance changes by the same ratio."""
    rot, t = forklift_view(2.0)
    corners = (K @ ((rot @ object_points(EDGE).T).T + t).T).T
    corners = corners[:, :2] / corners[:, 2:]
    assert estimate_qr_pose(corners, 2 * EDGE, K).distance == pytest.approx(4.0, rel=1e-6)


def test_degenerate_corners_give_no_pose():
    assert estimate_qr_pose(np.zeros((4, 2)), EDGE, K) is None


def test_quaternion_conversion_matches_rotation():
    for rvec in ([0.1, -0.4, 0.2], [3.0, 0.1, 0.0], [0.0, 0.0, -3.1]):
        rot, _ = cv2.Rodrigues(np.array(rvec))
        x, y, z, w = rotation_to_quaternion(rot)
        back = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        np.testing.assert_allclose(back, rot, atol=1e-9)


def test_confidence_behaviour():
    assert confidence(80, 0.2, decoded=False) == 0.0
    assert confidence(80, 0.2, True) > confidence(30, 0.2, True)
    assert confidence(80, 0.2, True) > confidence(80, 3.0, True)
    assert 0.0 <= confidence(500, 0.0, True) <= 1.0
