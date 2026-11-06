"""The controller configuration must agree with the parameters of the robot description."""

import pathlib

import pytest
import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]
CONTROLLERS = yaml.safe_load((PKG / 'config' / 'controllers.yaml').read_text())
MUX = yaml.safe_load((PKG / 'config' / 'twist_mux.yaml').read_text())


def description_params():
    candidates = [PKG.parent / 'forklift_description' / 'config' / 'forklift_params.yaml']
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(pathlib.Path(get_package_share_directory('forklift_description'))
                          / 'config' / 'forklift_params.yaml')
    except Exception:  # noqa: BLE001 - package index not available
        pass
    for c in candidates:
        if c.exists():
            return yaml.safe_load(c.read_text())
    pytest.skip('forklift_description params not found')


def test_wheel_geometry_matches_description():
    p = description_params()
    base = CONTROLLERS['forklift_base_controller']['ros__parameters']
    assert base['wheel_radius'] == pytest.approx(p['drive_wheel']['radius'])
    assert base['wheel_separation'] == pytest.approx(p['drive_wheel']['separation'])


def test_velocity_envelope_within_wheel_limits():
    p = description_params()
    base = CONTROLLERS['forklift_base_controller']['ros__parameters']
    wheel_speed_limit = p['drive_wheel']['max_velocity'] * p['drive_wheel']['radius']
    half_track = p['drive_wheel']['separation'] / 2
    assert base['linear.x.max_velocity'] <= wheel_speed_limit
    # Turn in place: the wheels must be able to get the rim speed = omega * half track.
    assert base['angular.z.max_velocity'] * half_track <= wheel_speed_limit


def test_braking_matches_tip_over_assumption():
    # The tip-over test of forklift_description uses a braking deceleration of 0.5 m/s^2 or less.
    base = CONTROLLERS['forklift_base_controller']['ros__parameters']
    assert abs(base['linear.x.max_deceleration']) <= 0.5
    assert abs(base['linear.x.min_acceleration']) <= 0.5


def test_odometry_does_not_publish_tf():
    base = CONTROLLERS['forklift_base_controller']['ros__parameters']
    assert base['enable_odom_tf'] is False
    assert base['base_frame_id'] == 'base_footprint'
    assert base['odom_frame_id'] == 'odom'


def test_estop_has_highest_priority_and_no_timeout():
    mux = MUX['twist_mux']['ros__parameters']
    estop = mux['locks']['estop']
    assert estop['timeout'] == 0.0
    assert all(estop['priority'] > t['priority'] for t in mux['topics'].values())
    assert mux['use_stamped'] is True


def test_load_handler_geometry_matches_description():
    from forklift_description.urdf_kinematics import generate_urdf, Model
    import numpy as np
    handler = yaml.safe_load((PKG / 'config' / 'load_handler.yaml').read_text())
    hp = handler['load_handler']['ros__parameters']
    p = description_params()
    model = Model(generate_urdf())
    assert hp['fork_min_height'] == pytest.approx(p['lift']['min_height'])
    assert hp['fork_thickness'] == pytest.approx(p['forks']['thickness'])
    assert hp['heel_x'] == pytest.approx(model.point('fork_carriage_link', np.zeros(3))[0])
    assert hp['lidar_x'] == pytest.approx(model.point('lidar_link', np.zeros(3))[0])
