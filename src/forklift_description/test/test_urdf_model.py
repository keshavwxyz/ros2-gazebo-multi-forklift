"""
Static validation of the forklift URDF that xacro makes.

The tests examine: generation for each backend, check_urdf, tree structure, joint
definitions and limits, inertial validity, collision coverage, ros2_control interfaces,
frame positions (forward kinematics), static stability and parametric overrides.
"""

import math
import pathlib
import shutil
import subprocess
import xml.etree.ElementTree as ET

from forklift_description.urdf_kinematics import generate_urdf, Model, vec
import numpy as np
import pytest
import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]
XACRO = PKG / 'urdf' / 'forklift.urdf.xacro'
PARAMS_FILE = PKG / 'config' / 'forklift_params.yaml'
P = yaml.safe_load(PARAMS_FILE.read_text())

EXPECTED_JOINTS = {
    'base_footprint_joint': 'fixed',
    'counterweight_joint': 'fixed',
    'overhead_guard_joint': 'fixed',
    'left_drive_wheel_joint': 'continuous',
    'right_drive_wheel_joint': 'continuous',
    'left_caster_joint': 'fixed',
    'right_caster_joint': 'fixed',
    'mast_joint': 'fixed',
    'fork_lift_joint': 'prismatic',
    'left_fork_joint': 'fixed',
    'right_fork_joint': 'fixed',
    'fork_tip_joint': 'fixed',
    'lidar_joint': 'fixed',
    'rear_lidar_joint': 'fixed',
    'camera_joint': 'fixed',
    'camera_optical_joint': 'fixed',
    'imu_joint': 'fixed',
}
# Reference frames only: no geometry and no mass.
FRAME_ONLY_LINKS = {'base_footprint', 'fork_tip_frame', 'camera_optical_frame'}
EXPECTED_LINKS = FRAME_ONLY_LINKS | {
    'base_link', 'counterweight_link', 'overhead_guard_link',
    'left_drive_wheel_link', 'right_drive_wheel_link', 'left_caster_link', 'right_caster_link',
    'mast_link', 'fork_carriage_link', 'left_fork_link', 'right_fork_link',
    'lidar_link', 'rear_lidar_link', 'camera_link', 'imu_link',
}
TOL = 1e-6


# --------------------------------------------------------------------------- helpers

def generate(**mappings):
    return generate_urdf(XACRO, **mappings)


def parse(urdf_xml):
    return ET.fromstring(urdf_xml)


@pytest.fixture(scope='module')
def urdf_xml():
    return generate()


@pytest.fixture(scope='module')
def model(urdf_xml):
    return Model(urdf_xml)


# --------------------------------------------------------------------------- generation

@pytest.mark.parametrize('backend,plugin', [
    ('mock', 'mock_components/GenericSystem'),
    ('gazebo', 'gz_ros2_control/GazeboSimSystem'),
])
def test_backend_selects_plugin(backend, plugin):
    root = parse(generate(sim_backend=backend))
    assert root.find('ros2_control/hardware/plugin').text == plugin


def test_invalid_backend_is_rejected():
    with pytest.raises(Exception, match='sim_backend must be one of'):
        generate(sim_backend='not_a_simulator')


def test_ros2_control_can_be_disabled():
    assert parse(generate(ros2_control='false')).find('ros2_control') is None


def test_check_urdf_accepts_model(urdf_xml, tmp_path):
    exe = shutil.which('check_urdf')
    if exe is None:
        pytest.skip('check_urdf (ros-jazzy-urdfdom) not installed')
    f = tmp_path / 'forklift.urdf'
    f.write_text(urdf_xml)
    result = subprocess.run([exe, str(f)], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'root Link: base_footprint' in result.stdout


def test_gazebo_sdf_conversion(tmp_path):
    """The URDF must convert to SDF (ros_gz_sim spawns SDF) without errors."""
    exe = shutil.which('gz')
    if exe is None:
        pytest.skip('gz CLI not available')
    f = tmp_path / 'forklift.urdf'
    f.write_text(generate(sim_backend='gazebo'))
    result = subprocess.run([exe, 'sdf', '-p', str(f)], capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stderr
    sdf = ET.fromstring(result.stdout)
    sdf_joints = {j.get('name'): j.get('type') for j in sdf.iter('joint')}
    # The movable joints must stay after the fixed joints are merged.
    assert sdf_joints.get('fork_lift_joint') == 'prismatic'
    assert sdf_joints.get('left_drive_wheel_joint') == 'revolute'
    assert sdf_joints.get('right_drive_wheel_joint') == 'revolute'


# --------------------------------------------------------------------------- structure

def test_links_and_joints_match_specification(model):
    assert set(model.links) == EXPECTED_LINKS
    assert {n: j.get('type') for n, j in model.joints.items()} == EXPECTED_JOINTS


def test_single_connected_tree_rooted_at_base_footprint(model):
    children = [j.find('child').get('link') for j in model.joints.values()]
    assert len(children) == len(set(children)), 'a link has more than one parent'
    roots = set(model.links) - set(children)
    assert roots == {'base_footprint'}
    for j in model.joints.values():
        assert j.find('parent').get('link') in model.links
        assert j.find('child').get('link') in model.links


def test_every_physical_link_has_visual_collision_and_inertial(model):
    for name, link in model.links.items():
        if name in FRAME_ONLY_LINKS:
            assert link.find('visual') is None and link.find('inertial') is None, name
            continue
        assert link.find('visual') is not None, f'{name}: no visual'
        assert link.find('collision') is not None, f'{name}: no collision'
        assert link.find('inertial') is not None, f'{name}: no inertial'


def test_collision_uses_primitives_only(model):
    for name, link in model.links.items():
        for geom in link.findall('collision/geometry'):
            kinds = {child.tag for child in geom}
            assert kinds <= {'box', 'cylinder', 'sphere'}, f'{name}: {kinds}'


# --------------------------------------------------------------------------- inertia

def inertia_tensor(inertial):
    i = inertial.find('inertia')
    g = {k: float(i.get(k)) for k in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')}
    return np.array([
        [g['ixx'], g['ixy'], g['ixz']],
        [g['ixy'], g['iyy'], g['iyz']],
        [g['ixz'], g['iyz'], g['izz']],
    ])


def test_inertias_are_physically_valid(model):
    for name, link in model.links.items():
        inertial = link.find('inertial')
        if inertial is None:
            continue
        m = float(inertial.find('mass').get('value'))
        assert m > 0, name
        eig = np.linalg.eigvalsh(inertia_tensor(inertial))
        assert np.all(eig > 0), f'{name}: not positive definite {eig}'
        a, b, c = eig
        # The principal moments of a rigid body obey the triangle inequality.
        assert a + b >= c - TOL and a + c >= b - TOL and b + c >= a - TOL, f'{name}: {eig}'


def test_total_mass_matches_parameters(model):
    expected = (P['chassis']['mass'] + P['counterweight']['mass'] + P['overhead_guard']['mass']
                + 2 * P['drive_wheel']['mass'] + 2 * P['caster']['mass'] + P['mast']['mass']
                + P['carriage']['mass'] + 2 * P['forks']['mass'] + 0.2 + 0.2 + 0.1 + 0.05)
    total, _ = model.mass_and_com()
    assert total == pytest.approx(expected)


def test_fork_composite_com_lies_inside_the_l_shape(model):
    fk = P['forks']
    com = vec(model.links['left_fork_link'].find('inertial/origin').get('xyz'))
    assert -fk['shank_thickness'] < com[0] < fk['length']
    assert -fk['thickness'] < com[2] < fk['shank_height']


# --------------------------------------------------------------------------- joints

def test_lift_joint_limits(model):
    j = model.joints['fork_lift_joint']
    lim = j.find('limit')
    assert vec(j.find('axis').get('xyz')).tolist() == [0, 0, 1]
    assert float(lim.get('lower')) == 0.0
    assert float(lim.get('upper')) == pytest.approx(P['lift']['max_travel'])
    assert float(lim.get('velocity')) == pytest.approx(P['lift']['max_velocity'])
    assert float(lim.get('effort')) == pytest.approx(P['lift']['max_effort'])
    assert float(j.find('dynamics').get('damping')) > 0


def test_lift_actuator_can_hold_carriage_forks_and_rated_load(model):
    moving = P['carriage']['mass'] + 2 * P['forks']['mass']
    required = (moving + P['lift']['rated_load']) * 9.81
    assert P['lift']['max_effort'] > 1.5 * required


@pytest.mark.parametrize('side', ['left', 'right'])
def test_drive_wheel_joints(model, side):
    j = model.joints[f'{side}_drive_wheel_joint']
    assert vec(j.find('axis').get('xyz')).tolist() == [0, 1, 0]
    lim = j.find('limit')
    assert float(lim.get('velocity')) == pytest.approx(P['drive_wheel']['max_velocity'])
    assert float(lim.get('effort')) > 0


# --------------------------------------------------------------------------- ros2_control

def test_ros2_control_interfaces(model):
    rc = model.root.find('ros2_control')
    joints = {j.get('name'): j for j in rc.findall('joint')}
    assert set(joints) == {'left_drive_wheel_joint', 'right_drive_wheel_joint', 'fork_lift_joint'}
    for name in joints:
        assert name in model.joints, f'{name} not in URDF'

    def ifaces(j, kind):
        return {i.get('name') for i in j.findall(kind)}

    for side in ('left', 'right'):
        j = joints[f'{side}_drive_wheel_joint']
        assert ifaces(j, 'command_interface') == {'velocity'}
        assert ifaces(j, 'state_interface') == {'position', 'velocity'}
    lift = joints['fork_lift_joint']
    # velocity commands (equivalent to a hydraulic valve), see docs/system_design.md D4
    assert ifaces(lift, 'command_interface') == {'velocity'}
    assert ifaces(lift, 'state_interface') == {'position', 'velocity', 'effort'}


def test_ros2_control_command_limits_match_urdf_limits(model):
    rc = model.root.find('ros2_control')
    for j in rc.findall('joint'):
        urdf_limit = model.joints[j.get('name')].find('limit')
        params = {p.get('name'): float(p.text)
                  for p in j.find('command_interface').findall('param')}
        # All commands are velocities. The URDF joint velocity limit is their maximum.
        assert params['max'] == float(urdf_limit.get('velocity'))
        assert params['min'] == -float(urdf_limit.get('velocity'))


# --------------------------------------------------------------------------- geometry

def test_robot_rests_on_its_wheels(model):
    for side in ('left', 'right'):
        centre = model.point(f'{side}_drive_wheel_link', np.zeros(3))
        assert centre[2] == pytest.approx(P['drive_wheel']['radius'], abs=TOL)
        assert centre[0] == pytest.approx(0.0, abs=TOL)
        caster = model.point(f'{side}_caster_link', np.zeros(3))
        assert caster[2] - P['caster']['radius'] == pytest.approx(0.0, abs=TOL)
    left = model.point('left_drive_wheel_link', np.zeros(3))
    right = model.point('right_drive_wheel_link', np.zeros(3))
    assert left[1] == pytest.approx(-right[1]) and left[1] > 0


def test_chassis_clears_the_ground(model):
    assert P['chassis']['ground_clearance'] > 0
    centre = vec(model.links['base_link'].find('collision/origin').get('xyz'))
    chassis_bottom = model.point('base_link', centre)[2] - P['chassis']['height'] / 2
    assert chassis_bottom == pytest.approx(P['chassis']['ground_clearance'], abs=TOL)


@pytest.mark.parametrize('q', [0.0, 0.3, P['lift']['max_travel']])
def test_fork_top_height_tracks_lift_joint(model, q):
    tip = model.point('fork_tip_frame', np.zeros(3), {'fork_lift_joint': q})
    assert tip[2] == pytest.approx(P['lift']['min_height'] + q, abs=TOL)
    # the fork bottom must not go into the floor at the lowest position
    assert P['lift']['min_height'] - P['forks']['thickness'] > 0


def test_fork_tip_reach_and_symmetry(model):
    tip = model.point('fork_tip_frame', np.zeros(3))
    expected_x = P['mast']['x'] + P['carriage']['x_offset'] + P['forks']['length']
    assert tip[0] == pytest.approx(expected_x, abs=TOL)
    assert tip[1] == pytest.approx(0.0, abs=TOL)
    left = model.point('left_fork_link', np.zeros(3))
    right = model.point('right_fork_link', np.zeros(3))
    assert left[1] - right[1] == pytest.approx(P['forks']['spacing'])


def test_forks_are_in_front_of_mast_and_chassis(model):
    heel = model.point('fork_carriage_link', np.zeros(3))
    mast_front = P['mast']['x'] + P['mast']['upright_size_x'] / 2
    carriage_back = heel[0] - P['forks']['shank_thickness'] - P['carriage']['bar_thickness']
    assert carriage_back >= mast_front - TOL, 'carriage bars intersect the mast uprights'
    assert mast_front - P['mast']['upright_size_x'] > P['chassis']['front_x'] - TOL


def test_lidar_scan_plane_is_unobstructed(model):
    """
    Make sure that no part of the robot crosses the 2D scan plane.

    The plane must be above the lowered tines and below the carriage bars and the mast. It
    must be outside the chassis body (see docs/system_design.md D6).
    """
    lidar = model.point('lidar_link', np.zeros(3))
    z = lidar[2]
    assert z == pytest.approx(P['sensors']['lidar']['z'], abs=TOL)
    assert lidar[0] - P['sensors']['lidar']['radius'] >= P['chassis']['front_x'] - TOL
    assert z > P['lift']['min_height'], 'lowered fork tines cross the scan plane'
    lower_bar_bottom = (P['lift']['min_height'] + P['carriage']['lower_bar_z']
                        - P['carriage']['bar_height'] / 2)
    assert z < lower_bar_bottom, 'carriage bar crosses the scan plane at lowest lift'
    assert z < P['mast']['bottom_z'], 'mast uprights cross the scan plane'


def test_rear_lidar_faces_backwards_with_an_unobstructed_scan(model):
    """
    Make sure that the rear LiDAR points rearward, is behind the body and sees no robot part.

    Each ray of the 180 deg scan, in the scan plane, must go out of the robot. It must not cross
    the chassis or the counterweight footprint (x from rear to front_x, |y| <= width / 2).
    """
    rl = P['sensors']['rear_lidar']
    tf = model.tf('rear_lidar_link')
    origin, forward = tf[:3, 3], tf[:3, 0]
    assert forward == pytest.approx([-1.0, 0.0, 0.0], abs=1e-9), 'rear LiDAR must face -x'
    assert origin[2] == pytest.approx(rl['z'], abs=TOL)
    rear_face = P['chassis']['front_x'] - P['chassis']['length']
    assert origin[0] + rl['radius'] <= rear_face + TOL, 'sensor body inside the chassis'
    assert origin[0] + rl['radius'] >= rear_face - 0.01, 'sensor not mounted on the rear face'
    assert P['chassis']['ground_clearance'] < origin[2] < (
        P['chassis']['ground_clearance'] + P['chassis']['height']), 'not on the rear face'
    half_w = max(P['chassis']['width'], P['counterweight']['width']) / 2
    for a in np.linspace(rl['min_angle'], rl['max_angle'], rl['samples']):
        d = tf[:3, :3] @ np.array([math.cos(a), math.sin(a), 0.0])
        for r in np.linspace(rl['range_min'], 3.0, 300):
            px, py = origin[0] + r * d[0], origin[1] + r * d[1]
            assert not (rear_face <= px <= P['chassis']['front_x'] and abs(py) <= half_w), (
                f'ray at {math.degrees(a):.0f} deg hits the body at {r:.2f} m')
    lidar = P['sensors']['lidar']
    assert (rl['update_rate'], rl['samples']) <= (lidar['update_rate'], lidar['samples']), (
        'rear LiDAR should be no heavier than the front one')


def test_camera_optical_frame_convention_and_view(model):
    tf = model.tf('camera_optical_frame')
    optical_z = tf[:3, 2]   # direction of view
    optical_x = tf[:3, 0]   # right of the image
    optical_y = tf[:3, 1]   # down in the image
    pitch = P['sensors']['camera']['pitch']
    np.testing.assert_allclose(optical_z, [math.cos(pitch), 0, -math.sin(pitch)], atol=TOL)
    np.testing.assert_allclose(optical_x, [0, -1, 0], atol=TOL)
    assert optical_y[2] < 0, 'image down must point towards the ground'
    cam = tf[:3, 3]
    assert cam[2] < P['overhead_guard']['roof_height'], 'camera inside the roof'
    assert cam[0] <= P['overhead_guard']['front_x'] + TOL


def test_camera_view_clears_raised_carriage(model):
    """At full lift, the carriage stays below the top image ray of the camera (16:9, 70° HFOV)."""
    top_ray_depression = P['sensors']['camera']['pitch'] - camera_vertical_fov() / 2
    cam = model.point('camera_link', np.zeros(3))
    carriage_x = model.point('fork_carriage_link', np.zeros(3))[0]
    carriage_top = (P['lift']['min_height'] + P['lift']['max_travel']
                    + P['carriage']['upper_bar_z'] + P['carriage']['bar_height'] / 2)
    ray_z_at_carriage = cam[2] - (carriage_x - cam[0]) * math.tan(top_ray_depression)
    assert top_ray_depression > 0, 'top image ray must point below the horizon'
    # The carriage can be in the lower part of the image. It must not cover the top rows.
    assert carriage_top < ray_z_at_carriage


def camera_vertical_fov():
    hfov = math.radians(70)
    return 2 * math.atan(math.tan(hfov / 2) * 9 / 16)


@pytest.mark.parametrize('target', [
    ('fork tips', 'tip'),
    ('QR on a pallet face at the fork tips', 1.0),
    ('QR at a 2.5 m staging distance', 2.5),
])
def test_camera_sees_alignment_targets(model, target):
    """The fork tips and a QR code (center 0.40 m above the ground) must be in the vertical FOV."""
    _, where = target
    cam = model.point('camera_link', np.zeros(3))
    tip = model.point('fork_tip_frame', np.zeros(3))
    if where == 'tip':
        point = tip
    else:
        point = np.array([tip[0] if where == 1.0 else cam[0] + where, 0.0, 0.40])
    depression = math.atan2(cam[2] - point[2], point[0] - cam[0])
    pitch = P['sensors']['camera']['pitch']
    half = camera_vertical_fov() / 2
    margin = math.radians(2)
    assert pitch - half + margin < depression < pitch + half - margin, \
        f'{target[0]} at {math.degrees(depression):.1f} deg below horizon is out of view'


def test_imu_is_inside_the_chassis(model):
    imu = model.point('imu_link', np.zeros(3))
    ch = P['chassis']
    assert ch['front_x'] - ch['length'] < imu[0] < ch['front_x']
    assert ch['ground_clearance'] < imu[2] < ch['ground_clearance'] + ch['height']
    assert imu[1] == pytest.approx(0.0, abs=TOL)


# --------------------------------------------------------------------------- stability

def support_polygon_x_range():
    return P['caster']['x'], 0.0   # contacts of the casters .. contacts of the drive axle


@pytest.mark.parametrize('q,load_kg', [
    (0.0, 0.0), (P['lift']['max_travel'], 0.0), (0.20, 50.0), (P['lift']['max_travel'], 50.0)])
def test_centre_of_mass_inside_support_polygon(model, q, load_kg):
    total, com = model.mass_and_com({'fork_lift_joint': q})
    if load_kg:
        # Box at the center of the forks, 0.3 m above the fork top surface.
        load_pos = model.point('fork_carriage_link', np.array([P['forks']['length'] / 2, 0, 0.3]),
                               {'fork_lift_joint': q})
        com = (total * com + load_kg * load_pos) / (total + load_kg)
    x_min, x_max = support_polygon_x_range()
    margin = 0.15
    assert x_min + margin < com[0] < x_max - margin, f'COM x={com[0]:.3f}'
    assert abs(com[1]) < 1e-3


@pytest.mark.parametrize('q', [0.20, P['lift']['max_travel']])
def test_rated_load_does_not_tip_forward_when_braking(model, q):
    """
    Make sure that the robot does not tip forward at rated load during braking.

    The inertial force of the braking moves the COM (dx = h * a / g). The COM must stay
    behind the tipping line of the front axle.
    """
    braking_decel = 0.5  # m/s^2, maximum value for the navigation limits of Phase 5
    load = P['lift']['rated_load']
    total, com = model.mass_and_com({'fork_lift_joint': q})
    load_pos = model.point('fork_carriage_link', np.array([P['forks']['length'] / 2, 0, 0.3]),
                           {'fork_lift_joint': q})
    com = (total * com + load * load_pos) / (total + load)
    shifted_x = com[0] + com[2] * braking_decel / 9.81
    assert shifted_x < -0.03, f'tipping margin only {-shifted_x:.3f} m'


# --------------------------------------------------------------------------- parametric

def test_parameter_file_overrides_geometry(tmp_path):
    params = yaml.safe_load(PARAMS_FILE.read_text())
    params['forks']['length'] = 1.20
    params['forks']['spacing'] = 0.50
    params['lift']['max_travel'] = 1.50
    params['drive_wheel']['separation'] = 0.90
    f = tmp_path / 'custom.yaml'
    f.write_text(yaml.safe_dump(params))
    m = Model(generate(params_file=f))
    assert float(m.joints['fork_lift_joint'].find('limit').get('upper')) == pytest.approx(1.5)
    tip = m.point('fork_tip_frame', np.zeros(3))
    assert tip[0] == pytest.approx(params['mast']['x'] + params['carriage']['x_offset'] + 1.20)
    assert m.point('left_fork_link', np.zeros(3))[1] == pytest.approx(0.25)
    assert m.point('left_drive_wheel_link', np.zeros(3))[1] == pytest.approx(0.45)
    lift_limit = m.joints['fork_lift_joint'].find('limit')
    assert float(lift_limit.get('upper')) == pytest.approx(1.5)
