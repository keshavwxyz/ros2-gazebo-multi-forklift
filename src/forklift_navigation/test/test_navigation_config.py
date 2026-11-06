"""
Static checks of the navigation configuration, the footprints and the saved map (no ROS graph).

The footprints must cover the real robot (with the forks) and a carried pallet. The Nav2
parameters must agree with the robot, with the command chain and with each other. The
saved map must align with the world/mission frame. The robot must be able to get to the
staging poses and turn there.
"""

import math
import pathlib

import cv2
from forklift_description.urdf_kinematics import generate_urdf, Model
from forklift_navigation.footprint import (circumscribed_radius, contains, edge_distance,
                                           load_footprints, parse_nav2_string, self_hit_mask)
from forklift_task_manager.mission_config import load_mission
import numpy as np
import pytest
import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]
SRC = PKG.parent
FOOTPRINTS = load_footprints(PKG / 'config' / 'footprints.yaml')
NAV2 = yaml.safe_load((PKG / 'config' / 'nav2_params.yaml').read_text())
SLAM = yaml.safe_load((PKG / 'config' / 'slam_toolbox_mapping.yaml').read_text())
MAP_META = yaml.safe_load((PKG / 'maps' / 'warehouse.yaml').read_text())
MAP_IMG = cv2.imread(str(PKG / 'maps' / MAP_META['image']), cv2.IMREAD_GRAYSCALE)
MISSION = load_mission(SRC / 'forklift_task_manager' / 'config' / 'warehouse_mission.yaml')
ROBOT = Model(generate_urdf(SRC / 'forklift_description' / 'urdf' / 'forklift.urdf.xacro'))


def ros(node):
    return NAV2[node]['ros__parameters']


def costmap(name):
    return NAV2[name][name]['ros__parameters']


# ---------------------------------------------------------------- footprints

@pytest.mark.parametrize('q', [0.0, 1.0])
def test_unloaded_footprint_covers_the_robot(q):
    """Each collision primitive (chassis, wheels, mast, forks) is 2 cm or more inside."""
    poly = FOOTPRINTS['unloaded']
    for link, pts in ROBOT.collision_points({'fork_lift_joint': q}):
        xy = pts[:, :2]
        assert contains(poly, xy).all(), f'{link} sticks out of the footprint'
        assert edge_distance(poly, xy).min() >= 0.02 - 1e-9, f'{link} too close to the edge'


def test_loaded_footprint_covers_a_carried_pallet():
    load = MISSION.load
    heel_x = ROBOT.point('fork_carriage_link', np.zeros(3))[0]
    x0, x1 = heel_x, heel_x + load.pallet_depth
    h = load.pallet_width / 2
    corners = np.array([[x0, h], [x1, h], [x1, -h], [x0, -h]])
    edges = np.concatenate([np.linspace(corners[i], corners[(i + 1) % 4], 20)
                            for i in range(4)])
    assert contains(FOOTPRINTS['loaded'], edges).all()
    assert edge_distance(FOOTPRINTS['loaded'], edges).min() >= 0.02
    assert not contains(FOOTPRINTS['unloaded'], edges).all(), 'unloaded must be narrower'


def test_loaded_footprint_contains_unloaded():
    assert contains(FOOTPRINTS['loaded'], FOOTPRINTS['unloaded'] * 0.999).all()


def test_local_costmap_starts_with_unloaded_footprint():
    np.testing.assert_allclose(parse_nav2_string(costmap('local_costmap')['footprint']),
                               FOOTPRINTS['unloaded'])


def test_global_planner_radius_covers_chassis_half_width():
    half_width = max(abs(FOOTPRINTS['unloaded'][:, 1]))
    assert costmap('global_costmap')['robot_radius'] >= half_width + 0.05


def test_fork_tip_offset_matches_robot_geometry():
    tip = ROBOT.point('fork_tip_frame', np.zeros(3))
    assert MISSION.fork_tip_offset == pytest.approx(tip[0], abs=1e-6)


# ---------------------------------------------------------------- scan self-filter

def test_self_filter_masks_fork_shanks_but_not_the_world():
    lidar = ROBOT.point('lidar_link', np.zeros(3))
    shank = ROBOT.point('left_fork_link', np.array([-0.02, 0.0, 0.1]))   # center of the shank
    rel = shank[:2] - lidar[:2]
    angles = np.array([math.atan2(rel[1], rel[0]), 0.0, math.radians(-90)])
    ranges = np.array([np.linalg.norm(rel), 4.1, 6.0])
    mask = self_hit_mask(ranges, angles, (lidar[0], lidar[1], 0.0),
                         FOOTPRINTS['unloaded'], 0.05)
    assert mask.tolist() == [True, False, False]


def test_self_filter_ignores_invalid_ranges():
    mask = self_hit_mask(np.array([np.inf, np.nan]), np.array([0.0, 1.0]), (0.25, 0, 0),
                         FOOTPRINTS['unloaded'], 0.05)
    assert not mask.any()


# ---------------------------------------------------------------- Nav2 parameters

def test_velocity_publishers_are_stamped():
    for node in ('controller_server', 'behavior_server', 'velocity_smoother',
                 'collision_monitor'):
        assert ros(node)['enable_stamped_cmd_vel'] is True, node


def test_command_chain_ends_at_twist_mux_nav_input():
    mux = yaml.safe_load((SRC / 'forklift_control' / 'config' / 'twist_mux.yaml').read_text())
    nav_topic = mux['twist_mux']['ros__parameters']['topics']['navigation']['topic']
    assert ros('collision_monitor')['cmd_vel_out_topic'] == nav_topic
    assert ros('collision_monitor')['cmd_vel_in_topic'] == 'cmd_vel_smoothed'


def test_all_scan_consumers_use_the_filtered_scan():
    assert ros('amcl')['scan_topic'] == '/scan_filtered'
    assert SLAM['slam_toolbox']['ros__parameters']['scan_topic'] == '/scan_filtered'
    assert ros('collision_monitor')['scan']['topic'] == '/scan_filtered'
    for name in ('local_costmap', 'global_costmap'):
        assert costmap(name)['obstacle_layer']['scan']['topic'] == '/scan_filtered'


def test_speed_limits_respect_tip_over_and_reverse_limits():
    vs = ros('velocity_smoother')
    assert abs(vs['max_decel'][0]) <= 0.5          # limit from the tip-over analysis
    assert vs['max_velocity'][0] <= 0.5
    # the rear is not visible: only slow reverse motion
    assert vs['min_velocity'][0] >= -0.2
    assert ros('controller_server')['FollowPath']['allow_reversing'] is False


def test_frames_and_odometry_are_consistent():
    assert ros('amcl')['base_frame_id'] == 'base_footprint'
    assert ros('bt_navigator')['robot_base_frame'] == 'base_footprint'
    assert ros('bt_navigator')['odom_topic'] == '/odometry/filtered'
    assert ros('controller_server')['odom_topic'] == '/odometry/filtered'
    assert costmap('local_costmap')['global_frame'] == 'odom'
    assert costmap('global_costmap')['global_frame'] == 'map'
    slam = SLAM['slam_toolbox']['ros__parameters']
    assert (slam['base_frame'], slam['odom_frame'], slam['map_frame']) == (
        'base_footprint', 'odom', 'map')


# ---------------------------------------------------------------- saved map

def world_to_cell(x, y):
    res, (ox, oy, _) = MAP_META['resolution'], MAP_META['origin']
    col = int((x - ox) / res)
    row = MAP_IMG.shape[0] - 1 - int((y - oy) / res)
    return row, col


def occupied_near(x, y, radius=0.1):
    r, c = world_to_cell(x, y)
    k = int(math.ceil(radius / MAP_META['resolution']))
    return (MAP_IMG[max(r - k, 0):r + k + 1, max(c - k, 0):c + k + 1] < 50).any()


@pytest.mark.parametrize('name,x,y', [
    ('west wall', -8.0, 0.0), ('east wall', 8.0, -4.0), ('south wall', 0.0, -6.0),
    ('north wall', -6.0, 6.0), ('column west face', -1.65, 0.0), ('column east face', 1.65, 0.0),
    ('platform SHELF_02 front', 0.0, 4.8), ('east rack front', 6.8, 0.0),
])
def test_map_aligned_with_world(name, x, y):
    """The known world structure is in the map, near (10 cm) its world position."""
    assert occupied_near(x, y), f'{name} not found in the map at ({x}, {y})'


@pytest.mark.parametrize('x,y', [(-6.0, 0.0), (0.0, -2.0), (0.0, 2.0), (-3.0, 0.0),
                                 (4.0, 0.0)])
def test_aisles_are_free(x, y):
    assert not occupied_near(x, y, radius=0.3)


def test_movable_loads_are_not_in_the_map():
    for p in MISSION.pickups.values():
        assert not occupied_near(p.x, p.y, radius=0.3), 'load mapped as static structure'


def staging_poses():
    poses = [(f'pickup {b.box_id}', MISSION.pickup_staging(b.box_id)) for b in MISSION.boxes]
    poses += [(f'dest {d}', MISSION.destination_staging(d)) for d in MISSION.destinations]
    return poses


@pytest.mark.parametrize('name,pose', staging_poses())
def test_staging_pose_has_turning_room(name, pose):
    """There is no mapped obstacle in the turning circle: the robot can turn in place there."""
    radius = circumscribed_radius(FOOTPRINTS['unloaded']) + 0.1
    for a in np.linspace(0, 2 * math.pi, 72, endpoint=False):
        for r in np.arange(0.0, radius, 0.05):
            x, y = pose.x + r * math.cos(a), pose.y + r * math.sin(a)
            row, col = world_to_cell(x, y)
            assert MAP_IMG[row, col] >= 50, f'{name}: obstacle at ({x:.2f}, {y:.2f})'
