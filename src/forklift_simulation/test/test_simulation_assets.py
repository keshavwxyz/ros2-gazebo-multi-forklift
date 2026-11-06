"""
Static checks of the Gazebo adapter assets (the simulator does not run).

The world must validate. The robot must convert to SDF with the expected sensors and
plugins. The bridge must map exactly the Gazebo topics of the sensors to the standard
ROS topics.
"""

import pathlib
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]
WORLD = PKG / 'worlds' / 'warehouse.sdf'
EXTENSION = PKG / 'urdf' / 'forklift_gazebo.xacro'
BRIDGE = yaml.safe_load((PKG / 'config' / 'gz_bridge.yaml').read_text())


def description_dir():
    src = PKG.parent / 'forklift_description'
    if src.exists():
        return src
    from ament_index_python.packages import get_package_share_directory
    return pathlib.Path(get_package_share_directory('forklift_description'))


PARAMS = yaml.safe_load((description_dir() / 'config' / 'forklift_params.yaml').read_text())
CANONICAL = {
    '/scan': 'sensor_msgs/msg/LaserScan',
    '/scan_rear': 'sensor_msgs/msg/LaserScan',
    '/camera/image_raw': 'sensor_msgs/msg/Image',
    '/camera/camera_info': 'sensor_msgs/msg/CameraInfo',
    '/imu/data': 'sensor_msgs/msg/Imu',
    '/ground_truth/odom': 'nav_msgs/msg/Odometry',
}

gz = shutil.which('gz')
needs_gz = pytest.mark.skipif(gz is None, reason='gz CLI not available')


@pytest.fixture(scope='module')
def robot_urdf():
    return xacro.process_file(
        str(description_dir() / 'urdf' / 'forklift.urdf.xacro'),
        mappings={'sim_backend': 'gazebo', 'extensions': str(EXTENSION),
                  'controllers_file': '/tmp/controllers.yaml'}).toxml()


@pytest.fixture(scope='module')
def robot_sdf(robot_urdf, tmp_path_factory):
    if gz is None:
        pytest.skip('gz CLI not available')
    f = tmp_path_factory.mktemp('sdf') / 'forklift.urdf'
    f.write_text(robot_urdf)
    out = subprocess.run([gz, 'sdf', '-p', str(f)], capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr
    return ET.fromstring(out.stdout)


@needs_gz
def test_world_is_valid_sdf():
    out = subprocess.run([gz, 'sdf', '-k', str(WORLD)], capture_output=True, text=True,
                         timeout=60)
    assert out.returncode == 0 and 'Valid' in out.stdout, out.stdout + out.stderr


def test_world_loads_required_systems():
    plugins = {p.get('name') for p in ET.parse(WORLD).getroot().iter('plugin')}
    for system in ('Physics', 'UserCommands', 'SceneBroadcaster', 'Sensors', 'Imu'):
        assert f'gz::sim::systems::{system}' in plugins


def test_world_contains_layout_features():
    models = {m.get('name') for m in ET.parse(WORLD).getroot().iter('model')}
    assert {'floor', 'walls', 'storage_platforms', 'east_rack', 'columns',
            'crate_stack'} <= models


def sensors(sdf_root):
    return {s.get('name'): s for s in sdf_root.iter('sensor')}


def test_robot_has_four_sensors_with_ros_frames(robot_sdf):
    s = sensors(robot_sdf)
    assert {n: x.get('type') for n, x in s.items()} == {
        'lidar': 'gpu_lidar', 'rear_lidar': 'gpu_lidar', 'camera': 'camera', 'imu': 'imu'}
    assert s['rear_lidar'].findtext('gz_frame_id') == 'rear_lidar_link'
    assert s['rear_lidar'].findtext('topic') == '/forklift/scan_rear'
    assert s['lidar'].findtext('gz_frame_id') == 'lidar_link'
    assert s['camera'].findtext('gz_frame_id') == 'camera_optical_frame'
    assert s['imu'].findtext('gz_frame_id') == 'imu_link'


def test_sensor_models_use_description_specs(robot_sdf):
    s = sensors(robot_sdf)
    sp = PARAMS['sensors']
    assert float(s['lidar'].findtext('update_rate')) == sp['lidar']['update_rate']
    assert int(s['lidar'].findtext('lidar/scan/horizontal/samples')) == sp['lidar']['samples']
    assert float(s['lidar'].findtext('lidar/range/max')) == sp['lidar']['range_max']
    for key in ('update_rate', 'samples', 'min_angle', 'max_angle', 'range_max'):
        path = {'update_rate': 'update_rate', 'samples': 'lidar/scan/horizontal/samples',
                'min_angle': 'lidar/scan/horizontal/min_angle',
                'max_angle': 'lidar/scan/horizontal/max_angle',
                'range_max': 'lidar/range/max'}[key]
        assert float(s['rear_lidar'].findtext(path)) == pytest.approx(sp['rear_lidar'][key])
    assert int(s['camera'].findtext('camera/image/width')) == sp['camera']['width']
    assert int(s['camera'].findtext('camera/image/height')) == sp['camera']['height']
    assert float(s['camera'].findtext('camera/horizontal_fov')) == pytest.approx(
        sp['camera']['horizontal_fov'])
    assert float(s['imu'].findtext('update_rate')) == sp['imu']['update_rate']


def test_sensors_are_noisy_not_ideal(robot_sdf):
    s = sensors(robot_sdf)
    assert float(s['lidar'].findtext('lidar/noise/stddev')) > 0
    assert float(s['camera'].findtext('camera/noise/stddev')) > 0
    assert float(s['imu'].findtext('imu/angular_velocity/z/noise/stddev')) > 0


def test_robot_plugins(robot_sdf):
    plugins = {p.get('name'): p for p in robot_sdf.iter('plugin')}
    rc = plugins['gz_ros2_control::GazeboSimROS2ControlPlugin']
    assert rc.findtext('parameters') == '/tmp/controllers.yaml'
    gt = plugins['gz::sim::systems::OdometryPublisher']
    # The ground-truth frames must stay out of the autonomy TF tree.
    assert gt.findtext('odom_frame') not in ('odom', 'map')
    assert gt.findtext('robot_base_frame') not in ('base_footprint', 'base_link')


def test_contact_friction_applied_to_lumped_collisions(robot_sdf):
    mu = {}
    for col in robot_sdf.iter('collision'):
        value = col.findtext('surface/friction/ode/mu')
        if value is not None:
            mu[col.get('name')] = float(value)
    casters = [v for k, v in mu.items() if 'caster' in k]
    forks = [v for k, v in mu.items() if 'fork_link' in k]
    wheels = [v for k, v in mu.items() if 'drive_wheel' in k]
    assert len(casters) == 2 and all(v < 0.05 for v in casters)
    assert len(wheels) == 2 and all(v >= 0.9 for v in wheels)
    assert len(forks) == 4 and all(v >= 0.5 for v in forks)


def test_bridge_maps_exactly_the_canonical_topics():
    mapped = {e['ros_topic_name']: e['ros_type_name'] for e in BRIDGE}
    assert mapped == CANONICAL
    assert all(e['direction'] == 'GZ_TO_ROS' for e in BRIDGE)


def test_bridge_gz_topics_match_sensor_topics(robot_sdf):
    s = sensors(robot_sdf)
    gz_topics = {e['gz_topic_name'] for e in BRIDGE}
    assert s['lidar'].findtext('topic') in gz_topics
    assert s['camera'].findtext('topic') in gz_topics
    assert s['camera'].findtext('camera/camera_info_topic') in gz_topics
    assert s['imu'].findtext('topic') in gz_topics
    gt = [p for p in robot_sdf.iter('plugin') if p.get('name').endswith('OdometryPublisher')][0]
    assert gt.findtext('odom_topic') in gz_topics
