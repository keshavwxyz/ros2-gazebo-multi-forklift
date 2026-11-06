#!/usr/bin/env python3
"""
Make and save the occupancy map of the warehouse (mapping procedure).

Before you start, do this step in a different terminal (do not put movable loads in the map):
  ros2 launch forklift_bringup sim.launch.py mode:=mapping spawn_loads:=false

This script moves the robot along a fixed loop through the two aisles. It uses the EKF
odometry, thus each run is the same. The script then does these steps:
  1. It lets slam_toolbox close the loop.
  2. It saves the map with map_saver_cli of nav2_map_server.
  3. It moves the saved map origin by the known start pose. Thus, the map frame aligns
     with the world/mission frame.
The SLAM map frame starts at the start pose of the robot. The start pose is the home pose
that the operator knows: (-6, 0, 0) in the world frame.

  scripts/build_map.py [-o src/forklift_navigation/maps/warehouse]
"""

import argparse
import math
import pathlib
import subprocess
import sys

from forklift_test.helpers import Driver, goto_pose_odom, spin_for, spin_until
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import yaml

START = (-6.0, 0.0, 0.0)                      # the mapping run starts from this world pose
# The loop goes through the south aisle (y = -2) and the north aisle (y = +2), in the world frame.
ROUTE = [(-6.0, -2.0), (5.0, -2.0), (5.0, 2.0), (-6.0, 2.0), (-6.0, -2.0), (-3.0, -2.0)]


def world_to_odom(x, y):
    sx, sy, syaw = START
    dx, dy = x - sx, y - sy
    return (math.cos(-syaw) * dx - math.sin(-syaw) * dy,
            math.sin(-syaw) * dx + math.cos(-syaw) * dy)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('-o', '--output', default='src/forklift_navigation/maps/warehouse')
    args = parser.parse_args()
    if START[2] != 0.0:
        sys.exit('origin alignment below assumes a start yaw of 0')

    rclpy.init()
    node = rclpy.create_node('map_builder',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
    latest = {}
    node.create_subscription(Odometry, '/odometry/filtered',
                             lambda m: latest.__setitem__('odom', m), 10)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, '/map', lambda m: latest.__setitem__('map', m),
                             latched)
    if not spin_until(node, lambda: 'odom' in latest and 'map' in latest, 60.0):
        sys.exit('no /odometry/filtered or /map: is the mapping bringup running?')
    driver = Driver(node)

    for i, (x, y) in enumerate(ROUTE):
        nx, ny = ROUTE[i + 1] if i + 1 < len(ROUTE) else (x + 1.0, y)
        yaw = math.atan2(ny - y, nx - x)              # at the end of a leg, turn to the next leg
        ox, oy = world_to_odom(x, y)
        print(f'leg {i + 1}/{len(ROUTE)}: -> world ({x:+.1f}, {y:+.1f})', flush=True)
        goto_pose_odom(node, driver, lambda: latest['odom'], (ox, oy, yaw - START[2]),
                       pos_tol=0.10, yaw_tol=0.05, timeout=180.0)
    spin_for(node, 5.0)                              # last map update and loop closure

    out = pathlib.Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(out),
                    '--ros-args', '-p', 'use_sim_time:=true', '-p',
                    'save_map_timeout:=10.0'], check=True)
    meta_path = out.with_suffix('.yaml')
    meta = yaml.safe_load(meta_path.read_text())
    meta['origin'] = [round(meta['origin'][0] + START[0], 4),
                      round(meta['origin'][1] + START[1], 4), meta['origin'][2]]
    meta_path.write_text(
        '# Built by scripts/build_map.py (slam_toolbox, mapping mode, no movable loads).\n'
        '# Origin shifted by the mapping start pose so map frame == world/mission frame.\n'
        + yaml.safe_dump(meta, sort_keys=False))
    print(f'saved {meta_path} (origin {meta["origin"]})')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
