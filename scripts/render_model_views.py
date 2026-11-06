#!/usr/bin/env python3
"""
Make orthographic views (side, top and front) of the forklift collision geometry.

The script draws each collision primitive (the convex hull of its projected outline) at the
lowest and the highest fork position. It also draws the LiDAR scan plane and the vertical
field of view of the camera. Use these views to examine the geometry and the sensor
clearances without a GUI or a simulator.

Usage (source the workspace first):
  scripts/render_model_views.py [-o docs/images/forklift_views.png] [--params FILE]
"""

import argparse
import math
import pathlib

from forklift_description.urdf_kinematics import DEFAULT_XACRO, generate_urdf, Model
import matplotlib
import numpy as np
from scipy.spatial import ConvexHull
import yaml

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402, I100 - must follow matplotlib.use()

COLORS = {
    'base_link': '#f28c0f', 'counterweight_link': '#333333', 'overhead_guard_link': '#555555',
    'mast_link': '#444444', 'fork_carriage_link': '#8c8c94', 'left_fork_link': '#8c8c94',
    'right_fork_link': '#8c8c94', 'lidar_link': '#1f77b4', 'camera_link': '#d62728',
    'imu_link': '#2ca02c',
}
VIEWS = [('Side view (x-z)', 0, 2), ('Top view (x-y)', 0, 1), ('Front view (y-z)', 1, 2)]


def draw(ax, model, q, ia, ib, alpha):
    for link, pts in model.collision_points(q):
        p2 = pts[:, [ia, ib]]
        try:
            hull = ConvexHull(p2)
            poly = p2[hull.vertices]
        # the outline has no area (for example, a cylinder that you see from its end)
        except Exception:
            continue
        ax.fill(poly[:, 0], poly[:, 1], color=COLORS.get(link, '#111111'),
                alpha=alpha, linewidth=0.6, edgecolor='black')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('-o', '--output', default='docs/images/forklift_views.png')
    parser.add_argument('--params', default='', help='geometry YAML (default: package config)')
    args = parser.parse_args()

    params_path = pathlib.Path(args.params or DEFAULT_XACRO.parents[1] / 'config'
                               / 'forklift_params.yaml')
    p = yaml.safe_load(params_path.read_text())
    model = Model(generate_urdf(params_file=args.params) if args.params else generate_urdf())
    q_max = p['lift']['max_travel']

    fig, axes = plt.subplots(1, 3, figsize=(18, 6.5),
                             gridspec_kw={'width_ratios': [1.6, 1.6, 1.0]})
    for ax, (title, ia, ib) in zip(axes, VIEWS):
        draw(ax, model, {'fork_lift_joint': q_max}, ia, ib, alpha=0.25)
        draw(ax, model, {'fork_lift_joint': 0.0}, ia, ib, alpha=0.9)
        ax.set_title(title)
        ax.set_aspect('equal')
        ax.grid(True, linewidth=0.3)
        ax.set_xlabel('xyz'[ia] + ' [m]')
        ax.set_ylabel('xyz'[ib] + ' [m]')
        if ib == 2:
            ax.axhline(0, color='saddlebrown', linewidth=1.5)

    side = axes[0]
    lidar = model.point('lidar_link', np.zeros(3))
    side.plot([lidar[0], lidar[0] + 3.0], [lidar[2]] * 2, 'b--', linewidth=1,
              label=f'LiDAR scan plane z={lidar[2]:.2f} m')
    cam = model.point('camera_link', np.zeros(3))
    hfov = math.radians(70)
    vfov = 2 * math.atan(math.tan(hfov / 2) * 9 / 16)
    pitch = p['sensors']['camera']['pitch']
    for ang in (pitch - vfov / 2, pitch + vfov / 2):
        reach = cam[2] / math.tan(ang)
        side.plot([cam[0], cam[0] + reach], [cam[2], 0], 'r:', linewidth=1)
    side.plot([], [], 'r:', label='camera vertical FOV (70° HFOV, 16:9)')
    total, com = model.mass_and_com()
    side.plot(com[0], com[2], 'k*', markersize=12, label=f'COM ({total:.0f} kg, unloaded)')
    side.set_xlim(-1.4, 4.2)
    side.set_ylim(-0.1, 2.4)
    side.legend(loc='upper right', fontsize=8)

    fig.suptitle(f'Forklift collision geometry: solid = forks lowered, '
                 f'faded = forks raised {q_max:.2f} m')
    out = pathlib.Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(out)


if __name__ == '__main__':
    main()
