"""Footprint polygons: load, point containment and distances (no ROS)."""

import numpy as np
import yaml


def load_footprints(path):
    """Return {name: (N, 2) float array} from a footprints YAML file."""
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f)
    result = {}
    for name, pts in data['footprints'].items():
        poly = np.asarray(pts, dtype=float)
        if poly.ndim != 2 or poly.shape[1] != 2 or len(poly) < 3:
            raise ValueError(f'footprint {name}: need >= 3 [x, y] points')
        result[name] = poly
    return result


def to_nav2_string(poly):
    """Format of the Nav2 costmap `footprint` parameter: '[[x, y], [x, y], ...]'."""
    return '[' + ', '.join(f'[{x:g}, {y:g}]' for x, y in poly) + ']'


def parse_nav2_string(text):
    return np.asarray(yaml.safe_load(text), dtype=float)


def contains(poly, points):
    """Even-odd point-in-polygon test, vectorized. points: (M, 2) -> (M,) bool."""
    pts = np.atleast_2d(np.asarray(points, dtype=float))
    x, y = pts[:, 0][:, None], pts[:, 1][:, None]
    x1, y1 = poly[:, 0][None, :], poly[:, 1][None, :]
    x2, y2 = np.roll(poly[:, 0], -1)[None, :], np.roll(poly[:, 1], -1)[None, :]
    crosses = (y1 > y) != (y2 > y)
    with np.errstate(divide='ignore', invalid='ignore'):
        x_at = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
    return np.count_nonzero(crosses & (x < x_at), axis=1) % 2 == 1


def edge_distance(poly, points):
    """Distance from each point to the polygon outline. points: (M, 2) -> (M,)."""
    pts = np.atleast_2d(np.asarray(points, dtype=float))[:, None, :]
    a = poly[None, :, :]
    b = np.roll(poly, -1, axis=0)[None, :, :]
    ab = b - a
    t = np.clip(np.sum((pts - a) * ab, axis=2) / np.sum(ab * ab, axis=2), 0.0, 1.0)
    closest = a + t[..., None] * ab
    return np.min(np.linalg.norm(pts - closest, axis=2), axis=1)


def inside_or_near(poly, points, margin):
    """Return True for points in the polygon or at `margin` or less from its outline."""
    return contains(poly, points) | (edge_distance(poly, points) <= margin)


def circumscribed_radius(poly):
    return float(np.max(np.linalg.norm(poly, axis=1)))


def self_hit_mask(ranges, angles, sensor_pose, poly, margin):
    """
    Mark the scan returns that are on the robot itself.

    ranges, angles: arrays with one value for each beam (angles in the sensor frame);
    sensor_pose: (x, y, yaw) of the sensor in the frame of the footprint. The function never
    marks ranges that are not finite.
    """
    ranges = np.asarray(ranges, dtype=float)
    sx, sy, syaw = sensor_pose
    a = np.asarray(angles, dtype=float) + syaw
    mask = np.zeros(len(ranges), bool)
    finite = np.isfinite(ranges)
    pts = np.column_stack([sx + ranges[finite] * np.cos(a[finite]),
                           sy + ranges[finite] * np.sin(a[finite])])
    if len(pts):
        mask[finite] = inside_or_near(poly, pts, margin)
    return mask
