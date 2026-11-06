"""
Small URDF kinematics module for validation and tools (no ROS graph is necessary).

This module makes the forklift URDF from xacro. It calculates the link poses in
base_footprint from the joint origins. Thus, the tests can examine the frame
positions and the mass distribution.
"""

import math
import pathlib
import xml.etree.ElementTree as ET

import numpy as np

_SOURCE_XACRO = pathlib.Path(__file__).resolve().parents[1] / 'urdf' / 'forklift.urdf.xacro'


def default_xacro():
    """
    Return the path of the forklift xacro.

    The source tree has the xacro next to this module (symlink install). A copy install
    puts this module in site-packages, thus use the share directory of the package.
    """
    if _SOURCE_XACRO.is_file():
        return _SOURCE_XACRO
    from ament_index_python.packages import get_package_share_directory
    share = pathlib.Path(get_package_share_directory('forklift_description'))
    return share / 'urdf' / 'forklift.urdf.xacro'


def generate_urdf(xacro_file=None, **mappings):
    """Run xacro and return the URDF XML string."""
    import xacro
    if xacro_file is None:
        xacro_file = default_xacro()
    doc = xacro.process_file(str(xacro_file),
                             mappings={k: str(v) for k, v in mappings.items()})
    return doc.toxml()


def vec(text, default='0 0 0'):
    return np.array([float(v) for v in (text or default).split()])


def rpy_to_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def origin_tf(elem):
    """4x4 transform of the <origin> child of an element (identity if there is no <origin>)."""
    tf = np.eye(4)
    o = elem.find('origin') if elem is not None else None
    if o is not None:
        tf[:3, :3] = rpy_to_matrix(*vec(o.get('rpy')))
        tf[:3, 3] = vec(o.get('xyz'))
    return tf


class Model:
    """Transforms of all links in the root frame for the given joint positions."""

    def __init__(self, urdf_xml):
        self.root = ET.fromstring(urdf_xml)
        self.links = {link.get('name'): link for link in self.root.findall('link')}
        self.joints = {j.get('name'): j for j in self.root.findall('joint')}
        self.parent_joint = {j.find('child').get('link'): j for j in self.joints.values()}

    def tf(self, link, q=None):
        q = q or {}
        out = np.eye(4)
        while link in self.parent_joint:
            j = self.parent_joint[link]
            local = origin_tf(j)
            value = q.get(j.get('name'), 0.0)
            axis = vec(j.find('axis').get('xyz')) if j.find('axis') is not None else None
            if j.get('type') == 'prismatic':
                motion = np.eye(4)
                motion[:3, 3] = axis * value
                local = local @ motion
            elif j.get('type') in ('revolute', 'continuous') and value:
                motion = np.eye(4)
                motion[:3, :3] = axis_angle(axis, value)
                local = local @ motion
            out = local @ out
            link = j.find('parent').get('link')
        return out

    def point(self, link, xyz, q=None):
        return (self.tf(link, q) @ np.append(xyz, 1.0))[:3]

    def mass_and_com(self, q=None):
        total, moment = 0.0, np.zeros(3)
        for name, link in self.links.items():
            inertial = link.find('inertial')
            if inertial is None:
                continue
            m = float(inertial.find('mass').get('value'))
            com = self.point(name, vec(inertial.find('origin').get('xyz')), q)
            total += m
            moment += m * com
        return total, moment / total

    def collision_points(self, q=None, samples=24):
        """Yield (link name, Nx3 surface sample points) for each collision primitive."""
        for name, link in self.links.items():
            for col in link.findall('collision'):
                tf = self.tf(name, q) @ origin_tf(col)
                pts = primitive_points(col.find('geometry')[0], samples)
                yield name, (tf[:3, :3] @ pts.T).T + tf[:3, 3]


def axis_angle(axis, angle):
    x, y, z = axis / np.linalg.norm(axis)
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return np.array([
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ])


def primitive_points(geom, samples):
    if geom.tag == 'box':
        sx, sy, sz = vec(geom.get('size')) / 2
        return np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)])
    angles = np.linspace(0, 2 * math.pi, samples, endpoint=False)
    r = float(geom.get('radius'))
    if geom.tag == 'cylinder':
        h = float(geom.get('length')) / 2
        ring = np.stack([r * np.cos(angles), r * np.sin(angles)], axis=1)
        return np.vstack([np.column_stack([ring, np.full(samples, z)]) for z in (-h, h)])
    if geom.tag == 'sphere':
        pts = [[r * math.cos(a) * math.cos(b), r * math.sin(a) * math.cos(b), r * math.sin(b)]
               for a in angles for b in np.linspace(-math.pi / 2, math.pi / 2, 7)]
        return np.array(pts)
    raise ValueError(f'unsupported collision geometry: {geom.tag}')
