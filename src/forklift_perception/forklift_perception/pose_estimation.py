"""
QR target pose from its four corners, the known printed size and the camera intrinsics.

Method: planar PnP with the IPPE_SQUARE solver of OpenCV. This solver is specially for a
square with a known side. It gives the two solutions of the planar pose ambiguity. The
module keeps the solution with the lower reprojection error. It also reports the
difference between the two solutions.

QR target frame: the origin is at the symbol center, x is to the right of the symbol, y is
up, and z points out of the printed face (to a camera that can read it). The pose is in
the camera optical frame (z forward, x right, y down).

Assumptions (docs/perception.md):
  - The pinhole intrinsics from CameraInfo are correct.
  - The symbol is flat and its printed size is exactly `edge` metres.
  - CameraInfo.d gives the lens distortion (zero for the Gazebo camera).
"""

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass
class QrPose:
    rvec: np.ndarray             # (3,) Rodrigues rotation, target -> camera
    tvec: np.ndarray             # (3,) target origin in the camera frame, m
    rotation: np.ndarray         # (3, 3)
    reprojection_error_px: float  # RMS for the 4 corners
    ambiguity_ratio: float       # error(best) / error(other); near 1 = ambiguous

    @property
    def distance(self):
        return float(np.linalg.norm(self.tvec))

    @property
    def normal_in_camera(self):
        return self.rotation[:, 2]

    @property
    def viewing_angle_deg(self):
        """Angle between the face normal and the line of sight (0 = head-on)."""
        to_camera = -self.tvec / np.linalg.norm(self.tvec)
        return math.degrees(math.acos(float(np.clip(self.normal_in_camera @ to_camera, -1, 1))))

    def quaternion_xyzw(self):
        return rotation_to_quaternion(self.rotation)


def object_points(edge):
    """Symbol corners in the target frame, in the IPPE_SQUARE sequence (TL, TR, BR, BL)."""
    h = edge / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], np.float64)


def estimate_qr_pose(corners_px, edge, camera_matrix, dist_coeffs=None):
    """Return a QrPose, or None if there is no physically valid solution."""
    k = np.asarray(camera_matrix, np.float64).reshape(3, 3)
    d = np.zeros(5) if dist_coeffs is None or len(dist_coeffs) == 0 else \
        np.asarray(dist_coeffs, np.float64)
    img = np.asarray(corners_px, np.float64).reshape(4, 1, 2)
    obj = object_points(edge)
    try:
        n, rvecs, tvecs, errors = cv2.solvePnPGeneric(obj, img, k, d,
                                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
    except cv2.error:
        return None
    candidates = []
    for i in range(n):
        rvec, tvec = rvecs[i].reshape(3), tvecs[i].reshape(3)
        rot, _ = cv2.Rodrigues(rvec)
        # Physically valid: in front of the camera, with the printed face turned to the camera.
        if tvec[2] <= 0 or rot[:, 2] @ (-tvec) <= 0:
            continue
        candidates.append((float(np.asarray(errors).reshape(-1)[i]), rvec, tvec, rot))
    if not candidates:
        return None
    candidates.sort(key=lambda c: c[0])
    err, rvec, tvec, rot = candidates[0]
    other = candidates[1][0] if len(candidates) > 1 else math.inf
    ratio = err / other if other > 0 and math.isfinite(other) else 0.0
    return QrPose(rvec, tvec, rot, err, ratio)


def rotation_to_quaternion(r):
    """Rotation matrix -> quaternion (x, y, z, w), with a numerically stable branch selection."""
    m = np.asarray(r, np.float64)
    tr = np.trace(m)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x = 0.25 * s, (m[2, 1] - m[1, 2]) / s
        y, z = (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x = (m[2, 1] - m[1, 2]) / s, 0.25 * s
        y, z = (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s
        y, z = 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s
        y, z = (m[1, 2] + m[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def confidence(side_px, reprojection_error_px, decoded):
    """
    Heuristic confidence from 0 to 1.

    The confidence increases with the apparent size (the maximum is at 60 px, where each
    module of a version-1 symbol is approximately 3 px). It decreases with the corner
    reprojection error. It is 0 if the decode was not successful.
    """
    if not decoded:
        return 0.0
    size_term = min(1.0, max(0.0, side_px / 60.0))
    fit_term = math.exp(-reprojection_error_px / 2.0)
    return float(size_term * fit_term)
