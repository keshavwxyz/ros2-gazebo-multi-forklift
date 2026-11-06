"""
Make synthetic camera images of QR labels at known poses (for the tests; no ROS).

The module projects each label (symbol + quiet zone) through a pinhole camera with the
given intrinsics. Then it puts the label on a textured background. The true pose is known
exactly. Thus, the tests can compare the pose estimation with the ground truth without a
simulator.
"""

import math

import cv2
from forklift_perception.pose_estimation import object_points
from forklift_perception.qr_codec import label_edge, render_qr_label
import numpy as np


def camera_matrix(width, height, hfov):
    f = (width / 2.0) / math.tan(hfov / 2.0)
    return np.array([[f, 0, width / 2.0], [0, f, height / 2.0], [0, 0, 1.0]])


def look_at_rotation(normal_cam):
    """
    Rotation (target -> camera) for a label with a face normal along `normal_cam`.

    The label stays upright: its y axis is as near as possible to the up direction of the
    image (-y optical).
    """
    z = np.asarray(normal_cam, np.float64)
    z /= np.linalg.norm(z)
    up = np.array([0.0, -1.0, 0.0])
    x = np.cross(up, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def render(labels, k, width=1280, height=720, symbol_edge=0.30, background=110,
           noise_sigma=2.0, blur_sigma=0.0, seed=0):
    """
    Make labels in a grayscale image.

    labels: iterable of (payload, rotation (3x3, target->camera), translation (3,)).
    The pose is for the *symbol* center. This agrees with the target frame of pose_estimation.
    """
    rng = np.random.default_rng(seed)
    img = np.full((height, width), background, np.float32)
    img += rng.normal(0, 8.0, (height // 8 + 1, width // 8 + 1)).repeat(8, 0).repeat(8, 1)[
        :height, :width]                       # low-frequency texture
    for payload, rot, t in labels:
        tex, n = render_qr_label(payload, px_per_module=16)
        half = label_edge(symbol_edge, n) / 2.0
        # corners of the full label in the target frame (TL, TR, BR, BL)
        obj = object_points(2 * half)
        cam = (np.asarray(rot) @ obj.T).T + np.asarray(t)
        if np.any(cam[:, 2] <= 0.05):
            continue
        px = (np.asarray(k) @ cam.T).T
        px = (px[:, :2] / px[:, 2:]).astype(np.float32)
        s = tex.shape[0] - 1
        src = np.float32([[0, 0], [s, 0], [s, s], [0, s]])
        h = cv2.getPerspectiveTransform(src, px)
        warped = cv2.warpPerspective(tex.astype(np.float32), h, (width, height),
                                     flags=cv2.INTER_LINEAR, borderValue=0)
        mask = cv2.warpPerspective(np.ones_like(tex, np.float32), h, (width, height),
                                   flags=cv2.INTER_LINEAR, borderValue=0)
        # A printed label reflects less light than ideal white paper.
        img = img * (1 - mask) + (warped * 0.85 + 15) * mask
    if blur_sigma > 0:
        img = cv2.GaussianBlur(img, (0, 0), blur_sigma)
    img += rng.normal(0, noise_sigma, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)
