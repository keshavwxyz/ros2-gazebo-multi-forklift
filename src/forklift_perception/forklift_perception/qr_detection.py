"""
QR detection and decoding from camera pixels only (OpenCV, no ROS).

This module has no data about the simulator. The input is an image. The output is the
decoded text and the four symbol corners in pixel coordinates.
"""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class QrCandidate:
    payload: str                 # '' if a symbol was found but the decode was not possible
    corners: np.ndarray          # (4, 2) float32 px: top-left, top-right, bottom-right,
    #                              bottom-left of the symbol, in its own orientation

    @property
    def side_length_px(self):
        c = self.corners
        return float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]))

    @property
    def centre_px(self):
        return self.corners.mean(axis=0)


class QrDetector:
    """
    Find and decode all QR codes in a grayscale image.

    Procedure: find each symbol with the multi-code *detector* of OpenCV. Then decode each
    symbol separately on its own crop (scaled up if it is small). In Gazebo, the combined
    multi-code decoder of OpenCV 4.6 gave the payload of one code for two different symbols
    and did not report other symbols. Thus, it attached a payload to the wrong corners. When
    each crop is decoded separately, each payload comes from the symbol that the module read.
    """

    def __init__(self, min_side_px=12.0, crop_margin=0.35, upscale_below_px=90.0):
        self.detector = cv2.QRCodeDetector()
        self.min_side_px = min_side_px
        self.crop_margin = crop_margin
        self.upscale_below_px = upscale_below_px

    def detect(self, gray):
        if gray.ndim != 2:
            raise ValueError('expected a single-channel image')
        quads = self._locate(gray)
        result = []
        for quad in quads:
            cand = QrCandidate('', quad)
            if cand.side_length_px < self.min_side_px or not _is_convex_quad(quad):
                continue
            if any(np.linalg.norm(cand.centre_px - r.centre_px) < 0.5 * cand.side_length_px
                   for r in result):
                continue                      # duplicate location of an accepted symbol
            result.append(self._decode_crop(gray, cand))
        return result

    def _locate(self, gray):
        try:
            ok, points = self.detector.detectMulti(gray)
        except cv2.error:
            ok, points = False, None
        if ok and points is not None and len(points):
            return [p.reshape(4, 2).astype(np.float32) for p in points]
        ok, points = self.detector.detect(gray)
        if ok and points is not None:
            return [points.reshape(4, 2).astype(np.float32)]
        return []

    def _decode_crop(self, gray, cand):
        """Decode one symbol from a crop around it. Correct the corners from the crop result."""
        side = cand.side_length_px
        lo = np.floor(cand.corners.min(axis=0) - self.crop_margin * side).astype(int)
        hi = np.ceil(cand.corners.max(axis=0) + self.crop_margin * side).astype(int)
        lo = np.maximum(lo, 0)
        hi = np.minimum(hi, [gray.shape[1], gray.shape[0]])
        crop = gray[lo[1]:hi[1], lo[0]:hi[0]]
        if crop.size == 0:
            return cand
        scale = 2.0 if side < self.upscale_below_px else 1.0
        if scale != 1.0:
            crop = cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        try:
            text, points, _ = self.detector.detectAndDecode(crop)
        except cv2.error:
            text, points = '', None
        if text and points is not None:
            corners = points.reshape(4, 2).astype(np.float32) / scale + lo.astype(np.float32)
            # The crop can contain part of an adjacent symbol. Accept the decode only if
            # it comes from the symbol that the module found.
            if np.linalg.norm(corners.mean(axis=0) - cand.centre_px) <= 0.25 * side:
                return QrCandidate(text, corners)
        return self._decode_rectified(gray, cand)

    def _decode_rectified(self, gray, cand, size=300, margin=0.2):
        """
        Decode after the located quad is warped to a fronto-parallel square.

        If the perspective makes a symbol very short (the camera looks down at approximately
        45-60 deg at short range), the decode is frequently not successful. If the module
        removes the perspective first, the decode is successful. The corners of the result
        are the initially located corners (already in image space).
        """
        pad = size * margin
        dst = np.float32([[pad, pad], [pad + size, pad], [pad + size, pad + size],
                          [pad, pad + size]])
        h = cv2.getPerspectiveTransform(cand.corners.astype(np.float32), dst)
        full = int(round(size + 2 * pad))
        flat = cv2.warpPerspective(gray, h, (full, full), flags=cv2.INTER_LINEAR,
                                   borderValue=255)
        try:
            text, points, _ = self.detector.detectAndDecode(flat)
        except cv2.error:
            return cand
        if not text or points is None:
            return cand                       # found, but the decode is not possible
        return QrCandidate(text, cand.corners)


def _is_convex_quad(corners):
    """Reject corner sets that are not valid (sometimes, OpenCV gives twisted quads)."""
    c = corners.astype(np.float64)
    cross = [np.cross(c[(i + 1) % 4] - c[i], c[(i + 2) % 4] - c[(i + 1) % 4]) for i in range(4)]
    return all(x > 0 for x in cross) or all(x < 0 for x in cross)
