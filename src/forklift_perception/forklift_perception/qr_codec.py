"""
QR code generation with exact module geometry (OpenCV only, no ROS).

The Gazebo texture generator uses this module. The tests that make synthetic camera
images also use it.
"""

import cv2
import numpy as np

QUIET_ZONE_MODULES = 4          # minimum quiet zone of ISO/IEC 18004
_LEVELS = {'L': 'QRCodeEncoder_CORRECT_LEVEL_L', 'M': 'QRCodeEncoder_CORRECT_LEVEL_M',
           'Q': 'QRCodeEncoder_CORRECT_LEVEL_Q', 'H': 'QRCodeEncoder_CORRECT_LEVEL_H'}


def qr_modules(payload, correction='M', max_version=10):
    """
    Return the QR symbol as a boolean module matrix (True = dark), without the quiet zone.

    The function tries the versions in increasing sequence. The automatic version selection
    of OpenCV 4.6 raises an exception for some payload lengths. Thus, the function finds the
    smallest applicable version explicitly.
    """
    if not payload:
        raise ValueError('payload must be non-empty')
    for version in range(1, max_version + 1):
        params = cv2.QRCodeEncoder.Params()
        params.correction_level = getattr(cv2, _LEVELS[correction])
        params.version = version
        try:
            img = cv2.QRCodeEncoder.create(params).encode(payload)
        except cv2.error:
            continue
        dark = np.argwhere(img == 0)
        (r0, c0), (r1, c1) = dark.min(axis=0), dark.max(axis=0)
        modules = img[r0:r1 + 1, c0:c1 + 1] == 0
        if modules.shape != (17 + 4 * version,) * 2:
            raise RuntimeError(f'unexpected symbol size {modules.shape} for version {version}')
        return modules
    raise ValueError(f'payload too long for QR version <= {max_version}: {payload!r}')


def render_qr_label(payload, px_per_module=20, correction='M'):
    """
    Make a printable label: the symbol plus a white quiet zone of 4 modules.

    Return (image, n_modules): uint8 grayscale image (white = 255), and the symbol size in
    modules. If the printed symbol (without the quiet zone) is `edge` metres wide, the
    full label is label_edge(edge, n_modules) metres wide.
    """
    modules = qr_modules(payload, correction)
    n = modules.shape[0]
    canvas = np.full((n + 2 * QUIET_ZONE_MODULES,) * 2, 255, np.uint8)
    canvas[QUIET_ZONE_MODULES:QUIET_ZONE_MODULES + n,
           QUIET_ZONE_MODULES:QUIET_ZONE_MODULES + n][modules] = 0
    image = cv2.resize(canvas, None, fx=px_per_module, fy=px_per_module,
                       interpolation=cv2.INTER_NEAREST)
    return image, n


def label_edge(symbol_edge, n_modules):
    """Physical label size (symbol + quiet zone) for a given symbol edge length."""
    return symbol_edge * (n_modules + 2 * QUIET_ZONE_MODULES) / n_modules
