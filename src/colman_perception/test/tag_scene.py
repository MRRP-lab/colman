"""Synthetic AprilTag images for detector tests: tag36h11 tags at known poses.

No real camera frames are kept in the repo, so scenes are rendered: the tag bitmap is
warped through a pinhole camera onto a table-coloured background. Imported by the
perception tests (not a test file itself).
"""

import cv2
import numpy as np
from colman_description.workcell import TAG_SIZE_M  # the cubes' tags: 39.2 mm

FAMILY_DICT = cv2.aruco.DICT_APRILTAG_36h11

# The overhead camera's 1080p intrinsics (merged calibration); 4K is exactly 2x.
K_1080P = np.array([[1460.5, 0, 962.4], [0, 1459.9, 547.1], [0, 0, 1.0]])
K_4K = K_1080P * np.array([[2], [2], [1]])
SIZE_1080P = (1920, 1080)
SIZE_4K = (3840, 2160)

# apriltag lists corners as lb-rb-rt-lt in its own image convention, which for an
# upright tag is top-right, top-left, bottom-left, bottom-right.
_H = TAG_SIZE_M / 2
TAG_CORNERS = np.array(
    [[_H, -_H, 0], [-_H, -_H, 0], [-_H, _H, 0], [_H, _H, 0]], dtype=np.float64
)

_BITMAP_PX = 8 * 32


def tag_bitmap(tag_id):
    """The tag's black-bordered 8x8 cell pattern, upscaled; uint8, 0 = black."""
    dictionary = cv2.aruco.getPredefinedDictionary(FAMILY_DICT)
    cells = cv2.aruco.generateImageMarker(dictionary, tag_id, 8, borderBits=1)
    return cv2.resize(cells, (_BITMAP_PX, _BITMAP_PX), interpolation=cv2.INTER_NEAREST)


def rotation(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
    rx, ry, rz = np.radians([roll_deg, pitch_deg, yaw_deg])
    cx, sx, cy, sy, cz, sz = (
        np.cos(rx),
        np.sin(rx),
        np.cos(ry),
        np.sin(ry),
        np.cos(rz),
        np.sin(rz),
    )
    mx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    my = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    mz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return mz @ my @ mx


def project(points, rvec, tvec, k):
    pixels, _ = cv2.projectPoints(points, rvec, tvec, k, None)
    return pixels.reshape(-1, 2)


def render(tags, k, size, background=200, tag_contrast=1.0, margin_px=0):
    """Render [(tag_id, R, t)] into a gray uint8 image of `size` (w, h).

    The table is a flat `background` gray with a white quiet zone around each tag.
    tag_contrast < 1 pulls the black and white of the tags towards mid gray.
    """
    w, h = size
    image = np.full((h, w), background, np.uint8)
    for tag_id, rot, trans in tags:
        rvec, _ = cv2.Rodrigues(rot)
        bitmap = tag_bitmap(tag_id)
        # White quiet zone, one cell wide, around the 8x8 pattern.
        pad = _BITMAP_PX // 8
        framed = cv2.copyMakeBorder(
            bitmap, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255
        )
        if tag_contrast < 1.0:
            framed = (
                127.5 + (framed.astype(np.float32) - 127.5) * tag_contrast
            ).astype(np.uint8)
        full = _BITMAP_PX + 2 * pad
        scale = (full / _BITMAP_PX) * TAG_SIZE_M
        half = scale / 2
        outer = np.array(
            [[-half, -half, 0], [half, -half, 0], [half, half, 0], [-half, half, 0]]
        )
        dst = project(outer, rvec, np.asarray(trans, float), k).astype(np.float32)
        src = np.array([[0, 0], [full, 0], [full, full], [0, full]], np.float32)
        homography = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(
            framed,
            homography,
            (w, h),
            flags=cv2.INTER_AREA,
            borderMode=cv2.BORDER_TRANSPARENT,
            dst=image,
        )
        image = warped
    return image


def solve_pose(corners, k):
    """Camera-from-tag (R, t) from the detector's four corners, by planar PnP."""
    ok, rvec, tvec = cv2.solvePnP(
        TAG_CORNERS,
        np.asarray(corners, np.float64),
        k,
        None,
        flags=cv2.SOLVEPNP_IPPE,
    )
    if not ok:
        return None
    rot, _ = cv2.Rodrigues(rvec)
    return rot, tvec.reshape(3)


def angle_deg(r1, r2):
    c = (np.trace(r1.T @ r2) - 1) / 2
    return float(np.degrees(np.arccos(np.clip(c, -1, 1))))
