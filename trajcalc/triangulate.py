"""Stereo triangulation of a missile's world position from two camera views."""
from __future__ import annotations

import numpy as np

from trajcalc.calibration import Camera


def triangulate_point(cam_a: Camera, cam_b: Camera, pixel_a: tuple, pixel_b: tuple) -> np.ndarray:
    """Recover the 3D world point observed at `pixel_a`/`pixel_b` (DLT triangulation)."""
    import cv2

    ua = cam_a.undistort_pixel(pixel_a)
    ub = cam_b.undistort_pixel(pixel_b)

    pts_a = np.array(ua, dtype=np.float64).reshape(2, 1)
    pts_b = np.array(ub, dtype=np.float64).reshape(2, 1)

    homogeneous = cv2.triangulatePoints(
        cam_a.projection_matrix(), cam_b.projection_matrix(), pts_a, pts_b
    )
    return (homogeneous[:3] / homogeneous[3]).flatten()


def ray_separation(cam_a: Camera, cam_b: Camera, pixel_a: tuple, pixel_b: tuple) -> float:
    """Shortest distance between the two cameras' sight rays.

    A quality check: large separation relative to the scene scale means the
    stereo geometry (baseline/angle) is too weak for reliable triangulation,
    or the two pixels weren't actually the same real-world instant/point.
    """
    o1, d1 = cam_a.unproject_ray(pixel_a)
    o2, d2 = cam_b.unproject_ray(pixel_b)

    cross = np.cross(d1, d2)
    norm = np.linalg.norm(cross)
    if norm < 1e-9:
        return float(np.linalg.norm(np.cross(o2 - o1, d1)))
    return float(abs(np.dot(o2 - o1, cross)) / norm)


def project_point(camera: Camera, point_world: np.ndarray) -> np.ndarray:
    """Forward-project a 3D world point into a camera's pixel coordinates."""
    homogeneous = np.append(point_world, 1.0)
    projected = camera.projection_matrix() @ homogeneous
    return projected[:2] / projected[2]
