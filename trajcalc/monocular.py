"""Best-effort single-camera trajectory approximation.

A single camera cannot recover true depth from two frames alone. This module
assumes the missile stayed at a fixed assumed distance from the camera along
its optical axis, and maps each pixel to the point on the plane perpendicular
to the camera's forward direction at that distance. This is an approximation
for when only one camera/angle is available, not a measurement — accuracy is
bounded by how good the assumed distance is.
"""
from __future__ import annotations

import numpy as np

from trajcalc.calibration import Camera, rotation_from_look_at


def default_camera(
    image_width: int, image_height: int, fov_deg: float = 60.0, camera_height: float = 1.5
) -> Camera:
    """A camera at `camera_height` above the origin, aimed level along +X."""
    fx = image_width / (2.0 * np.tan(np.radians(fov_deg) / 2.0))
    fy = fx
    position = np.array([0.0, 0.0, camera_height])
    rotation = rotation_from_look_at(position, position + np.array([1.0, 0.0, 0.0]))

    return Camera(
        name="mono",
        image_width=image_width,
        image_height=image_height,
        fx=fx,
        fy=fy,
        cx=image_width / 2.0,
        cy=image_height / 2.0,
        position=position,
        rotation=rotation,
    )


def pixel_to_point_at_distance(camera: Camera, pixel: tuple, distance: float) -> np.ndarray:
    """World point along the sight ray through `pixel`, at `distance` from the camera."""
    u, v = camera.undistort_pixel(pixel)
    point_cam = np.array(
        [
            (u - camera.cx) / camera.fx * distance,
            (v - camera.cy) / camera.fy * distance,
            distance,
        ]
    )
    return camera.rotation.T @ point_cam + camera.position
