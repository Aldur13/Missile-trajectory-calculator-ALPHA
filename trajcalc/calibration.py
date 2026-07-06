"""Camera intrinsic/extrinsic calibration for stereo triangulation."""
from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Optional

import numpy as np
import yaml


@dataclasses.dataclass
class Camera:
    name: str
    image_width: int
    image_height: int
    fx: float
    fy: float
    cx: float
    cy: float
    position: np.ndarray  # camera center C in world coordinates, shape (3,)
    rotation: np.ndarray  # world-to-camera rotation matrix R, shape (3,3)
    dist_coeffs: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros(5))

    def K(self) -> np.ndarray:
        return np.array(
            [
                [self.fx, 0.0, self.cx],
                [0.0, self.fy, self.cy],
                [0.0, 0.0, 1.0],
            ]
        )

    def extrinsic_Rt(self) -> np.ndarray:
        """3x4 [R | t] mapping world points to this camera's coordinate frame."""
        t = -self.rotation @ self.position
        return np.hstack([self.rotation, t.reshape(3, 1)])

    def projection_matrix(self) -> np.ndarray:
        return self.K() @ self.extrinsic_Rt()

    def undistort_pixel(self, pixel: tuple) -> np.ndarray:
        """Undistorted pixel-scale coordinates of a raw pixel observation."""
        import cv2

        pts = np.array([[pixel]], dtype=np.float64)
        undistorted = cv2.undistortPoints(pts, self.K(), self.dist_coeffs, P=self.K())
        return undistorted[0, 0]

    def unproject_ray(self, pixel: tuple) -> tuple:
        """World-space (origin, unit direction) of the sight ray through a pixel."""
        u, v = self.undistort_pixel(pixel)
        k_inv = np.linalg.inv(self.K())
        dir_cam = k_inv @ np.array([u, v, 1.0])
        dir_world = self.rotation.T @ dir_cam
        dir_world = dir_world / np.linalg.norm(dir_world)
        return self.position.copy(), dir_world


def rotation_from_look_at(
    position: np.ndarray, target: np.ndarray, up: np.ndarray = np.array([0.0, 0.0, 1.0])
) -> np.ndarray:
    """World-to-camera rotation for a camera at `position` aimed at `target`.

    Camera-frame convention matches OpenCV: x right, y down, z forward.
    """
    forward = target - position
    forward = forward / np.linalg.norm(forward)
    right = np.cross(forward, up)
    right = right / np.linalg.norm(right)
    true_up = np.cross(right, forward)
    return np.array([right, -true_up, forward])


def _build_camera(name: str, spec: dict) -> Camera:
    position = np.array(spec["position"], dtype=np.float64)
    if "rotation" in spec:
        rotation = np.array(spec["rotation"], dtype=np.float64)
    else:
        target = np.array(spec["look_at"], dtype=np.float64)
        up = np.array(spec.get("up", [0.0, 0.0, 1.0]), dtype=np.float64)
        rotation = rotation_from_look_at(position, target, up)

    dist = np.array(spec.get("dist_coeffs", [0, 0, 0, 0, 0]), dtype=np.float64)

    return Camera(
        name=name,
        image_width=spec["image_width"],
        image_height=spec["image_height"],
        fx=spec["fx"],
        fy=spec["fy"],
        cx=spec["cx"],
        cy=spec["cy"],
        position=position,
        rotation=rotation,
        dist_coeffs=dist,
    )


def load_calibration(path) -> dict:
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    return {name: _build_camera(name, spec) for name, spec in data["cameras"].items()}


def intrinsics_from_specs(
    image_width: int,
    image_height: int,
    focal_length_mm: float,
    sensor_width_mm: float,
    sensor_height_mm: Optional[float] = None,
) -> dict:
    """Rough intrinsics estimate from a camera/lens spec sheet.

    Less accurate than a checkerboard calibration (cv2.calibrateCamera) — use
    that instead if results better than a few percent are needed.
    """
    if sensor_height_mm is None:
        sensor_height_mm = sensor_width_mm * image_height / image_width

    fx = image_width * focal_length_mm / sensor_width_mm
    fy = image_height * focal_length_mm / sensor_height_mm
    return {
        "fx": fx,
        "fy": fy,
        "cx": image_width / 2.0,
        "cy": image_height / 2.0,
    }
