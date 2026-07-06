"""Reporting and plotting for a solved trajectory."""
from __future__ import annotations

from typing import Optional

import numpy as np


def summarize(p0, v0, trajectory: dict) -> str:
    v0 = np.array(v0)
    horizontal_speed = np.linalg.norm(v0[:2])
    speed = np.linalg.norm(v0)
    angle_deg = np.degrees(np.arctan2(v0[2], horizontal_speed))

    apogee_t, apogee_pos = trajectory["apogee"]
    impact = trajectory["impact"]

    lines = [
        f"Launch point (t1):            {np.array(p0)}",
        f"Solved launch velocity:       {v0} m/s",
        f"Launch speed:                 {speed:.2f} m/s",
        f"Launch angle:                 {angle_deg:.2f} deg above horizontal",
        f"Apogee:                        {apogee_pos[2]:.2f} m at t={apogee_t:.2f}s (from t1)",
    ]

    if impact is not None:
        impact_t, impact_pos = impact
        ground_range = np.linalg.norm(impact_pos[:2] - np.array(p0)[:2])
        lines.append(f"Impact point:                  {impact_pos}")
        lines.append(f"Time of flight (t1 -> impact): {impact_t:.2f} s")
        lines.append(f"Ground range from launch:      {ground_range:.2f} m")
    else:
        lines.append("Impact: not reached within simulated t_max")

    return "\n".join(lines)


def plot_trajectory(p1, p2, trajectory: dict, output_path: Optional[str] = None):
    import matplotlib.pyplot as plt

    positions = trajectory["positions"]

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label="Fitted trajectory")
    ax.scatter(*p1, color="green", s=60, label="Observed point (t1)")
    ax.scatter(*p2, color="orange", s=60, label="Observed point (t2)")

    _, apogee_pos = trajectory["apogee"]
    ax.scatter(*apogee_pos, color="red", marker="^", s=80, label="Apogee")

    if trajectory["impact"] is not None:
        _, impact_pos = trajectory["impact"]
        ax.scatter(*impact_pos, color="black", marker="x", s=80, label="Impact")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m, altitude)")
    ax.legend()
    ax.set_title("Reconstructed missile trajectory")

    if output_path:
        fig.savefig(output_path, dpi=150)
    else:
        plt.show()


def overlay_on_image(camera, image_path: str, trajectory: dict, output_path: str):
    """Project the fitted trajectory back into one camera's photo (AR-style check)."""
    import cv2

    from trajcalc.triangulate import project_point

    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(image_path)

    positions = trajectory["positions"]
    stride = max(1, len(positions) // 500)

    pts = []
    for pos in positions[::stride]:
        u, v = project_point(camera, pos)
        if 0 <= u < camera.image_width and 0 <= v < camera.image_height:
            pts.append((int(u), int(v)))

    for i in range(1, len(pts)):
        cv2.line(image, pts[i - 1], pts[i], (0, 0, 255), 2)

    cv2.imwrite(output_path, image)
