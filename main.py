#!/usr/bin/env python3
"""CLI entry point: two-camera stereo missile trajectory reconstruction."""
from __future__ import annotations

import argparse
from typing import Optional

from trajcalc import ballistics, calibration, picker, triangulate, visualize


def _parse_pixel(value: str) -> tuple:
    x_str, y_str = value.split(",")
    return float(x_str), float(y_str)


def _get_pixel(image_path: str, manual: Optional[str]) -> tuple:
    if manual is not None:
        return _parse_pixel(manual)
    return picker.pick_pixel(image_path)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Reconstruct a missile's 3D trajectory from two stereo-camera "
        "photo pairs taken at two moments in flight."
    )
    parser.add_argument("--calibration", required=True, help="Path to calibration YAML")

    parser.add_argument("--img-a-t1", required=True)
    parser.add_argument("--img-b-t1", required=True)
    parser.add_argument("--img-a-t2", required=True)
    parser.add_argument("--img-b-t2", required=True)

    parser.add_argument("--pixel-a-t1", help="Override the click picker with 'x,y'")
    parser.add_argument("--pixel-b-t1", help="Override the click picker with 'x,y'")
    parser.add_argument("--pixel-a-t2", help="Override the click picker with 'x,y'")
    parser.add_argument("--pixel-b-t2", help="Override the click picker with 'x,y'")

    parser.add_argument("--dt", required=True, type=float, help="Seconds between t1 and t2")

    parser.add_argument("--mass", required=True, type=float, help="Missile mass (kg)")
    parser.add_argument("--area", required=True, type=float, help="Cross-sectional area (m^2)")
    parser.add_argument("--drag-coeff", type=float, default=0.3)
    parser.add_argument("--air-density", type=float, default=1.225)

    parser.add_argument("--ground-z", type=float, default=0.0)
    parser.add_argument("--t-max", type=float, default=300.0)

    parser.add_argument(
        "--plot-output", help="Save the trajectory plot to this path instead of showing it"
    )
    parser.add_argument(
        "--overlay-camera",
        choices=["A", "B"],
        help="Also render an AR-style trajectory overlay on this camera's t1 photo",
    )
    parser.add_argument("--overlay-output", help="Output path for the overlay image")

    return parser


def main():
    args = build_arg_parser().parse_args()

    cameras = calibration.load_calibration(args.calibration)
    cam_a, cam_b = cameras["A"], cameras["B"]

    px_a_t1 = _get_pixel(args.img_a_t1, args.pixel_a_t1)
    px_b_t1 = _get_pixel(args.img_b_t1, args.pixel_b_t1)
    px_a_t2 = _get_pixel(args.img_a_t2, args.pixel_a_t2)
    px_b_t2 = _get_pixel(args.img_b_t2, args.pixel_b_t2)

    separation_t1 = triangulate.ray_separation(cam_a, cam_b, px_a_t1, px_b_t1)
    separation_t2 = triangulate.ray_separation(cam_a, cam_b, px_a_t2, px_b_t2)
    print(f"Ray separation at t1: {separation_t1:.4f} m (triangulation quality check)")
    print(f"Ray separation at t2: {separation_t2:.4f} m (triangulation quality check)")

    p1 = triangulate.triangulate_point(cam_a, cam_b, px_a_t1, px_b_t1)
    p2 = triangulate.triangulate_point(cam_a, cam_b, px_a_t2, px_b_t2)
    print(f"Triangulated point at t1: {p1}")
    print(f"Triangulated point at t2: {p2}")

    k = ballistics.drag_k(args.mass, args.area, args.drag_coeff, args.air_density)

    v0, result = ballistics.solve_initial_velocity(p1, p2, args.dt, k)
    if not result.success:
        print(f"Warning: velocity solver did not fully converge ({result.message})")

    trajectory = ballistics.simulate_trajectory(
        p1, v0, k, dt=0.001, t_max=args.t_max, ground_z=args.ground_z
    )

    print()
    print(visualize.summarize(p1, v0, trajectory))

    if args.overlay_camera:
        overlay_cam = cam_a if args.overlay_camera == "A" else cam_b
        overlay_image = args.img_a_t1 if args.overlay_camera == "A" else args.img_b_t1
        output_path = args.overlay_output or "trajectory_overlay.png"
        visualize.overlay_on_image(overlay_cam, overlay_image, trajectory, output_path)
        print(f"Overlay image saved to {output_path}")

    visualize.plot_trajectory(p1, p2, trajectory, args.plot_output)


if __name__ == "__main__":
    main()
