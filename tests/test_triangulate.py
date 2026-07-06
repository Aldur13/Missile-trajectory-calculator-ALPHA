import numpy as np

from trajcalc import calibration, triangulate


def _make_camera(name, position, look_at):
    rotation = calibration.rotation_from_look_at(np.array(position, dtype=np.float64), np.array(look_at, dtype=np.float64))
    return calibration.Camera(
        name=name,
        image_width=1920,
        image_height=1080,
        fx=1400.0,
        fy=1400.0,
        cx=960.0,
        cy=540.0,
        position=np.array(position, dtype=np.float64),
        rotation=rotation,
    )


def test_triangulate_point_recovers_synthetic_3d_point():
    cam_a = _make_camera("A", position=[0.0, -50.0, 2.0], look_at=[30.0, 0.0, 10.0])
    cam_b = _make_camera("B", position=[80.0, -20.0, 3.0], look_at=[30.0, 0.0, 10.0])

    true_point = np.array([28.0, 5.0, 12.0])

    pixel_a = triangulate.project_point(cam_a, true_point)
    pixel_b = triangulate.project_point(cam_b, true_point)

    recovered = triangulate.triangulate_point(cam_a, cam_b, pixel_a, pixel_b)

    np.testing.assert_allclose(recovered, true_point, atol=1e-4)


def test_ray_separation_near_zero_for_consistent_observation():
    cam_a = _make_camera("A", position=[0.0, -50.0, 2.0], look_at=[30.0, 0.0, 10.0])
    cam_b = _make_camera("B", position=[80.0, -20.0, 3.0], look_at=[30.0, 0.0, 10.0])

    true_point = np.array([28.0, 5.0, 12.0])
    pixel_a = triangulate.project_point(cam_a, true_point)
    pixel_b = triangulate.project_point(cam_b, true_point)

    separation = triangulate.ray_separation(cam_a, cam_b, pixel_a, pixel_b)
    assert separation < 1e-4
