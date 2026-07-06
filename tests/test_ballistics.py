import numpy as np
import pytest

from trajcalc import ballistics


def test_propagate_no_drag_matches_analytic():
    p0 = np.array([0.0, 0.0, 0.0])
    v0 = np.array([20.0, 5.0, 20.0])
    t = 2.0

    pos, vel = ballistics.propagate(p0, v0, k=0.0, t_target=t, dt=0.0005)

    expected_xy = p0[:2] + v0[:2] * t
    expected_z = v0[2] * t - 0.5 * ballistics.GRAVITY * t**2
    expected_vz = v0[2] - ballistics.GRAVITY * t

    assert pos[0] == pytest.approx(expected_xy[0], abs=1e-3)
    assert pos[1] == pytest.approx(expected_xy[1], abs=1e-3)
    assert pos[2] == pytest.approx(expected_z, abs=1e-3)
    assert vel[2] == pytest.approx(expected_vz, abs=1e-3)


def test_solver_recovers_known_velocity_no_drag():
    p0 = np.array([0.0, 0.0, 0.0])
    true_v0 = np.array([30.0, -10.0, 25.0])
    dt_flight = 1.7

    p1, _ = ballistics.propagate(p0, true_v0, k=0.0, t_target=dt_flight, dt=0.0005)

    bad_guess = true_v0 + np.array([15.0, -15.0, -15.0])
    solved_v0, result = ballistics.solve_initial_velocity(
        p0, p1, dt_flight, k=0.0, v0_guess=bad_guess
    )

    assert result.success
    np.testing.assert_allclose(solved_v0, true_v0, atol=1e-2)


def test_solver_recovers_known_velocity_with_drag():
    p0 = np.array([0.0, 0.0, 0.0])
    true_v0 = np.array([40.0, 5.0, 30.0])
    dt_flight = 2.0
    k = ballistics.drag_k(mass_kg=5.0, area_m2=0.02, drag_coeff=0.4)

    p1, _ = ballistics.propagate(p0, true_v0, k=k, t_target=dt_flight, dt=0.0005)

    solved_v0, result = ballistics.solve_initial_velocity(p0, p1, dt_flight, k=k)

    assert result.success
    np.testing.assert_allclose(solved_v0, true_v0, atol=1e-2)


def test_simulate_trajectory_finds_apogee_and_impact():
    p0 = np.array([0.0, 0.0, 0.0])
    v0 = np.array([10.0, 0.0, 15.0])

    trajectory = ballistics.simulate_trajectory(p0, v0, k=0.0, dt=0.001, t_max=10.0)

    apogee_t, _ = trajectory["apogee"]
    expected_apogee_t = v0[2] / ballistics.GRAVITY
    assert apogee_t == pytest.approx(expected_apogee_t, abs=0.01)

    assert trajectory["impact"] is not None
    impact_t, impact_pos = trajectory["impact"]
    expected_impact_t = 2 * v0[2] / ballistics.GRAVITY
    assert impact_t == pytest.approx(expected_impact_t, abs=0.01)
    assert impact_pos[2] == pytest.approx(0.0, abs=1e-2)
