"""Projectile physics: quadratic-drag integration and initial-velocity solving."""
from __future__ import annotations

from typing import Optional

import numpy as np
from scipy.optimize import least_squares

GRAVITY = 9.80665


def drag_k(mass_kg: float, area_m2: float, drag_coeff: float, air_density: float = 1.225) -> float:
    """Quadratic-drag coefficient k in acceleration = -k*|v|*v (per unit mass)."""
    return 0.5 * air_density * drag_coeff * area_m2 / mass_kg


def _acceleration(velocity: np.ndarray, k: float, g: float) -> np.ndarray:
    speed = np.linalg.norm(velocity)
    drag = -k * speed * velocity
    gravity = np.array([0.0, 0.0, -g])
    return drag + gravity


def _rk4_step(position, velocity, k, g, dt):
    def deriv(pos, vel):
        return vel, _acceleration(vel, k, g)

    k1p, k1v = deriv(position, velocity)
    k2p, k2v = deriv(position + 0.5 * dt * k1p, velocity + 0.5 * dt * k1v)
    k3p, k3v = deriv(position + 0.5 * dt * k2p, velocity + 0.5 * dt * k2v)
    k4p, k4v = deriv(position + dt * k3p, velocity + dt * k3v)

    new_position = position + (dt / 6.0) * (k1p + 2 * k2p + 2 * k3p + k4p)
    new_velocity = velocity + (dt / 6.0) * (k1v + 2 * k2v + 2 * k3v + k4v)
    return new_position, new_velocity


def propagate(p0, v0, k: float, t_target: float, g: float = GRAVITY, dt: float = 0.001):
    """Integrate forward to a specific time `t_target`, returning (position, velocity)."""
    position, velocity = np.array(p0, dtype=np.float64), np.array(v0, dtype=np.float64)
    t = 0.0
    while t < t_target - 1e-12:
        step = min(dt, t_target - t)
        position, velocity = _rk4_step(position, velocity, k, g, step)
        t += step
    return position, velocity


def simulate_trajectory(
    p0,
    v0,
    k: float,
    g: float = GRAVITY,
    dt: float = 0.001,
    t_max: float = 300.0,
    ground_z: float = 0.0,
) -> dict:
    """Integrate forward from launch until impact with `ground_z` (or `t_max`).

    Returns a dict with the full time series plus derived apogee/impact events.
    """
    position, velocity = np.array(p0, dtype=np.float64), np.array(v0, dtype=np.float64)
    t = 0.0

    times = [t]
    positions = [position.copy()]
    velocities = [velocity.copy()]

    apogee = (t, position.copy())
    impact = None

    while t < t_max:
        prev_position = position
        position, velocity = _rk4_step(position, velocity, k, g, dt)
        t += dt

        if position[2] > apogee[1][2]:
            apogee = (t, position.copy())

        if position[2] <= ground_z and prev_position[2] > ground_z:
            frac = (prev_position[2] - ground_z) / (prev_position[2] - position[2])
            impact_t = t - dt + frac * dt
            impact_pos = prev_position + frac * (position - prev_position)
            impact = (impact_t, impact_pos)
            times.append(impact_t)
            positions.append(impact_pos)
            velocities.append(velocity.copy())
            break

        times.append(t)
        positions.append(position.copy())
        velocities.append(velocity.copy())

    return {
        "times": np.array(times),
        "positions": np.array(positions),
        "velocities": np.array(velocities),
        "apogee": apogee,
        "impact": impact,
    }


def solve_initial_velocity(
    p0, p1, dt_flight: float, k: float, g: float = GRAVITY, v0_guess: Optional[np.ndarray] = None
):
    """Find the initial velocity at `p0` (t=0) that reaches `p1` at `t=dt_flight`.

    Shooting method: since drag_k is supplied rather than fit, this is a
    3-unknown / 3-equation root-find, solved as a least-squares problem.
    """
    p0 = np.array(p0, dtype=np.float64)
    p1 = np.array(p1, dtype=np.float64)

    if v0_guess is None:
        v0_guess = (p1 - p0) / dt_flight
        v0_guess[2] += 0.5 * g * dt_flight
    else:
        v0_guess = np.array(v0_guess, dtype=np.float64)

    def residual(v0):
        position, _ = propagate(p0, v0, k, dt_flight, g=g)
        return position - p1

    result = least_squares(residual, v0_guess)
    return result.x, result
