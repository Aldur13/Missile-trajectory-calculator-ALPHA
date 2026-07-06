# Missile Trajectory Calculator (ALPHA)

Reconstructs a missile's 3D flight trajectory from photos taken at two
different moments in flight, then fits a projectile motion model (gravity +
quadratic air drag) to predict apogee, impact point, and time of flight.

## How it works

1. **Getting a 3D position at each moment.**
   - **Single camera (default):** click the missile's pixel in one photo at
     each of the two moments. Since one camera can't recover true depth from
     a single view, this assumes the missile stayed at a roughly constant
     distance from the camera — an approximation, not a measurement, but
     usable when you only have one vantage point.
   - **Stereo (optional, more accurate):** with a second, calibrated camera,
     each moment gets a synchronized photo pair, and the missile's pixel in
     both photos triangulates its true 3D position — no distance assumption
     needed.
2. **Trajectory fit.** Given the two 3D points, the time gap between them,
   and known physical parameters of the missile (mass, cross-sectional area,
   drag coefficient, air density), a shooting-method solver finds the
   initial velocity vector at `t1` that reaches the observed point at `t2`.
   That velocity is then integrated forward (RK4) to produce the full
   trajectory, apogee, and impact point.

**Note:** with drag included, two observed points are not enough to solve for
*both* velocity and drag coefficient — you supply the drag parameters, and
the solver only searches over the initial velocity vector.

## Setup

```
pip install -r requirements.txt
```

## GUI (default)

Run with no arguments (or just double-click the packaged `.exe`) to open the
GUI:

```
python main.py
```

Load a photo for Point 1 and Point 2, click the missile's position in each,
and press Calculate — any parameter left blank falls back to a reasonable
default (listed in the output under "Assumptions used"). Check "Advanced: I
have a second camera + calibration file" to switch to accurate stereo
triangulation instead of the single-camera approximation.

## Camera calibration (stereo mode only)

Copy `config/calibration.example.yaml` and fill in your own two cameras:

- `position`: camera center in a shared world frame (meters), X/Y horizontal, Z up, ground at Z=0.
- `look_at` + `up`: aim direction (simplest way to specify orientation), or supply a `rotation` matrix directly if you already have one.
- `fx/fy/cx/cy`: pixel intrinsics, ideally from a checkerboard calibration (`cv2.calibrateCamera`). `trajcalc.calibration.intrinsics_from_specs()` gives a rougher estimate from focal length + sensor size if that's all you have.
- `dist_coeffs`: lens distortion `[k1, k2, p1, p2, k3]`, default zero.

## CLI (scripting / stereo without the GUI)

```
python main.py \
  --calibration config/calibration.yaml \
  --img-a-t1 photos/camA_t1.jpg --img-b-t1 photos/camB_t1.jpg \
  --img-a-t2 photos/camA_t2.jpg --img-b-t2 photos/camB_t2.jpg \
  --dt 0.8 \
  --mass 12.0 --area 0.05 --drag-coeff 0.3 \
  --plot-output trajectory.png
```

Each photo opens in a click-to-mark window (Enter to confirm, Esc to cancel).
To skip the GUI (e.g. for scripting/testing), pass pixel coordinates directly:
`--pixel-a-t1 "1200,640"` etc. If a coordinate is negative, use the `=` form
(`--pixel-a-t1="-40.5,640"`) — otherwise argparse mistakes it for a flag.

Useful flags:
- `--ground-z`: elevation of the ground/impact plane (default 0).
- `--overlay-camera A|B` + `--overlay-output path.png`: draws the fitted
  trajectory back onto that camera's t1 photo, as a sanity check.

## Running tests

```
pytest
```

Tests cover the RK4 integrator against closed-form projectile motion,
the shooting-method solver recovering known velocities (with and without
drag), and stereo triangulation recovering a known synthetic 3D point.

## Limitations

- Accuracy depends heavily on pixel-picking precision, inter-camera time
  sync, and calibration accuracy — these errors compound.
- Weak stereo geometry (narrow baseline/angle between the two cameras
  relative to the missile) degrades triangulation; the CLI prints a
  ray-separation quality check for each observed point — large values mean
  don't trust the result.
- Drag coefficient, air density, and cross-sectional area are usually the
  least-known inputs in practice and directly bias the fitted velocity.
- The physics model assumes constant mass, drag coefficient, and air density
  over the flight (no motor thrust, no altitude-varying atmosphere).
