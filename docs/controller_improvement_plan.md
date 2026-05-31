# Controller Improvement Plan

This document captures **salvageable ideas** from the experimental Modern-Robotics-style torque controller stack (removed from the repo) and proposes **concrete fixes** for the production procedure-based controller ahead of the conference demo.

The full MR rework design remains in `docs/control_system_rework_plan.md` for reference. That approach was **not integrated** into the climb pipeline and did not improve limb motion or jump success in practice. The recommendations below stay within the existing `ControlSignal` + `procedure.py` architecture unless noted otherwise.

---

## What the MR experiment taught us

### Keep as concepts (do not re-add the full stack yet)

1. **Trapezoid motion profiles** (`TrapezoidProfile`)
   - Pre-compute accel / cruise / decel segments for a scalar displacement (angle or extension).
   - Guarantees single smooth speed-up and slow-down instead of reactive bang-bang heuristics.
   - **Port target:** replace `SmoothAdjustAngle` rate logic and constant-speed `AdjustLength` with profile-based setpoints sampled each timestep.

2. **Feedforward + feedback joint tracking**
   - `JointTarget` carried desired angle, rate, and acceleration — PD only on error, with optional computed-torque feedforward.
   - **Port target:** in the velocity-servo layer, command `(desired_rate, desired_accel)` derived from a profile; let `RotationMotor` track rate with a consistent PD instead of ad hoc blending.

3. **Body pose from fixed contacts (stance IK)**
   - `BodyStanceIKController` solved for body SE(2) and limb `(α, l)` given gripped world points.
   - **Port target:** use closed-form 2D geometry inside `MoveCenter` so all active legs move coherently toward a body goal instead of per-leg sign logic fighting constraints.

4. **Contact wrench distribution**
   - When multiple feet grip, distribute a desired body wrench across contacts via a small linear solve.
   - **Port target:** optional later step for body moves; explains why changing one leg length while others are fixed causes constraint fighting and apparent “jerky” motion.

5. **Foot endpoint impedance**
   - World-frame PD on free-foot position mapped through `J_footᵀ`.
   - **Port target:** for `Tracker` / foot repositioning, treat the foot as tracking a moving world target with damped approach rather than decoupled angle + length bang-bang.

6. **Grip-based state estimation**
   - Infer body pose from known grip locations and limb encoders.
   - **Port target:** use for diagnostics and for aligning planned jump takeoff state with simulated state before release.

### Do not repeat (lessons from the failed integration)

- Replacing the entire procedure pipeline with torque controllers **without** jump support, UPF integration, or tuned Pymunk actuator limits.
- Running joint-space PID against stiff leg springs without coordinated body/stance logic — oscillation and timeout failures dominated batch tests.
- Tuning dozens of PID weights (Optuna batches) before fixing the root cause: **open-loop rate commands into a constrained multibody system**.

---

## Problem 1: Jerky limb angle adjustment (~26% unrelated but visible in demo)

### Root causes in the current stack

| Layer | Issue |
|-------|--------|
| `AdjustAngle` | Bang-bang ±`rotation_speed`; no deceleration planning. |
| `SmoothAdjustAngle` | Heuristic accel limit, but deceleration uses `rotation_speed * dt` (wrong units vs `acceleration_rate`); competes with `AdjustLength` in `Tracker`. |
| `MoveCenter` | Uses plain `AdjustAngle` with `acceleration_rate = 0`; moving target point changes desired angle every step → repeated speed reversals. |
| `RotationMotor` | Low-pass blend `0.9*real + 0.1*desired` fights the controller; body rotation from other legs appears as leg rate noise. |
| `Tracker` | Angle and extension adjusted independently toward a moving target; Pymunk constraints cause coupled motion not modeled by procedures. |

### Recommended fixes (priority order)

#### Phase A — Motion profiles in procedures (highest impact, low risk)

1. Add `monkey_bot/motion_profiles.py` with `TrapezoidProfile` (copied concept from removed stack; ~80 lines, no MR dependencies).
2. Replace `SmoothAdjustAngle` with **profile-driven angle motion**:
   - On init: compute total angle displacement to target; build trapezoid with `max_speed = rotation_speed`, `max_accel` from config (new `RobotConfig.rotation_acceleration`).
   - Each step: sample profile at elapsed time → set `signal.rotation[i] = profile.velocity`.
3. Same pattern for `AdjustLength` (extension profile).
4. In `Tracker`, run **sequential** or **dominant-DOF** motion: e.g. rotate until within tolerance, then extend, or use a single profile on foot arc-length in body frame.

#### Phase B — Simulator tracking

1. Change `RotationMotor.step` to a consistent velocity servo: `rate = Kp*(desired - actual)` with clamp, or critically damped first-order lag using `acceleration_rate`.
2. Optionally reduce `leg_spring_stiffness` during rotation-only phases (config flag) to reduce constraint fighting.

#### Phase C — MoveCenter coherence

1. Compute desired body displacement vector; for each active leg, set foot target = `body_goal + R(θ) * limb_offset` instead of chasing a moving center sample.
2. Use `SmoothAdjustAngle` (profile version) for all legs in `MoveCenter`, not zero-acceleration `AdjustAngle`.

#### Phase D — Validation

- Re-run `experiments/limb_angle_sweep.py` and `experiments/experiment_metrics.py`; success = monotonic \|ω\| envelope, no sign flips in rate log.
- Use `RotationMotor` log → `experiments/results/logs/adjust_angle_data.txt` (see restructured paths).

---

## Problem 2: Jump success ~26%

### Root causes

| Layer | Issue |
|-------|--------|
| Geometric precompute | `GeometricLauncher` assumes point-mass ballistic center; ignores leg inertia, spring release, and simultaneous foot detach. |
| Takeoff execution | `MoveCenter` + `ReleaseAtPoint` approximate takeoff speed via center motion, not impulse at release. |
| Catch phase | `DynamicCatch` greedy nearest-grip assignment; timing window from geometry may not match simulated flight time after drag/constraints. |
| Sim vs plan drift | No verification that simulated state at release matches `(start, jump_vector)` stored in `cache/transition_links/`. |
| Batch metric | `actual_sim_batch.py` may count failures from controller timeout, catch miss, and plan infeasibility together — separate metrics needed. |

### Recommended fixes (priority order)

#### Phase 1 — Execution fidelity

1. **Takeoff impulse:** at release, apply explicit center velocity impulse matching `jump_vector` (Pymunk `body.velocity` set once all jumping legs released), instead of relying only on runway `MoveCenter` speed.
2. **Runway alignment:** before jump sequence, verify both jumping feet are within `epsilon` of grid anchor and center is within runway segment; abort/replan if not.
3. **Synchronized release:** release all non-support legs first; hold jumping legs until center crosses takeoff plane, then release together.

#### Phase 2 — Catch robustness

1. Widen catch assignment: solve bipartite matching (feet → landing grips) by minimum distance, not greedy order-dependent assignment.
2. During flight, command `Tracker` toward predicted landing grips using **predicted center from ballistic integrator**, not current center.
3. Increase catch tolerance dynamically with vertical velocity (more time near grips).

#### Phase 3 — Precompute validation

1. After computing each transition link, **roll forward a short headless sim** (no render) and store success bit in cache metadata.
2. Prune links that fail sim validation even if geometry passes (`prune_short_jumps` etc. stay, add `prune_sim_failed`).
3. Log failure mode histogram: `{missed_runway, wrong_speed, catch_timeout, attach_fail}`.

#### Phase 4 — Tuning knobs to sweep

- `max_takeoff_speed`, runway length fraction, `leg_spring_stiffness` at release
- `epsilon` for catch vs attach
- Gravity scale consistency between `GeometricLauncher` and Pymunk space

---

## Suggested implementation order for conference deadline

1. **Trapezoid profiles** in `AdjustAngle` / `AdjustLength` (1–2 days).
2. **RotationMotor velocity servo** cleanup (hours).
3. **Jump takeoff impulse** + catch matching (1–2 days).
4. **Headless sim validation** for transition links when cache is rebuilt (1 day).
5. Defer full MR torque stack until post-demo unless Phase A–3 fail.

---

## Repository layout (after restructure)

```
cache/
  plans/              # UPF plan cache (one file per instance)
  transition_links/   # Jump launch instruction cache
experiments/
  *.py                # Batch runners and analysis scripts
  results/            # CSV, PNG, metrics from experiments (gitignored)
tests/                # Unit and integration tests only
monkey_bot/           # Core planner, sim, procedure controller
```

---

## Metrics to track before/after

| Metric | Source | Target |
|--------|--------|--------|
| Angle rate sign changes per move | rotation motor log | ≤ 2 (accel + decel) |
| Foot settle time | experiment_metrics | ↓ 30% |
| Jump success rate | actual_sim_batch | > 80% for demo instances |
| Catch attach rate | jump_runs.csv | > 90% when airborne trajectory valid |
| Plan + cache hit rate | sim_runner | unchanged |
