# PID Longitudinal Controller (FormulaSlug Auto Onboarding)

One or two sentences: a PID controller that drives a simulated 1D car to a
target velocity, with anti-windup and velocity-scheduled gains.

## Quick start
```bash
python -m venv auto_venv && source auto_venv/bin/activate
pip install numpy matplotlib
python run_template.py
```

## Project layout
| File | Purpose |
|---|---|
| car.py | 1D car model (state + physics update) |
| pid_controller.py | Basic PID + throttle conversion/saturation |
| interpolated_pid.py | Gain-scheduled PID (GainPoint table) |
| run_template.py | Runs the simulation and plots the results |
| starter/, class-versions/ | Original onboarding templates |

## How it works
The car object contains the state of the car alongside the desired velocity. The desired velocity is passed to a PID controller object, which then computes a desired_acceleration, which is then converted to throttle and clipped. The clipped throttle is passed to the car's update method, completing one time step.

## Features / design decisions
- Back-calculation anti-windup: At high desired_v, the integral would keep increasing even though the output was saturated. This caused a massive overshoot, growing as the desired_v grew. Back-calculation adds the difference between the clipped and the raw output, lowering the net_integral when the car is going at max speed. K_B determines the strength of the unwinding.
- Interpolated gains: When the environment around the car varies, i.e. friction/drag changes with speed, adding multiple gains and interpolating them allows for the car to respond better to different environments. In this simulation, which has constant friction, it didn't add much functionality. 

## Tuning
### For fixed PID
| Gain | Reacts to | Too low | Too high |
|---|---|---|---|
| K_P | Current error | Slow rise; settles ~2/K_P m/s below target (friction) | Throttle saturates; oscillation |
| K_I | Accumulated error | Steady-state error from friction lingers | Overshoot; slow oscillation |
| K_D | Rate of change of error | Overshoot not damped | Sluggish / noisy response |
| K_B | Saturated − raw output | Integral windup → large overshoot at high desired_v | Integral unwinds too aggressively, slow to settle |

Final values: K_P = 1, K_I = 0.1, K_D = 0.5, K_B = 0.1

### For Gain-Scheduled PID
Gains are defined as a `gain_table` of `GainPoint`s in `run_template.py`:

```python
gain_table: list[GainPoint] = [
    GainPoint(velocity=0.0,  K_P=1.5, K_I=0.05, K_D=0.2, K_B=0.04),
    GainPoint(velocity=30.0, K_P=1.0, K_I=0.10, K_D=0.3, K_B=0.05),
    GainPoint(velocity=60.0, K_P=2.0, K_I=0.15, K_D=0.4, K_B=0.3),
]
```

Each step, every gain is linearly interpolated (`np.interp`) using the car's **current** velocity. For example, at 15 m/s, K_P = 1.25.

**Rules**
- Velocities must be strictly increasing, and all gains must be non-negative (otherwise a `ValueError` is raised).
- A single `GainPoint` behaves like the fixed PID.
