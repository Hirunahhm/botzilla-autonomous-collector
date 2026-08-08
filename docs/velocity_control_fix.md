# Velocity control: deadband compensation and command smoothing

Fixes visible jerking during Nav2 navigation on the physical robot, and removes the
mechanism behind a long-standing "robot never makes progress toward its goal" failure.
Both turned out to be the **same** bug.

Everything below was measured on the real robot (Jetson + Kobuki base + rate gyro), not
inferred. Where a number appears, it came from an instrumented run.

---

## 1. Symptom

Two complaints that looked unrelated:

1. **Jerking.** The robot visibly snapped between motions instead of turning smoothly.
2. **Goals never completed.** Nav2's planner returned healthy 123–161-waypoint paths, the
   explorer dispatched them, and 32 s later the robot's own stall watchdog cancelled with
   no progress. Eleven consecutive frontier targets failed this way; `controller_server`
   independently logged `Failed to make progress`.

Measured during navigation: **1.888 m travelled in 80 s** (0.024 m/s average against a
0.2 m/s limit) with **24 angular sign flips**. The robot was rotating back and forth far
more than it was driving.

## 2. Root cause

There was **no smoothing anywhere in the velocity chain**, and one parameter made gentle
rotation impossible.

### 2a. DWB's default generator does not bound the emitted command

`nav2_params.yaml` set no `trajectory_generator_name`, so DWB used its default,
`dwb_plugins::StandardTrajectoryGenerator`. That generator samples the **entire**
configured velocity range every control cycle, independent of the robot's current
velocity. `acc_lim_x` / `acc_lim_theta` only shape the *simulated* trajectory — they never
bound the command that gets published.

### 2b. The base driver was a stateless passthrough

`kobuki_base_node.cmd_vel_callback` converted Twist → wheel mm/s and wrote straight to
hardware. No rate limiting, no memory of the previous command. It also had **no watchdog**:
the last command stayed latched in the motors forever, so if the publisher died mid-drive
the robot kept going.

### 2c. The velocity floor forbade gentle rotation

`min_speed_theta: 0.2` existed for a good reason — see §3 — but DWB rejects a trajectory
only when translation < `min_speed_xy` **AND** |rotation| < `min_speed_theta`. With the
floor at 0.2, the smallest legal in-place rotation *was* 0.2 rad/s. The reachable command
set was effectively:

```
{0, ±0.2 … ±0.4}
```

The robot could not ask for a small heading correction. Every correction was a step input,
it overshot, the error flipped sign, and it stepped back the other way — bang-bang control.
Combined with 2a and 2b, `cmd_vel` could legally jump `0 → +0.4 → −0.4` on consecutive
20 Hz cycles.

**That is both the jerking and the oscillation-instead-of-converging.** One bug.

The same self-inflicted limit cycle had already been confirmed in simulation earlier:
568 commands alternating ±0.232 rad/s, 54 sign changes, 0.000 m travelled over 25 s on
completely open floor.

## 3. Why the floor was there: a real mechanical deadband

The floor was not arbitrary. The base genuinely ignores small commands. Characterised
open-loop with all correction disabled (`ff`/`kp`/`ki` all 0), 5 s hold per point, first
2 s of transient discarded:

| commanded (rad/s) | 0.10 | 0.15 | 0.20 | 0.25 | 0.30 | 0.40 |
|---|---|---|---|---|---|---|
| **actual (rad/s)** | 0.001 | 0.023 | 0.076 | 0.120 | 0.152 | 0.251 |
| ratio | 1% | 15% | 38% | 48% | 51% | 63% |

Least-squares fit over the points that moved:

```
actual = 0.891 × (commanded − 0.121)        R² = 0.997
```

So: a **0.121 rad/s deadband**, and above it a near-unity slope. The model is essentially
exact for this unit. Below ~0.12 rad/s the robot does not turn at all, which is why
flooring DWB's output made it move — and why removing the floor naively would bring back
the original stall.

### The linear axis has a far smaller deadband

Measured the same way, using short forward bursts with reverse returns to stay inside a
~1.5 m corridor (3.6 s per point, first 1.2 s discarded):

| commanded (m/s) | 0.05 | 0.08 | 0.10 | 0.13 | 0.16 | 0.20 |
|---|---|---|---|---|---|---|
| **actual (m/s)** | 0.033 | 0.056 | 0.082 | 0.101 | 0.126 | 0.157 |
| ratio | 65% | 70% | 82% | 77% | 79% | 78% |

```
actual = 0.830 × (commanded − 0.009)        R² = 0.995
```

**0.009 m/s** — an order of magnitude smaller than the yaw deadband, and physically
sensible: driving forward turns both wheels the same way, whereas rotating in place must
break stiction on both wheels in *opposite* directions.

This corrected a bad guess. These values were previously **estimated** at deadband 0.04 /
gain 0.90. The 0.04 offset was 4.4× too large, so the feedforward **over-commanded by ~38%
at low speed**, producing a surge-then-settle cycle. That was directly observable: with
yaw measured and linear estimated, rotation felt smooth while forward/reverse still had
noticeable jerk — a clean natural experiment separating measured from guessed parameters.

## 4. The fix

A deadband is a **static nonlinearity**. The correct primary correction is a static inverse
applied *feedforward*, not integral feedback.

### 4a. Feedforward deadband inversion in `kobuki_base_node.py`

Inverting the fitted curve — to deliver setpoint `s`, command `deadband + |s| / gain`:

```python
def _feedforward(setpoint, deadband, gain):
    if setpoint == 0.0 or gain <= 0.0:
        return setpoint          # never add the offset to a commanded stop
    sign = 1.0 if setpoint > 0.0 else -1.0
    return sign * (deadband + abs(setpoint) / gain)
```

`cmd_vel_callback` now only stores the setpoint; a **50 Hz control timer** does the
hardware write, so correction continues between and after `cmd_vel` messages. Yaw feedback
comes from the bias-calibrated rate gyro already published on `/imu`; forward speed from
the encoder-derived velocity already computed in `_odom_update`.

**Why not just PI?** It was tried first. PI alone reached 90% steady-state accuracy but
took **~1.8 s to rise**, and rippled over a 0.316 rad/s spread. DWB re-decides at 20 Hz, so
a base needing ~2 s to reach a commanded rate cannot track it — that lag is *itself* a
source of Nav2-level oscillation. Feedforward cut rise time to ~0.4 s.

Small PI terms are retained to trim what the static model misses. They are kept small on
purpose — see §5 on the noise floor.

### 4b. Safety watchdog

`CMD_VEL_TIMEOUT_S = 0.5`. If no `cmd_vel` arrives in that window the motors stop and the
integrators reset. This closes the latched-command hazard in 2b.

### 4c. `LimitedAccelGenerator`

```yaml
trajectory_generator_name: "dwb_plugins::LimitedAccelGenerator"
```

Restricts DWB's sampling to velocities reachable within one control period, making
acceleration continuity a property of the controller rather than something bolted on
downstream. `acc_lim_theta` raised 0.8 → 1.5, because these limits stop being advisory once
this generator is selected: at 20 Hz, 0.8 allows only 0.04 rad/s of change per cycle, which
needs a full 0.5 s to reach `max_vel_theta` and left the sampling window too narrow for DWB
to consider meaningfully different options.

### 4d. Velocity floors back to zero

```yaml
min_speed_xy: 0.0        # was 0.05
min_speed_theta: 0.0     # was 0.2
```

Safe now that the base delivers what it is asked for.

### 4e. `velocity_smoother.py` (new node)

`cmd_vel_nav` → acceleration limiting → `cmd_vel`. This covers what neither 4a nor 4c can:
**`behavior_server`**. `BackUp` and `Spin` publish their own velocity commands directly,
bypassing DWB and its trajectory generator entirely — and recovery behaviours were the most
visibly violent part of the motion, occurring exactly when the robot is already close to
something.

A smoother (`nav2_velocity_smoother`) had been in this chain before and was removed because
on hardware it **silently stopped republishing**: `controller_server` kept publishing to
`cmd_vel_nav` while `/cmd_vel` had no publisher at all, the node reporting lifecycle state
active and logging nothing. The invisibility was the real problem. This node is built so
that failure cannot be silent:

- Publishes on a **timer**, unconditionally, so `/cmd_vel` always has a live publisher.
- Input watchdog **decays to zero** instead of latching, logging the transition once.
- Periodic diagnostic logs input/output rates, so a stalled stream is visible in the log.
- Not a lifecycle node — it must transport velocity before and after managed nodes
  transition, and adding it to `lifecycle_nodes` would make the whole bringup fail if
  absent.

One subtlety worth recording, because the first implementation got it wrong: accel and
decel are different limits, and which applies depends on whether **this step** grows or
shrinks the magnitude — not on whether the target's magnitude is larger. Deciding from
`abs(target) > abs(current)` calls a full reversal "speeding up" and applies the gentler
accel limit to the slowing phase, making a commanded *reversal* take longer to bite than a
commanded *stop* — backwards, since a reversal is usually the more urgent. Correct test:

```python
speeding_up = True if current == 0.0 else (delta * current) > 0.0
```

### 4f. Feedback filtering — the encoder polling artifact

The encoder-derived velocity (`d/dt` in `_odom_update`) is a poor control signal. We poll
the Kobuki at 50 Hz but its encoder registers refresh more slowly, so many polls return an
**unchanged** tick count (`d = 0`, velocity 0), and then one poll catches the whole
accumulated movement over a very short `dt` and `d/dt` explodes.

Measured while driving at a commanded 0.05 m/s:

```
median 0.000 m/s     mean 0.036 m/s     peaks to 0.35 m/s
/odom inter-arrival: median 19.9 ms, minimum 0.64 ms (16 of 259 intervals under 5 ms)
```

The signal is **bimodal** — runs of zeros punctuated by spikes — rather than noisy about
the truth. Its mean is correct, so it is unbiased and a low-pass filter recovers the real
value. Unfiltered, those spikes land straight in the P term and the integrator.

Fix: discard samples with `dt < MIN_ODOM_DT_S` (0.005 s — such a sample is a polling
artifact, not a measurement), then EMA the rest with `MEAS_FILTER_ALPHA = 0.25`
(~0.12 s time constant at 50 Hz). Applied to the yaw feedback too, where it suppresses
genuine drivetrain judder rather than a polling artifact.

**Applied only to the controller's copy.** The values published on `/odom` are left raw:
`robot_localization` has covariances tuned against that raw signal (`ekf_hardware.yaml`),
and silently changing the statistics of a topic other nodes consume would be a far wider
change than this fix warrants.

## 5. Results

Delivered yaw rate, before vs after:

| commanded | before | after |
|---|---|---|
| 0.10 rad/s | **~1%** (no motion) | **88.8%** |
| 0.20 rad/s | 38% | 111% |
| 0.30 rad/s | 51% | 106% |
| 0.40 rad/s | 63% | 105% |

- Rise time: **1.8 s → ~0.4 s**
- Ripple stdev: **0.084 → 0.032–0.056**
- Largest single-step change on `/cmd_vel` during a full-scale reversal: **0.125 rad/s**,
  exactly the decel bound, versus **0.8** before — a **6.4× reduction** in worst-case jerk.
- Both watchdogs (smoother and base) verified firing and releasing correctly.

Residual overshoot is ~5–11%, judged acceptable against the previous 46% shortfall.

Delivered **linear** speed, cross-checked against ground truth (distance ÷ time, so it
does not depend on the noisy instantaneous velocity signal at all):

| commanded | before | after (odom avg) | after (ground truth) |
|---|---|---|---|
| 0.05 m/s | 65% | 106.3% | **96%** |
| 0.10 m/s | 82% | 100.3% | **98%** |
| 0.16 m/s | 79% | 100.2% | **99%** |
| 0.20 m/s | 78% | 102.8% | **99%** |

Note the `stdev` of the *published* `/odom` velocity stays high (0.096–0.381) after this
change — as designed. The filter is applied to the controller's copy only, so a
measurement that reads the raw topic sees the unchanged spiky signal. Ground truth is the
honest check, and it says 96–99%.

### The remaining ripple is mechanical, not controller-induced

Worth recording so nobody wastes time tuning it away. Open loop at a commanded 0.345 rad/s
(delivering ~0.172), with every correction term zeroed:

```
mean 0.172   stdev 0.084   spread 0.349   (briefly reverses sign)
```

Closed loop at the same delivered rate had spread **0.298** — *better* than open loop. The
ripple is stick-slip in the drivetrain at low speed, faithfully reported by the gyro. No
gain schedule removes it, and large feedback gains would only inject it into the motor
command. This is why the PI terms are deliberately small.

## 6. Files changed

| File | Change |
|---|---|
| `botzilla_control/kobuki_base_node.py` | Feedforward deadband inversion + PI trim, 50 Hz control timer, cmd_vel watchdog |
| `botzilla_control/velocity_smoother.py` | **New** — accel-limits `cmd_vel_nav` → `cmd_vel` |
| `botzilla_control/setup.py` | `velocity_smoother` entry point |
| `botzilla_navigation/config/nav2_params.yaml` | Floors → 0, `LimitedAccelGenerator`, `acc_lim_theta` 0.8→1.5 |
| `botzilla_navigation/config/nav2_params_sim.yaml` | Now a documented no-op (see below) |
| `botzilla_navigation/launch/nav2.launch.py` | Remap velocity publishers to `cmd_vel_nav`, launch smoother |

`cmd_vel` chain, after:

```
controller_server ─┐
                   ├─> cmd_vel_nav ─> velocity_smoother ─> cmd_vel ─> kobuki_base_node
behavior_server  ──┘                                                   (closed loop)
```

### The sim overlay became redundant

`nav2_params_sim.yaml` existed solely to zero the velocity floors that hardware needed and
simulation did not. Both sides of that conflict are gone: the deadband is now compensated
where it actually lives, so the same floors (0.0) are correct on both platforms. The same
reasoning retired the other planned divergence — `max_vel_theta` was a candidate for a
hardware override because the open-loop base delivered only ~56% of commanded yaw rate;
closed-loop control corrects that directly. The file is retained as a documented no-op so
the overlay mechanism stays wired up and tested for the next genuine divergence.

## 7. Tuning

All gains are live ROS parameters — no rebuild needed:

```bash
ros2 param set /kobuki_base_node yaw_ff_gain 0.891
ros2 param set /kobuki_base_node yaw_ff_deadband 0.121
```

| parameter | default | source |
|---|---|---|
| `yaw_ff_deadband` | 0.121 | measured, R²=0.997 |
| `yaw_ff_gain` | 0.891 | measured, R²=0.997 |
| `lin_ff_deadband` | 0.009 | measured, R²=0.995 |
| `lin_ff_gain` | 0.830 | measured, R²=0.995 |
| `yaw_kp` / `yaw_ki` | 0.20 / 0.15 | trim only |
| `lin_kp` / `lin_ki` | 0.20 / 0.15 | trim only |

**To re-characterise** (do this per unit — these are physical measurements that drift as
the gearbox wears): zero `*_ff_gain`, `*_ff_deadband`, `*_kp`, `*_ki`, sweep commands
holding each for 5 s, discard the first 2 s, then least-squares fit
`actual = gain × (cmd − deadband)`.

## 8. Known gaps

1. **Goals still do not complete — this fix did not solve navigation.** Measured over a
   full explorer run under the new config: **0 goals succeeded, 12 stalled and were
   cancelled, 6 more had no valid path, 20 targets blacklisted.** The robot now *moves*
   (20.2 m in 150 s vs 1.9 m in 80 s before) but does not *arrive*. Path-relative
   measurement during that run:

   ```
   mean |bearing error| to own path : 96.9 deg
   path point BEHIND the robot      : 58% of samples
   reverse while path is AHEAD      : 55.7%
   command split                    : 349 forward / 1295 reverse
   LOCAL plan length                : mean 8.4 poses, min 0 (sometimes empty)
   ```

   **ROOT CAUSE FOUND — see §9.** An earlier revision of this section warned "do not fix
   this by clamping `min_vel_x`", on the grounds that it would mask the symptom. That
   warning was written before the mechanism was identified and is now withdrawn: the
   scoring gap turned out to be a genuine misconfiguration, and restricting reverse is
   part of the correct fix rather than a band-aid.

2. **Recovery behaviours are failing.** In the same run a `BackUp` aborted and the
   fallback `Spin` aborted too — consistent with the local costmap being tighter than it
   appears.
3. **Local costmap is lidar-only** (`observation_sources: scan`), so it cannot see anything
   below the lidar plane. Unrelated to this fix, but must be addressed with
   `pointcloud_to_laserscan` before cube collection.

---

## 9. Why the robot drove backwards to its goals

Separate root cause from the velocity work above, found after it. Worth recording because
the symptom (never reaching a goal) looked like a planner or SLAM problem and was neither.

### Only two critics care about heading, and both were disabled by a parameter

The active critic list is:

```yaml
critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
```

`PathAlign` and `GoalAlign` are the **only** two that consider which way the robot faces.
From nav2's own header (`dwb_critics/path_align.hpp`): *"this critic calculates how far a
point `forward_point_distance` in front of the robot is from the global path."*

`forward_point_distance` was set to **0.1 m**, against a `robot_radius` of **0.20 m**. The
projected point therefore sat *inside the robot's own footprint*, and rotating the robot a
full 180° moved it by only 0.2 m — a rounding error at 0.05 m costmap resolution.

So both heading-aware critics were effectively blind, while the two highest-weighted
critics scored pure distance:

| critic | scale | heading-aware? |
|---|---|---|
| `PathDist` | 32.0 | no — pure distance |
| `GoalDist` | 24.0 | no — pure distance |
| `PathAlign` | 32.0 | yes, but crippled at 0.1 m |
| `GoalAlign` | 24.0 | yes, but crippled at 0.1 m |

**Reversing toward a goal scored identically to driving forward toward it.** With
`min_vel_x: -0.10` making reverse legal, DWB had no reason to prefer forward. Measured
over a 150 s hardware run: **1295 reverse commands vs 349 forward**, mean bearing error to
its own path **96.9°**, path *behind* the robot 58% of the time, **0 of 12 goals reached**.

### Fix

```yaml
PathAlign.forward_point_distance: 0.325   # was 0.1 (0.325 is nav2's default)
GoalAlign.forward_point_distance: 0.325   # was 0.1
min_vel_x: 0.0                            # was -0.10
```

At 0.325 the projected point clears the footprint by ~0.12 m, so facing the wrong way
costs real score.

### Why simulation did not catch it

Both offending values were present in the simulation config that ran successfully. The
misconfiguration was **latent, not absent**: reversing in simulation works fine — perfect
odometry, no deadband, instant localisation — so a reverse-biased controller still reached
its goals. On hardware, reverse driving is unsensed (the depth camera faces forward, and
the local costmap is lidar-only), and every forward/reverse flip bleeds odometry accuracy
while RTAB-Map only corrects pose at ~1 Hz.

The lesson worth keeping: **simulation validates logic, not cost-function tuning.** A
critic weighting that merely fails to *prefer* the right behaviour is invisible in a world
forgiving enough that the wrong behaviour also succeeds.

### The `min_vel_x` history, corrected

`min_vel_x: -0.10` was originally introduced because a robot wedged against a wall had no
escape trajectory — recorded at the time as "the path-alignment critics kept pulling it
back toward the blocked heading". That is the *same* bug: the align critics could not see
heading, so they could not reward turning away from the obstruction. Allowing reverse
treated the symptom and, on hardware, made things considerably worse.

Reversing still exists where it belongs: `behavior_server`'s `BackUp` recovery, a
deliberate bounded escape, rather than a routine path-following option.
