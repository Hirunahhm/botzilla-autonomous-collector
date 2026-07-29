# Ghost maps on hardware: mirrored LiDAR scan

**Status:** fixed and verified on hardware (Jetson Orin Nano + Kobuki + RPLIDAR C1 + Kinect v1).

## TL;DR

`/scan` was **mirrored**. The RPLIDAR reports its angle increasing **clockwise**; a ROS
`sensor_msgs/LaserScan` published with a positive `angle_increment` means bins increase
**counter-clockwise** (REP-103). `bin_scan_points()` binned the raw angle directly and
never negated it.

A mirrored scan is invisible while the robot is stationary — it just looks like a
plausible room. But when the robot turns by `Δ`, the mirrored pattern rotates by `+Δ`
instead of `-Δ`, so after TF applies the robot's own `+Δ` the walls sweep across the map
at **2Δ**. Every scan lands at a different wrong angle, so RTAB-Map draws the same wall
over and over at different orientations — the "ghost map".

One-line fix in `botzilla_perception/rplidar_driver.py`:

```python
# before
i = int(p.angle_deg / bin_width_deg) % num_samples
# after
i = int(math.floor(-p.angle_deg / bin_width_deg)) % num_samples
```

## Symptoms

- Map fine while the robot sat still.
- As soon as it rotated (Nav2 goal or manual `cmd_vel`), *"the lidar boundaries start to
  move with it, then the point cloud map of the depth, then ghost map."*
- Occupancy grid filled with a rotational "fan" of duplicated room outlines, with wall
  segments cutting straight through open space.
- The robot's URDF appeared to flip/rotate in RViz.

The phrase **"boundaries move *with* the robot"** turned out to be the single most
diagnostic observation, and it points directly at the sign error: a correctly-handed
scan stays fixed in the map; a mirrored one sweeps in the same direction as the turn.

## Root cause in detail

`rplidar_node.py` publishes:

```python
msg.angle_min = 0.0
msg.angle_max = 2.0 * math.pi
msg.angle_increment = 2.0 * math.pi / NUM_SAMPLES     # POSITIVE => counter-clockwise
```

So a consumer reads bin `i` as bearing `+i * 0.9°` CCW. But `bin_scan_points()` computed
`i` straight from `p.angle_deg`, which the RPLIDAR measures clockwise. Result: every
point was placed at the mirror of its true bearing.

Why it survived so long:

| robot state | mirrored scan looks like | noticeable? |
|---|---|---|
| stationary | a mirror image of the room, static | no — plausible room |
| rotating by `Δ` | pattern rotates `+Δ` instead of `-Δ`; after TF adds `+Δ`, walls move `2Δ` | yes — everything smears |

## Evidence

All of this was measured, not inferred. The decisive test was: transform two scans of the
same static room, taken at different yaws, into the `odom` frame, and check whether they
coincide. **They must**, if TF + scan angles + timestamps all agree.

1. **Visual overlay** — two scans ~30° apart, both transformed to `odom`, plotted in two
   colours. The outlines were clearly rotated relative to each other, while the two lidar
   origins coincided (so: rotation error, not a mounting/translation error).

2. **Numerical residual** — brute-force search for the extra rotation still needed to
   align cloud B onto cloud A after the TF transform:

   ```
     dyaw(tf)   residual     interpretation
         20.2      -42.0     ~= -2 x dyaw   -> sign-flipped scan angle
   ```

   `residual ≈ 0` would mean correct; `residual ≈ -dyaw` would mean no compensation at
   all; `residual ≈ -2·dyaw` is specifically a **sign flip**.

3. **After the fix**, same measurement:

   ```
     dyaw(tf)   residual     interpretation
        -20.5       -2.0     ALIGNED (ok)     (-2.0 is the 2 deg search step)
   ```

4. **Map** — an in-place rotation in a known room went from a fan of superimposed rotated
   outlines to a single crisp rectangle, with the grid barely changing size
   (170×123 → 170×124 cells), which is what an in-place rotation should produce.

## Diagnostic method (reusable)

The generic technique for "is my scan geometry right?":

> Record `(TF yaw, scan)` pairs while the robot rotates. Transform each scan into `odom`
> using TF **at the scan's own timestamp**. Pick two scans separated by 20–40° of yaw and
> search for the rigid rotation that best aligns them. For a static room the answer must
> be ~0.

Use `botzilla_control`'s `rotation_test_node` to drive the rotation — it is bounded
(fixed duration, explicit stop, self-shutdown), so it cannot run away:

```bash
ros2 run botzilla_control rotation_test_node                              # ~137 deg CCW
ros2 run botzilla_control rotation_test_node --ros-args -p angular_speed:=-0.3   # reverse (unwind cables)
```

**Important:** do not touch or reposition the robot during a measurement. Pushing it by
hand turns the wheels not at all and produces exactly the same signature as this bug
(room moves, odometry doesn't) — that contaminated one run during this investigation.

## Secondary bugs found and fixed along the way

These were all real and are all fixed, but none of them were the ghosting root cause.

1. **Publish-time timestamping** (`rplidar_node.py`, `kinect_bridge.py`)
   Messages were stamped in the publish timer instead of at capture, so each scan claimed
   to be ~115ms newer than its own data and consumers resolved TF at the wrong moment.
   Stamp age at arrival, before → after: scan `2.1ms → 113.7ms`, depth `→ 20.1ms`,
   rgb `→ 15.2ms`.

2. **Silent empty scans** (`rplidar_node.py`)
   Attaching to a device left mid-stream (e.g. after `kill -9`) desynchronises the byte
   reader; every point parses invalid, `bin_scan_points()` filters all of them, and the
   node published a 400-bin all-`inf` scan **forever** with no error and no reconnect.
   Measured 0/400 finite bins while the device itself was perfectly healthy (19 802
   points, all valid, 0.4–6.8 m). Added `EMPTY_SCAN_LIMIT` — a streak of empty
   revolutions now raises and forces a reconnect (`connect()` sends `CMD_STOP` + flushes,
   which resyncs). After a clean reconnect: 364/400 finite bins.

3. **Silent hang in `iter_scans()`** (`rplidar_driver.py`)
   `if len(packet) < 5: continue` spun on read timeouts indefinitely — no data, no
   exception, so the caller's reconnect path never ran and `/scan` simply never appeared.
   Added `DATA_TIMEOUT_S`.

4. **Gyro byte decoding** (`KobukiDriver.py`)
   `gyro_velocity_data()` scaled the low and high bytes of each 16-bit sample
   *independently* instead of combining them into one signed value. Extracted as
   `gyro_bytes_to_signed_int16()` with unit tests.

5. **`/odom` twist never populated** (`kobuki_base_node.py`)
   `ekf_hardware.yaml` fuses **only** velocities from wheel odometry, but the message's
   twist fields were left at their `0.0` defaults — so the EKF fused "velocity = 0" on
   every update and `/odometry/filtered` stayed pinned at the origin regardless of real
   motion. Now computes `d/dt` and `dtheta/dt` from encoder deltas over real elapsed time.

6. **Gyro bias tracker corrupted during turns** (`kobuki_base_node.py`)
   The stationary-bias EMA was gated only on wheel-encoder motion. Skid-steering during an
   in-place turn makes individual 20 ms ticks read as near-zero, so the tracker absorbed
   the real turn rate into "bias" and subtracted it back out. Now also requires the last
   `/cmd_vel` to be zero.

## Hypotheses that were wrong

Recorded so nobody re-chases them. Each was disproved by measurement.

| hypothesis | how it was ruled out |
|---|---|
| EKF yaw lags / undershoots | gyro integral, wheel `/odom`, `/odometry/filtered` and TF all agreed within ~2° through a turn |
| ICP bounds too tight (`Icp/MaxRotation`, `CorrespondenceRatio`) | loosening them let a *falsely confident* spurious correction into the graph (odometry said 0.0002 m, ICP claimed 0.163 m at variance 0.00016) — strictly worse. Reverted to defaults |
| RTAB-Map detection rate too low | raising 1→2 Hz is a mild genuine improvement and was kept, but ghosting persisted at 2 Hz |
| depth camera painting the floor as obstacle | ran with `Grid/Sensor:=0` (lidar-only grid) — still ghosted |
| gyro scale factor ~9% off | measured scale factor **0.9989** across 50 sample pairs; the single-pair 66.6°-vs-73.2° reading that suggested 9% was noise (per-pair ratios scatter 0.76–1.11) |

The general lesson: every one of those was a plausible *parameter* explanation, and each
cost a round of tuning. The bug was in the **data geometry**, and only a direct geometric
measurement found it. Measure the invariant ("two scans of a static room must coincide in
`odom`") before touching tuning parameters.

## Regression test

`botzilla_perception/test/test_rplidar_driver.py::test_bin_scan_points_is_not_mirrored`
pins the handedness with a 360-bin scan (1° per bin, so bin index == bearing in degrees):

```python
# RPLIDAR 90 deg (clockwise) is physically to the robot's RIGHT,
# which in ROS is bearing -90 deg == 270 deg CCW.
ranges, _ = bin_scan_points([_point(90.0, 1000.0)], num_samples=360, ...)
assert ranges[270] == 1.0
assert ranges[90] == float('inf')   # binning here would mean a mirrored scan
```

Run with:

```bash
cd botzilla_Workspace
python3 -m pytest src/botzilla_perception/test/test_rplidar_driver.py -q
```

## Related configuration

`botzilla_navigation/launch/rtabmap.launch.py` gained a `grid_sensor` launch argument
(`0`=laser only, `1`=depth only, `2`=both, default `2`) so the depth camera can be
isolated from the occupancy grid during diagnosis without editing the file:

```bash
ros2 launch botzilla_navigation rtabmap.launch.py \
  use_sim_time:=false depth_topic:=/camera/depth/image_meters grid_sensor:=0
```

`rtabmap_debug.launch.py` is the same stack with RTAB-Map's `--udebug` logging and
`log_to_rosout_level:=0`, for when per-frame registration detail is needed.
