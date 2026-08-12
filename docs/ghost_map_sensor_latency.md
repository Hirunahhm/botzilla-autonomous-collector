# Ghost maps on hardware, round 2: stale Kobuki sensor data

**Status:** fixed and verified on hardware (Jetson Orin Nano + Kobuki + RPLIDAR C1 + Kinect v1).

Sequel to `ghost_map_investigation.md`. That round found a **mirrored** scan; this one is a
different mechanism with the same visible symptom, found after ghost maps reappeared in two
separate mission runs. The mirrored-scan fix from round 1 is still in place and still correct —
it was explicitly re-verified here (see "Hypotheses that were wrong").

## TL;DR

`KobukiDriver.read_data()` read a **fixed 200 bytes** per packet when a Kobuki feedback packet
is **79 bytes** from its length byte. The serial port is opened with no timeout, so
`pyserial.read(n)` blocks until all `n` bytes arrive. That meant:

* every read **blocked ~49 ms** before returning, so the data was already that stale before
  parsing started; and
* `2 + 200 = 202` bytes is **2.56 packets**, so each read consumed two whole packets beyond the
  one it parsed and stopped 44 bytes into a third — mid-stream, forcing the 2-byte sync loop to
  hunt for the next header and throw the remainder away.

Net effect: **the gyro and encoders actually refreshed at ~20 Hz, not the 50 Hz the node
assumed**, and every sample was tens of milliseconds older than its own timestamp claimed.

`ekf_hardware.yaml` fuses the gyro **yaw rate and nothing else**, with no absolute yaw
reference anywhere in the system, so that staleness integrated straight into heading with
nothing able to correct it. Measured result: a fixed **~200 ms lag** between the laser data and
the pose estimate, i.e. **~3.4° of misregistration on every scan taken while turning** at
0.3 rad/s. RTAB-Map then found its odometry (`type=0`) edges disagreeing with its scan-matched
loop closures by 6–37°, exceeded `RGBD/OptimizeMaxError`, and **rejected every loop closure it
ever proposed** — so the map never self-corrected and accumulated ghost walls.

One-line core fix in `botzilla_control/KobukiDriver.py`:

```python
# before
__temp = Kobuki.seri.read(200)
# after
__temp = Kobuki.seri.read(PACKET_REMAINDER_BYTES)   # 77 — exactly the rest of the packet
```

## Symptoms

* Ghost/duplicate walls appearing in two separate mission runs, minutes apart.
* RTAB-Map log full of `Rejecting all added loop closures ... because a wrong loop closure has
  been detected after graph optimization`, always naming a **`type=0`** (odometry/neighbour)
  edge with a large angular error.
* **Zero** loop closures accepted across an entire 688-frame run — 34 wholesale rejections,
  63 individual ones, 0 accepted.
* The bad edges were not general drift: only **three** edges were ever flagged, and all three
  coincided with `TARGETING`, the in-place rotation phase.

```
edge 276->292, type=0, abs error=  5.6° -> 10.2°
edge 496->523, type=0, abs error= 14.0° -> 36.9°
edge 559->576, type=0, abs error= 21.6°
```

RTAB-Map assumes odometry yaw is good to ±1.81° (`stddev=0.031623` rad). Errors of 6–37° give an
error ratio of 3–16 against `RGBD/OptimizeMaxError: 3.0`, so **all** loop closures in that
iteration are discarded. Loop closures are the only thing that merges duplicate geometry, so
discarding them all is exactly "ghost map".

## Root cause in detail

Kobuki feedback framing is:

```
AA 55 <len> <len payload bytes> <checksum>
```

`read_data()` syncs by reading 2 bytes and testing `== 333`. `333 == 0x014D`, little-endian
`[0x4D, 0x01]` = `[77, 1]` — i.e. it matches `<len=77>` followed by sub-payload id `0x01`
(Basic Sensor Data). So the driver only ever accepts the 77-byte-payload variant, and having
consumed those two bytes exactly **77** remain: 76 further payload bytes plus the checksum.

Measured consequences of asking for 200 instead (115200 baud, 8N1, 50 Hz packets, 81 B/packet
= 4050 B/s):

| | `read(200)` | `read(77)` |
|---|---|---|
| bytes consumed from `<len>` | 202 | 79 |
| = packets | **2.56** | **1.00** |
| blocking wait | **49 ms** | 19 ms |
| lands on | 44 B into a later packet (needs resync hunt) | next `AA 55`, exactly |
| effective fresh-data rate | **~20 Hz** | 50 Hz |

The `read(200)` was not arbitrary — it was over-reading deliberately so a scan for the *next*
packet's `AA 55` header could populate `__general_purpose_input`. That field's only reader,
`general_purpose_input_data()`, is **called by nothing in this workspace**, so the scan was
removed along with the over-read.

## Evidence

All measured, not inferred. The core instrument is the reusable method from round 1: rotate in
place, transform each scan into `odom` using TF **at the scan's own timestamp**, and check
whether scans of the same static room coincide. Extended here with a *time-offset sweep* — for
each candidate offset `D`, re-transform each rotating scan using the pose interpolated at
`scan_stamp + D` and align it against a stationary reference scan. The `D` that minimises the
residual is how far the pose estimate lags the laser.

**1. There is a lag, and it is a lag in *time*, not an error in *angle*.**

| test | rate | best time offset | equivalent angle |
|---|---|---|---|
| forward 0.3 rad/s | 17.7°/s | **+186 ms** | 3.30° |
| reverse −0.3 rad/s | 17.9°/s | **+244 ms** | 4.36° |
| forward 0.15 rad/s | 10.6°/s | **+180 ms** | 1.91° |
| reverse −0.3, **full CPU load** | 17.9°/s | **+223 ms** | 3.99° |

Halving the speed **halves the angle but leaves the time unchanged** — the signature of a fixed
latency. A yaw *scale* error would grow the residual proportionally with `dyaw`; it did not
(residual was flat at ~4.5° across `dyaw` 16°→34°). A *sign flip* would give `-2×dyaw`; the
offset stayed positive in both directions.

**2. It is not the EKF.** Gyro-integrated yaw versus EKF-reported yaw agreed to **0.03°** over a
121° rotation. The EKF is faithfully integrating what it is given; what it is given is late.

**3. The direction is logically forced.** The lidar stamps a revolution when *Python sees* it
begin, which is necessarily *after* physical capture — so a scan can only ever be stamped late,
never early. A **positive** required offset therefore means the pose lags, not that the scan
leads.

**4. It is load-independent.** Under full mission load (RTAB-Map + YOLO, load average 4.8),
`/imu` still ran 49.4 Hz with a single dropped update (0.1%) and the lag was unchanged
(223 ms vs 244 ms for the identical light-load run). This is a protocol/buffering defect, not a
CPU one.

## The fixes

Four changes, all with the same theme: **report data with the time it was captured, and do not
throw captured data away.**

### 1. `KobukiDriver.py` — read exactly one packet

`read(200)` → `read(PACKET_REMAINDER_BYTES)` (77), plus a short-read guard so a truncated packet
resyncs instead of being parsed. Removed the now-pointless next-header scan (see above).

### 2. `KobukiDriver.py` + `kobuki_base_node.py` — stamp at capture, not publish

The driver now records `Kobuki.packet_monotonic = t.monotonic()` after a packet is fully parsed.
`kobuki_base_node._capture_stamp()` converts that to ROS time by measuring the data's **age** on
the monotonic clock and subtracting it from ROS `now()` — avoiding any assumption about the
offset between the two clocks, and falling back to `now()` if the age is missing, negative, or
beyond `MAX_CAPTURE_AGE_S`.

Applied to **`/imu`** and **`/odom`**. This is the same defect round 1 fixed for `/scan` and
depth; the IMU was simply missed at the time, which mattered more than either since it is the
only heading source.

### 3. `kobuki_base_node.py` — use every gyro sample

The Kobuki reports the gyro faster than the 50 Hz packet rate, so each packet carries several
z-axis samples. `_imu_update` took only `z_samples[-1]` and discarded the rest. What the EKF
does with the value is *integrate it over the interval*, and the integral of an interval is its
**mean** — a trailing point sample is both noisier and, whenever the rate changes across the
packet, biased. Now `sum(z_samples) / len(z_samples)`.

### 4. `rplidar_node.py` — stamp the revolution's midpoint

A revolution's rays span ~100 ms but every consumer in this stack (RTAB-Map's occupancy grid,
Nav2's obstacle layer) collapses them to the single instant in `header.stamp` rather than
deskewing with `time_increment`. Against a non-deskewing consumer, the `sensor_msgs/LaserScan`
first-ray convention is **not neutral** — it biases every scan late by half a revolution, always
in the same direction. Stamping the midpoint removes the bias and halves worst-case skew to
±`scan_time/2`.

This is a deliberate departure from the strict convention. `time_increment` is still reported
honestly. Nothing here deskews `LaserScan` (`rtabmap_util`'s `lidar_deskewing` node takes
`PointCloud2`), but a consumer that did would need to account for the midpoint.

### Bonus: `/odom`'s `dt` also comes from capture time

`_odom_update` used `get_clock().now()` for both the stamp *and* the `dt` that divides the
encoder delta into a velocity. The delta spans the interval between the **packets** the readings
came from; dividing by the interval between **timer firings** is only the same number while the
two rates agree. When they did not — which, with the old ~20 Hz effective packet rate against a
50 Hz poll, was often — the timer read the same packet twice: one poll saw `delta=0`, the next
saw two packets' worth of ticks still divided by one timer period, i.e. an apparent doubling.
That is precisely the signature that trips `MAX_PLAUSIBLE_SPEED_MPS`, so a share of the
long-running **"Rejecting implausible encoder tick"** warnings were an artifact of this mismatch
rather than genuinely corrupted reads. Timing off capture makes it self-correcting: re-reading
one packet yields `dt <= 0` and is skipped outright instead of inventing a velocity.

## Results

**Scan misregistration during rotation** (mean absolute residual aligning rotating scans against
a stationary reference):

| condition | original | after fix 1 | after all fixes |
|---|---|---|---|
| forward 0.3 rad/s | 4.48° | 2.60° | **1.50°** |
| reverse −0.3 rad/s | 4.00° | 2.80° | **2.21°** |
| forward 0.15 rad/s | 2.77° | 1.59° | **1.12°** |
| **mean** | **3.75°** | 2.33° | **1.61°** |

**~57% reduction**, consistent in all three conditions at both stages.

**The ghost mechanism itself**, over a full live mission:

| | before | after |
|---|---|---|
| `type=0` odometry-edge disagreements | 34 / 688 frames (**4.94%**) | **1 / 2438 (0.04%)** |
| worst magnitude | **36.9°** | **0.62°** |

**120× lower rate, 60× smaller magnitude.** The single survivor at 0.62° sits inside the ±1.81°
stddev RTAB-Map already assumes for odometry edges — it is noise, flagged only as the largest in
one iteration.

Secondary observations from the same run:

* **Map converged** and stopped growing (stable at `250x293`). A ghosting map balloons as
  duplicate walls accumulate.
* **9 cubes delivered**, full mission cycle repeating reliably.
* RTAB-Map processing delay improved (mean 337 → 285 ms, median 300 → 266 ms).

## Hypotheses that were wrong

Recorded so nobody re-chases them.

| hypothesis | how it was ruled out |
|---|---|
| **CPU starvation** of the 50 Hz IMU timer (NoMachine + YOLO + RTAB-Map contending) | under full load `/imu` still ran 49.4 Hz with **one** dropped update (0.1%), and the measured lag was unchanged (223 ms vs 244 ms light-load). Load-independent |
| the round-1 mirrored-scan bug had regressed | offset stayed **positive in both rotation directions**; a sign flip gives `-2×dyaw`. Round-1 fix verified intact |
| gyro **scale** error | residual was flat (~4.5°) across `dyaw` 16°→34°; a scale error grows proportionally. Round 1 had already measured scale at 0.9989 |
| the EKF diverging / filter mistuning | gyro-integrated yaw vs EKF yaw agreed to **0.03°** |
| discarding gyro samples (`z_samples[-1]`) was the primary cause | a real defect and fixed, but sampling a slowly-varying rate at 50 Hz is adequate; it was not where the ~200 ms came from |

The general lesson repeats round 1's: every plausible *parameter* explanation cost a round of
tuning, and the bug was again in the **data pipeline**. Measure the invariant before touching
parameters — and when the invariant fails, measure *which axis* it fails along (time vs angle),
because that alone eliminated three of the five hypotheses above.

## Still to be fixed

### 1. Serial framing is fragile (observed ~1.3/min)

`read_data()` syncs on a bare 2-byte match for `[77, 1]`. That pattern can occur **inside packet
data**, producing a false sync and a misparsed packet. Observed live as garbage encoder values:

```
L=4095 R=4095   L=8 R=4095   L=4112 R=8   L=36 R=65516   L=132 R=854
```

Every one was caught by `MAX_PLAUSIBLE_SPEED_MPS` and **none reached the map**, but the sync
should not rely on a coincidence-prone byte pair.

**Fix:** sync on the real `0xAA 0x55` header, read `<len>`, read `len + 1` bytes, and verify the
trailing XOR checksum before publishing anything.

### 2. No minimum-`dt` guard on the plausibility check

`_odom_update`'s `MAX_PLAUSIBLE_SPEED_MPS` test divides by `dt` with only a `dt > 0` guard, so a
legitimate small delta over a very short interval inflates into an "impossible" speed — observed
once as `1.12 m/s over 3.7 ms` with a perfectly ordinary 53/45-tick delta. `_meas_lin` already
guards this with `MIN_ODOM_DT_S`; the plausibility check should too.

### 3. RTAB-Map accepts **zero** loop closures

The graph is no longer *poisoned*, but it is not *self-correcting* either. Across 2438 frames,
zero loop closures were accepted. The remaining rejections are `Not enough inliers` (16) plus a
few `type=1`/`type=2` candidate-edge errors (0.65–5.4°, 0.12–0.17 m) — normal candidate
filtering, not odometry corruption. Likely a perception/scene-richness or ICP-parameter matter.
Worth checking `Reg/Strategy`, `Icp/CorrespondenceRatio`, and whether the arena has enough
distinctive scan geometry to close a loop at all.

### 4. `TARGETING` aborts ~33% of the time

Lost the cube 4 of 12 times in the validated run. Measured with `ros2 topic hz`:
`/detected_cube` had a **6+ second window with zero messages** between detections, so a single
detection arrives and then nothing until `CUBE_LOST_TIMEOUT_S` (5.0 s) expires. Unrelated to any
of the above. Options: raise the timeout, hold the last bearing and keep rotating through a gap,
or address YOLO throughput / GPU contention directly.

### 5. NoMachine still starves Nav2

Not a cause of *this* bug — explicitly disproven above — but the previously documented problem
stands: the NX virtual session runs at realtime priority and has been measured dropping DWB's
control loop from 20 Hz to 4.7 Hz. Kill the virtual-session processes before any
navigation-timing work. Requires `sudo` (they are owned by `gdm`/`root`/`nx`).

## Reproducing the measurement

The instrument is worth rebuilding if this ever recurs:

1. Bring up `hardware.launch.py` only (no Nav2/RTAB-Map — they are not needed and add variables).
2. Record `/scan`, `/imu`, `/odometry/filtered`, and TF `odom -> laser_frame` resolved **at each
   scan's own stamp**, while driving `rotation_test_node` (bounded: fixed duration, explicit
   stop, self-shutdown — safe in a 1 m radius since it rotates in place).
3. Offline, convert each scan to points, transform into `odom`, and brute-force the rigid
   rotation that best aligns a rotating scan onto a stationary reference. For a static room the
   answer must be ~0.
4. Sweep a time offset `D` applied to the pose lookup and find the `D` that minimises the
   residual. **Run at two speeds** — that is what separates a time lag from an angle error.

Two traps worth knowing:

* **Judge the confidence, not just the minimum.** Compare the residual at the best offset against
  the residual at `D = 0`. A 2.6× well is trustworthy; a 1.0× well means the minimum is flat and
  the number is noise. As the lag shrinks the well necessarily shallows, so late-stage estimates
  are inherently softer than early ones.
* **Do not differentiate the pose stamps directly.** Once `/imu` and `/odom` are stamped at
  capture they derive from the *same* serial packet, so the EKF's output stamps cluster onto
  packet arrivals and repeat. `np.gradient` across those near-zero `dt` values returned
  500–1200°/s on a 17°/s turn. Resample onto a uniform grid first.
