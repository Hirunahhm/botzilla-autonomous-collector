# Research Discussion: Literature Review, Evidence Audit, and Experiment Design

This document records a working session (2026-09-23 to 09-24) that reviewed the state of the
BotZilla research (*Interleaved Exploration and Coverage for Target Retrieval under Short-Range
Perception*), read the four closest papers in the literature, and turned both into a concrete
experiment plan.

It covers, in order:

1. An audit of what the existing run logs actually support (§1).
2. The four papers, what each does, and what each means for this project (§2).
3. How the papers fit together, and where BotZilla sits (§3).
4. The coverage cost layer: built, tested on hardware (run 22), and its known risks (§4).
5. Hardware findings about cube visibility (§5).
6. The decisions taken about the research question and experiment (§6).
7. The inspection-primitive question: lawnmower rows vs. spinning at viewpoints (§7).
8. **The final design: region-by-region search with robot-optimised inspection (§8).**
9. Work order and open items (§9), and references (§10).

The single most important finding: **the headline result in the extended abstract (+15.6 pp for
interleaving) does not survive a matched comparison, and the interleaving trigger never actually
switched.** The research question is sound, but the evidence for it has to be re-collected.

---

## 1. Evidence audit: what the run logs actually support

### 1.1 The policy comparison is confounded

The abstract's Table I compared *final* coverage across runs of different lengths and different
code versions:

- All 3 `exhaustion` (explore-then-sweep) runs are on commit `602074b` (Sept 5).
- 5 of the 7 `fraction` (interleaved) runs came from later commits (`44d8aa7`, `99a5e3a`,
  `53be328`), which include the sweep-spacing fix, the straight-line row planner and other
  navigation changes.
- The fraction runs also ran longer (38 and 50 min vs 15–34 min), and coverage only grows.

Comparing at the **same elapsed time on the same commit** (`602074b`) removes the advantage:

| Swept coverage (%) | 10 min | 14 min | 20 min | 30 min |
|---|---|---|---|---|
| Exhaustion (3 runs) | 28.0 / 24.5 / 30.4 | 43.9 / 28.5 / 39.0 | 29.1 / 41.9 | 32.4 |
| Fraction (3 runs) | 34.5 / 25.4 / 28.6 | — / 30.1 / 31.4 | 38.6 / 37.4 | 49.8 / 55.9 |

Exhaustion is *ahead* at 14 min. Fraction only leads at 30 min, where there is a single
exhaustion run (the one stuck at a doorway). A Mann–Whitney test on the original 3-vs-7 split
gives p ≈ 0.008, but that test assumes runs differ only in policy. Here they differ in code and
duration, so the statistic must not be reported.

### 1.2 The interleaving trigger never switched

The `fraction` policy sweeps whenever un-swept free area exceeds 15% of known free area. In every
log checked, the first sweep starts at 100% un-swept (e.g. `11238/11238`), and coverage never
approaches 85%. The condition stays true for the whole run. Each sweep ends with roughly 45% of
the floor still un-swept, because the map grows during the sweep and partly-swept rows are
re-driven end to end.

**So the "interleaved" arm actually ran as "sweep almost continuously, with a 20 s cooldown".**
`sweep_fraction` has never been tested as a parameter. This also explains the apparent
contradiction a reviewer would spot: a 15% threshold with a 58% final coverage.

### 1.3 Other gaps found

| Gap | Consequence |
|---|---|
| No run has a `--layout` (ground-truth cube positions) | `cube_inspected`, the designed primary metric, has never been collected. Every result so far uses the coverage proxy. |
| The four Sept 6 evening runs ran with metrics off | No coverage data for them at all. |
| Coverage denominator is "known free cells" | A policy that maps less gets a higher percentage for the same floor seen. Needs a fixed reference (measured arena floor area). |
| `run_config.json` did not record the planner before run 22 | Earlier runs' planner is inferred (GridBased by default), not recorded. Fixed now. |
| Swept mask counts floor from 0 m | The floor within ~0.48 m of the camera is below the image, and depth is blind below 0.55 m. Coverage is overstated. |
| No occlusion check | Floor behind a chair leg or box within 1 m counts as seen. Both Star-Searcher and HEATS require "not occluded". |
| Partly-swept rows are re-driven whole | `generate_coverage_waypoints` drops a run only if every cell is swept, so a 4 m row with one un-swept cell is driven end to end. |
| Sweep rows cross the whole map | Rows ignore rooms and doorways; a multi-room map crosses a doorway on every row. |

### 1.4 What is solid

- The drivetrain `rotate_flag` root cause, proven by a same-battery A/B against gyro ground truth.
- The "actuation faults masquerade as planning faults" finding.
- `mission_metrics_node` is subscribe-only, so it cannot influence the runs it measures.
- Rejected hypotheses are documented in the code rather than silently reverted.

---

## 2. The four papers

### 2.1 Star-Searcher (Luo et al., IEEE RA-L 9(5), 2024)

**Platform.** A drone with a 360° LiDAR (mapping) and a wide-angle RGB camera (inspection), all
on a Jetson Orin NX, the same family as BotZilla's Orin Nano.

**Mechanics.**

1. **Inspection-aware map.** Each occupied voxel stores the closest distance the camera has seen
   it from (in view and not occluded). Voxels never seen within `dmax` (3 m sim, 2 m real) are
   "uninspected". This is the 3D-surface version of BotZilla's swept mask.
2. **Viewpoint scoring.** `score = viewing-angle term × (0.8·uninspected + 0.2·frontier)`. The
   paper only says the inspection weight is "larger"; the 0.8/0.2 values are not justified.
3. **Visibility-based clustering.** Mutually visible viewpoints form one cluster, roughly one
   open region, which is finished before moving on.
4. **History-aware global path.** Keeps the previous visiting order for unchanged regions and
   slots new ones in; resets only if the old order becomes clearly worse. Prevents dithering.
5. **Local path cost includes turning time:** `max(distance / vmax, yaw change / ωmax)`.

**Results.** 4 simulated scenes, 8 AprilTags each, 5 runs per method, mean ± std of time and path
length plus completeness. 100% completeness everywhere and fastest. Ablation: 126.3 s without
either module → 97.8 s with both. Real world: 6 and 10 tags found in 110 s and 180 s.

**The result that matters most for BotZilla.** FUEL, a pure exploration planner, with sensing
range 4 m against a 3 m recognition range, found only **95.0 / 77.5 / 68.8 / 81.3%** of tags.
With its range set to 3 m (FUEL-3m) it found 100%, but more slowly. That is the
"mapped ≠ seen" problem, measured independently. BotZilla's mismatch is far larger (LiDAR maps
several metres; the camera reaches 1 m).

### 2.2 HEATS (Zhang et al., IEEE/RSJ IROS 2025; arXiv 2503.07986)

**Platform.** A mobile manipulator: differential base with LiDAR, depth camera on the arm. Code at
`github.com/Andy168byte/HEATS`.

**Mechanics.**

- Builds on Star-Searcher's inspection map and 0.8/0.2 weights.
- Viewpoints sampled on rings around each cluster (ΔR 0.3 m, Δθ 30°), each with level, downward
  and upward arm poses precomputed offline.
- Travel term `exp(−λ·t)` with **straight-line distance / vmax**, not planned path length.
- **Two-stage decision:** the 2D costmap is split into regions; a tour problem (ATSP, LKH solver)
  orders the regions; a second one orders viewpoints inside the current region. The current
  region is finished before leaving.
- Ignores the goal heading, because the 360° LiDAR and the arm compensate.

**Results.** Office (30×20 m) and maze (25×22 m), 8 tags each, 10 runs, 15-minute limit. HEATS
found all tags in 556 ± 13 s and 420 ± 21 s. Baselines (WG-NBVP, AEP) ran past 900 s with
42.5–85% completeness. Real world: 12×5 m, 8 tags in 118 s over 16.9 m.

**Caveats.** The baselines use only the arm camera (no LiDAR). WG-NBVP was designed for mapping a
contamination region, not for finding tags, so it is scored on a task it wasn't built for.

### 2.3 Gao et al.: Indoor exploration and simultaneous trolley collection (arXiv 2309.11107; ICRA 2024 per HEATS' citation)

**Why it matters most.** The closest mission to BotZilla: a ground robot explores with LiDAR, has
a front camera with limited view, and does find → approach → collect → return to a drop-off →
keep exploring. It starts from the "trolley returning spot", i.e. HOME.

**Mechanics.**

1. The 3D LiDAR (Ouster OS1) data is split into **walls** (→ room outlines, labelled room or
   corridor), **object proposals** (clusters outside the camera view whose size and shape match a
   trolley) and **obstacles** (polygons).
2. **Proposals fix "the camera didn't see it":** the robot drives to a viewpoint on the line
   toward each proposal, at `0.8 × min(camera range, distance)`, and lets the camera confirm.
3. **Next goal from a tour with strict priority tiers** (TSP with precedence constraints): object
   proposals → frontiers in the current room → frontiers in other rooms → corridors. Travel cost
   is real collision-free path length.

**Results and gaps.** Exploration only is evaluated numerically (20 sim runs per method, coverage
vs time, beating classic frontier, FAEL and a greedy segmentation method), **with object
proposals switched off**. The real trolley collection is a demonstration with no numbers. The
authors concede the robot can miss objects it never revisits with the right pose.

**This is BotZilla's opening:** the only paper doing find-and-fetch with returns to base doesn't
measure the search part.

### 2.4 WG-NBVP (Naazare et al., IEEE RA-L 7(2), 2022)

**Platform.** Mobile manipulator with an arm-mounted D435: 86°×57° FOV, **0.3–1.5 m range**, as
short-range as BotZilla's Kinect cone.

**Mechanics.** An RRT of candidate camera poses; score `5·Gf + 1·Gm + 500·Gv` (unmapped volume,
prior contamination value, and a −1 penalty for already-visited poses). **No travel-cost term.**
Uses the arm first; drives only when no new pose is reachable by the arm.

**Results.** 10 sim runs per setting, 25×25 m area with an 8×8 m region of interest: ~82.6% of
the region mapped in 60 min vs under 40% for AEP.

**Two results that matter for BotZilla.**

1. **Fixed vs moving camera (Table V):** arm locked with the camera facing forward, like BotZilla,
   mapped **50.2% after 309.6 m**; with the arm moving, **82.6% after 167.1 m**. Independent
   evidence that a fixed, forward-facing short-range camera is the hard case.
2. **The travel-cost claim is weak:** 167 ± 27, 185 ± 69 and 187 ± 43 m overlap heavily. Don't
   cite it as showing travel cost doesn't matter.

**Connection to the cost layer.** The `Gv` visited penalty is a pose-level ancestor of BotZilla's
coverage cost layer, which applies the same idea along the whole route.

### 2.5 References verified through the papers' bibliographies

- **Kim et al., RA-L 7(3):6343–6350, 2022** (Star-Searcher ref [4]; Gao ref [24]): ground robot,
  2D map segmentation plus object detection. Likely the closest ground-robot comparison.
- **Heng et al., ICRA 2015** (Star-Searcher ref [11]): "Efficient visual exploration and coverage
  with a micro aerial vehicle in unknown environments". Title is almost this topic.
- **Luperto et al., RA-L 2022** (Gao ref [28]): room segmentation from a 2D occupancy grid. The
  tool for region-by-region sweeps without a 3D LiDAR.

---

## 3. How the papers fit together, and where BotZilla sits

The clearest framing for related work is **how each method combines exploration and inspection**:

| Approach | Papers | How the two are combined |
|---|---|---|
| **Weighted sum** | WG-NBVP, Star-Searcher, HEATS | One score, fixed weights |
| **Strict priority** | Gao et al. | Object proposals always before frontiers |
| **Threshold switching** | **BotZilla (fraction trigger)** | Explore until the un-swept backlog passes a threshold, then sweep |
| **Path-level shaping** | **BotZilla (coverage cost layer)** | Changes the route, not the goal |

WG-NBVP's own references (Marler & Arora; Wilfried & Blume) discuss the known weaknesses of
weighted sums against cascaded or priority schemes. That gives a principled reason to study
switching, not just a different design choice. It only holds once the trigger actually switches.

**Where BotZilla differs from all four:** a ground robot with a **fixed, narrow, ~1 m camera**,
targets on the **floor**, **physical retrieval** that interrupts the search, and **low-cost
hardware** (Orin Nano, Kobuki, 2D LiDAR).

**Where the papers are weaker than they look (useful for positioning):**

- Idealised targets: AprilTags with a fixed recognition range; no detector misses modelled.
- Handicapped baselines (no-LiDAR baselines in HEATS; depth-camera FUEL and a re-implemented
  "Semantic" in Star-Searcher).
- Real-world results are single runs without statistics.
- No paper retrieves anything during its evaluation.

**Cautions.** Copying the 0.2/0.8 weights is weaker than it sounds: they multiply raw cell
counts, and with a 1 m camera against a multi-metre LiDAR the LiDAR counts would swamp the camera
counts. Normalise each count (e.g. by the maximum achievable in one view) and say so. Also search
"coverage-aware costmap" and "inspection cost layer" before claiming the cost layer is new.

---

## 4. The coverage cost layer

### 4.1 What was built

| Component | What it does |
|---|---|
| `botzilla_coverage_layer` (new C++ package) | Global-costmap layer. Samples `/coverage_cost_map` by world coordinate and max-combines it below inscribed cost. Never resizes the master costmap (a second `StaticLayer` would, and a stale grid would wipe the real map). |
| `frontier_explorer_node` | New `coverage_cost` parameter (0–74, default 0 = off). Publishes the cost smoothed over the 0.47 m swath half-width. |
| Chase/delivery fix | While the executor owns the base, the cost grid is all zeros, so chasing and the HOME delivery plan on the plain costmap. |
| Abandoned-cube fix | `executor_node` estimates each cube's map position from ranged detections and publishes `/cube_abandoned` when a chase times out. The explorer treats a 0.75 m disc around it as un-swept for planning until re-seen from ≥0.5 m away (max 2 revisits per spot). The metric mask is never un-marked. |
| `run_full_mission.sh` | `--coverage-cost N` flag; `run_config.json` now records `planner`, `row_planner`, `coverage_cost`. |

At `coverage_cost = 15` (≈38 internal), SmacPlanner2D with `cost_travel_multiplier: 3.0` charges
fully inspected floor about 1.45× per metre, so it accepts up to ~45% longer routes through
un-inspected floor.

**Framing (adopted):** the layer changes *how the robot gets there*, not *when* to sweep or
*where* to go. It is a factor applied on top of a policy, not a competing arm. The effect is
policy-with-layer minus the same policy without it.

### 4.2 Run 22 (hardware, `run_logs/20260924-001401`)

SmacGrid, `coverage_cost = 15`, metrics on, no cubes.

| | 5 min | 10 min | 12 min | End (14.9 min) |
|---|---|---|---|---|
| Run 22 | 20.0% | 31.6% | 35.6% | 40.4%, 45.8 m |
| Earlier runs (range) | 19–25% | 24–38% | 27–46% | — |

- The layer loaded and worked; explorer + executor CPU ~17% (rtabmap ~117%).
- Mid-pack coverage: one run on a new commit and day can't show a benefit either way.
- 9 stalls, including ~3 min lost on one sweep row (DWB `ObstacleFootprint`, all 440
  trajectories rejected), the same row-level problem as run 21.
- No cubes present, so the chase/revisit fixes were only tested in isolation.

### 4.3 Known risks

| Risk | Status |
|---|---|
| **Wall-hugging returning** | Partly handled. Inflation band (0.165–0.45 m) costs 107–252 internal, far above 38, so max-combine preserves it. Gap: un-inspected floor just *outside* the band (0.45–0.9 m from walls) costs 0 while inspected open floor costs 38, so the planner may prefer strips alongside walls. Fix: no discount within ~0.3 m of the inflation edge. |
| **Weaving / flip-flop** | Forward camera makes its own path ahead more expensive each second; with 1 Hz replanning this could push paths sideways. Run 22 shows no sign: 88.7°/m total turning (52.1°/m while moving) vs 80–139°/m (44–61°/m) in 12 earlier runs. Fix if needed: publish the cost grid only at goal dispatch. |
| Effect fades late in a run | Expected; coverage-vs-time is the right metric. |
| Sweep rows ignore it | By design: SweepStraight ignores cost; only frontier and transit legs are affected. |

Correction recorded: an earlier claim that max-combine flattens the inflation gradient beyond
~0.8 m was wrong, because `inflation_radius` is 0.45 m.

---

## 5. Hardware findings: can the robot see cubes other than with YOLO?

- **The RPLIDAR cannot see cubes.** Its scan plane (~12 cm on the robot, per the user; the URDF
  says 0.24 m, so **measure it**, since the URDF feeds TF) passes over the 10 cm cubes.
- **The depth scan (`/scan_camera`) sees them only sometimes.** It keeps points 0.05–0.30 m high,
  0.55–3.0 m out, within ±0.5 rad. A 10 cm cube only has a 5 cm slice in that band.

**Likely causes, most likely first:**

1. **Uncalibrated camera tilt.** The URDF mounts the camera level (`xyz="0 0 0.19" rpy="0 0 0"`);
   the Kinect has a tilt motor that `kinect_bridge` never sets or reads. A tilt error θ shifts
   heights by ~`distance × tan θ` (1.5° at 2 m ≈ 5 cm, the whole window). Tilted up → cube points
   drop below 0.05 m (misses). Tilted down → far floor appears above 0.05 m (phantom obstacles).
2. **Depth intrinsics are probably the RGB camera's** (`fx = fy = 525`); the unregistered depth
   comes from the IR camera (typically ~580–590). Shrinks the cube's visible slice to ~4 cm.
3. **Depth noise grows with distance²** (~1 cm at 2 m, 2–3 cm at 3 m).
4. **Single-frame decisions**: no accumulation of evidence across frames.

**Possible link to the stalls (unproven):** `/scan_camera` marks obstacles in the local costmap.
Phantom floor obstacles from a downward tilt would produce exactly `ObstacleFootprint / Trajectory
Hits Obstacle`, and would explain run-to-run variation in stall rates.

**10-minute static test:** hardware launch only; face open floor; record `/camera/points`; fit
the floor plane (gives real tilt/roll/height vs URDF 0°/0°/0.19 m); place a cube at 1.0, 1.5,
2.0, 2.5 m and measure the fraction of frames with cube points in band; check `/scan_camera` for
returns on empty floor.

**Fixes once confirmed:** command the tilt motor to a set angle at startup and publish the actual
pitch into TF; use IR-camera intrinsics; for cube proposals, detect bumps relative to a per-frame
floor fit and require persistence across frames and viewpoints.

**Why it matters for research:** if calibrated, the depth scan sees cubes geometrically out to
3 m, a wedge ~8.7× the area of the 1 m detection cone. That makes Gao-style "depth-geometry
proposals" possible (drive to a viewpoint ~0.8 m from a candidate blob and let YOLO confirm).

**Decision (09-24): dropped.** The depth stream has proven too unreliable on this hardware to base
a search strategy on, so depth-geometry proposals are out of the plan. The tilt calibration is
still worth doing for navigation (possible phantom obstacles), but YOLO within ~1 m remains the
only cube detector the research relies on.

---

## 6. Decisions: research question and experiment design

### 6.1 Research question

> *Given a fixed time budget, which strategy lets a ground robot with a fixed short-range camera
> and a 2D LiDAR find the most floor targets, and find them soonest? Does the state-of-the-art
> blended-utility approach, designed for drones and arm-mounted cameras, still win when turning
> is expensive and the camera sees very little?*

Either outcome is publishable: blending carries over to cheap robots, or structured coverage wins
and the paper shows where the state of the art breaks down.

> **Refined in §8.6** into a single question with a proposed answer (region-by-region search).

### 6.2 Decisions taken

- **All experiments on hardware.** No simulation campaign (simulation introduced more problems
  than it solved).
- **No cube collection during the comparison.** Retrieval noise (failed chases, grabber drops,
  delivery trips whose length depends on cube placement) is unrelated to the strategy, and each
  delivery drags the robot back to HOME. Collection is shown separately as an **end-to-end
  demonstration** (2–3 full missions with the best strategy), the role Gao et al.'s demo plays.
- **Not "fastest to find all" or "fastest to finish"** as the main measure: hardware runs often
  won't find all cubes or finish coverage in the time limit, leaving those values undefined.

### 6.3 Metrics

- **Primary:** within a fixed 15 minutes (as in HEATS), the fraction of cubes found and the time
  to find each one.
- **"Found" at two levels:** *inspected* (cube entered the 1 m camera cone, from the layout
  file; the pure effect of the strategy) and *detected* (YOLO reported it; includes detector
  misses). The gap between them is attributed to the detector.
- **Supporting:** camera-inspection coverage over time (and area under the curve), map coverage
  over time, distance, total turning, stalls, false-positive detections.
- **Separate:** cubes collected, from the end-to-end demonstration.

### 6.4 Arms

| # | Arm | What it does | Extra work |
|---|---|---|---|
| **A** | Frontier-only (Yamauchi) | Explore until no frontiers remain, then stop | None: identical to B until exploration ends; read from B's runs |
| **B** | Explore-then-sweep | Current `exhaustion` arm | None |
| **C** | Interleaved switching | Current `fraction` arm **with the trigger fixed** | Trigger fix (small) |
| **D** | Blended utility (HEATS/Star-Searcher style) | Score viewpoints by `motion gain × (0.2·exploration + 0.8·inspection)` | Largest job |
| **E** | Camera-range exploration (Star-Searcher's FUEL-3m control) | Frontier exploration on the swept mask: go to the nearest seen/unseen boundary | Small: reuse `find_frontiers` on the mask |

**Factors (on/off, applied to the best one or two arms):** coverage cost layer. (Depth-geometry
cube proposals were dropped; see §5.)

> **Superseded in part by §8.** The arms above mixed two questions (*when* to inspect and *how*).
> The final design in §8 keeps B, C, D and E as **baselines** for one proposed method, and adds
> ablations that separate the two questions.

**Ablations for C:** adaptive trigger vs a fixed schedule (the stubbed `interval` mode); two or
three threshold values.

**Optional reference:** a boustrophedon sweep on a pre-built map (upper bound for normalising).

Left out: random walk and wall-following (too weak to be informative).

### 6.5 Adapting arm D fairly

| HEATS / Star-Searcher | BotZilla version | Why |
|---|---|---|
| Inspects 3D surfaces | Inspects floor cells via the shared swept mask | Targets are on the floor; all arms share one definition of "inspected" |
| Ring sampling (ΔR 0.3 m, Δθ 30°) | Same, **including heading** | Fixed camera; HEATS ignores heading only because its arm compensates |
| Weights 0.2 / 0.8 | Same, **normalised** by the most cells one view can see | 1 m camera vs multi-metre LiDAR |
| `exp(−λ·t)`, straight-line distance (HEATS) | `t = max(path length / v, turn angle / ω)`; path lengths from one Dijkstra over the costmap | Straight lines cut through walls; turning is the costly move |
| Two-stage region/viewpoint ordering | Same, with Luperto-style 2D room segmentation and a small solver (e.g. OR-Tools) | Few candidates; runs on the Orin Nano |
| Whole-body motion planning | Not used; Nav2 drives to the viewpoint | Same executor for every arm |

Name it a **"HEATS-style blended-utility planner"**, never "HEATS". Pending decision: full
two-stage version (~1 week) vs a lighter greedy version (2–3 days, easier to call a weak
baseline). Recommendation: full version.

### 6.6 Hardware protocol

- **Budget.** 15 min run + ~10 min reset ≈ 2 runs/hour. B, C, D, E × 5 runs = 20 counted runs,
  ~10–13 hours of lab time before failures.
- **Blocks.** Each block = one run of each arm, in **shuffled order**, spreading battery, time of
  day and wear evenly. Never run all of one arm on one day (the confound in §1.1).
- **Battery.** Start every run above a fixed voltage; record `/battery`.
- **Layouts.** 2–3 layouts of 4–6 cubes; every arm on every layout (paired comparisons).
- **Frozen code.** One tagged commit; `git_dirty: false` for every counted run.
- **Cube displacement.** Check each cube's position after every run; a moved cube invalidates
  its ground truth for that run.
- **Statistics.** Mean ± std, per-layout paired comparisons, rank-based tests.

### 6.7 New code the design needs

1. **Detect-only mission mode** in `executor_node`: log each detection with its estimated map
   position (reusing the `/cube_abandoned` projection) and keep exploring instead of chasing.
2. **Detection-to-cube matching** in `mission_metrics_node`: nearest layout cube within ~0.5 m;
   unmatched = false positive.
3. A **fixed coverage denominator** (measured arena floor area).

---

## 7. The inspection primitive: lawnmower rows vs spinning at viewpoints

### 7.1 The idea

Search the way a person with a torch searches a dark room: go region by region, stop, sweep the
beam around, mark the area as searched (found or not), move on. With a fixed camera, "turning
the head" means rotating the robot. The observation behind it is correct: a lawnmower pattern is
built for tools that must physically pass over every spot (mowers, vacuums). A camera only has to
be within ~1 m and pointed at a spot. That makes this a viewpoint-coverage (art-gallery) problem,
which is why HEATS and Star-Searcher are viewpoint planners.

### 7.2 The numbers

| | Lawnmower rows | 360° spin at points |
|---|---|---|
| How it sees | 0.94 m strip while driving | Ring around the robot while rotating |
| Speed | 0.2 m/s (`max_vel_x`) | 0.4 rad/s (`max_vel_theta`), ~16 s per full spin |
| Theoretical rate | ~11 m²/min while on a row | ~2 m²/min including short trips between points |
| Actual rate (run 22) | **~1.5 m²/min**, including exploration, transits and stalls | Not measured |

In theory rows are ~5× more efficient; in practice overhead eats almost all of that. The real
question is **which primitive fails less on this robot**. Spins only need a clear circle of ~0.42 m
radius and point-to-point travel; rows fail when they run along furniture, which is where the
stalls happen.

### 7.3 Three corrections to the idea

1. **Blind hole under the robot.** Camera at 0.19 m, Kinect vertical FOV ~43° (±21.5°): the floor
   only comes into view from ~0.48 m (assuming a level camera). A spin covers a **ring 0.48–1.0 m**,
   not a disc. Marking the square inside the circle would claim floor that was never seen.
   → Space spin points **≤ ~1.0 m** apart so neighbours cover each hole; let the swept mask record
   what was seen; use squares for planning only. Effective area drops to ~0.8–1.0 m² per spin.
2. **Partial spins.** Rotate only through the directions where the ring still has unseen floor
   (from the swept mask). Beside a wall or a swept area that may be 120°, not 360°. This
   "look only where needed" version is the one worth building.
3. **Rotation speed vs YOLO frame rate.** At 0.4 rad/s a cube stays in the 57° view for ~2.5 s.
   YOLO's frame rate isn't in the logs; measure it and set the spin speed for several frames per
   cube.

### 7.4 Where each should win

- **Spins in clutter:** see around objects from every direction; a failed point costs one point,
  not a row.
- **Rows in open rooms:** continuous sensing, no stops.
- This suggests a new option: **choose the primitive per region** (rows in open rooms, spins in
  cluttered ones).

**Region by region** is a separate, sound idea: Gao et al. and HEATS both finish one region before
the next and show it reduces back-and-forth. It applies to either primitive. "Region done" should
mean its floor is covered in the swept mask, not a flag the robot sets.

**Novelty, honestly:** stopping to scan is not new in exploration. What is new is testing it
against a lawnmower sweep for a fixed 1 m camera on real hardware.

### 7.5 How it fits the experiment

The primitive (*how* to inspect) is a separate question from the arms (*when* to inspect).
Crossing them on hardware doubles the runs, so:

1. **Pilot first:** one room, pre-built map, same cubes; row sweep vs full-spin grid vs partial
   spin, 3 runs each (~1.5 h). Measure floor inspected per minute, cubes found, stalls.
2. **Use the winner as the primitive for B and C.** Arm D already chooses viewpoints with a
   heading, so partial spin fits it naturally.
3. **Report the pilot** as the justification. If the difference is large, per-region primitive
   choice may become a contribution of its own.

---

## 8. Final design: region-by-region search with robot-optimised inspection

### 8.1 How the design was reached

1. **The original arms mixed two questions.** Every strategy answers *when* to inspect
   (sequential, switching, blended) and *how* to inspect (rows, spins, viewpoints). B and C use
   rows; D and E use viewpoints. If D beat C, it would be impossible to say whether blending beat
   switching or viewpoints beat rows.
2. **Star-Searcher and HEATS are system papers.** Neither asks "which strategy is best" as an open
   comparison. Each proposes one method, beats baselines, and uses ablations (removing one
   component at a time) to show each part matters. Following that structure turns the two
   questions into **one proposed method with supporting ablations**, which also fits a two-page
   abstract.
3. **Region by region is borrowed, not new.** HEATS (Algorithm 1) orders regions with a tour
   solver and only leaves the current region when it has no candidate viewpoints left. Gao et al.
   visit current-room frontiers before other rooms. Star-Searcher's visibility clusters do the
   same. The contribution has to come from what happens *inside* each region.
4. **Small tiles were considered and rejected** in favour of rooms (§8.2).

### 8.2 The unit of search: rooms, not small tiles

The human intuition was to search in small patches: check a small area, mark it done, move to
the next. The idea is well-grounded (it is **grid-based cellular decomposition coverage**; the
classic online version is Spiral-STC, Gabriely & Rimon 2002), but it copies a human limitation
the robot doesn't have:

| | People | BotZilla |
|---|---|---|
| **Why small patches** | Limited memory and attention: you can't remember which spots you checked across a whole room | Doesn't apply: the swept mask records every cell the camera covered, at 5 cm |
| **Weak point** | — | **Stopping and turning:** 0.4 rad/s rotation, Nav2 start/align/settle overhead on every goal, stall risk. Run 22 showed overhead eating most of the theoretical efficiency. Small tiles mean many stops, hitting this hardest |
| **Strong point** | Turning the head is free | **Planning over everything known at once** and **looking while moving** |

So the unit should be **the largest area the robot can plan over well: a room.** Once a room is
mapped, its whole inspection is planned in one go, with as few stops as possible, while the camera
keeps looking during every move.

What is kept from the human idea is the **discipline, not the patch size**:

- **finish an area before leaving it** (no trips back across the map later);
- **"done" is verified**, meaning the swept mask shows the floor covered, not that the robot
  decided so.

> **Human-style region discipline, robot-optimised inspection within each region.**

Small tiles stay as an **optional ablation** (same method, unit size ~1–2 m) to turn "robot memory
makes small units unnecessary" from an argument into evidence, if the hardware budget allows.

### 8.3 The proposed method

```
          ┌─────────────────────────────────────────────┐
          ▼                                             │
  1. EXPLORE region R  ── until R has no inside frontiers
          │                                             │
          ▼                                             │
  2. INSPECT region R  ── until R's floor is ≥ ~90% seen
          │               (rows or viewpoints, chosen per leftover patch)
          ▼                                             │
  3. LEAVE R through an exit (doorway frontier)         │
          │   into the nearest unfinished region  ──────┘
          ▼
  no exits left and every region done  →  finished
```

**The two switches between exploring and inspecting:**

1. **Explore → inspect:** when region R has no *inside* frontiers left. R's full shape is known,
   so its inspection can be planned properly (the failure of sweeping a half-known map is what
   §1.2 found).
2. **Inspect → explore:** when R's floor coverage in the swept mask reaches the "done" level
   (~90%; below 100% so floor the robot can't reach can't block progress). The robot then leaves
   through an exit into new space, which becomes the next region.

**Inside vs exit frontiers.** Frontiers inside R (an unmapped corner) are part of exploring R.
Frontiers on R's boundary that lead elsewhere (an open doorway) are **exits**: recorded, but
locked until R is done. That makes "never leave a room half-searched" a precise rule.

**Inspection is chosen per leftover patch.** While exploring R the camera already sees part of the
floor; inspection plans only what is still unseen:

- **large open patches → rows** (continuous looking while driving, no stops);
- **small scattered patches → viewpoints** (a pose plus heading chosen so one look covers the
  most unseen floor). Suggested threshold: patches under ~2 m² (about four looks).

**Details that keep it robust:**

- **Freeze a region's boundaries once work on it starts.** The map keeps growing; without this,
  regions split and merge mid-task (the back-and-forth problem Star-Searcher's history-aware
  planning solves).
- **A doorway found during inspection** is recorded as an exit and never interrupts the
  inspection (same rule as today's "a sweep always runs to completion").
- **Next region = nearest unfinished one** (close to HEATS' tour ordering with few rooms);
  **corridors last**, following Gao et al., if the arena has one.
- **Regions from the 2D map:** split at narrow passages (gaps under ~1 m, found from each free
  cell's distance to the nearest wall; Luperto et al. is the published method). **In open space
  without walls,** fall back to bounded virtual regions (split at narrow passages, or capped at a
  set size such as 4×4 m).

**Condition:** in a single open room the whole arena is one region, and the method becomes
identical to B (explore-then-sweep). **The test arena needs several rooms or partitions**
(boxes or boards). All the compared papers tested multi-room layouts too.

**How its switching differs from the other strategies:**

| Strategy | When does it switch from exploring to inspecting? |
|---|---|
| **B: explore-then-sweep** | Once, when the **whole map** is explored; then sweeps everything, crossing the map back and forth |
| **C: interleaved** | Whenever the **global un-swept percentage** passes a tuned threshold (which never switched off in the logs) |
| **D: HEATS-style** | **Never explicitly:** every decision weighs both in one score |
| **Proposed** | **Per region:** when the region's shape is known; leaves when its floor is covered |

No tuned trigger remains: the only parameter is the "done" coverage level, whose meaning is
obvious. And with regions removed, the method is exactly B, so its ablation is clean.

### 8.4 Relation to HEATS, and the contribution

| | HEATS | Proposed |
|---|---|---|
| **Order of regions** | Region by region, finish before leaving | **Same (borrowed)** |
| **Inside a region** | **Blended:** exploration and inspection viewpoints compete in one score (0.2/0.8) | **Sequenced:** explore the region, then inspect what's left |
| **Inspection method** | Viewpoints only | **Per leftover patch:** rows for large open areas, viewpoints for small scattered ones |
| **When a region is done** | Implicitly, when no viewpoint scores above a threshold | Explicitly, when the swept mask shows ~90% of the region's floor covered |
| **Camera** | Arm-mounted, aimed freely, 3–4 m range | Fixed, ~1 m, the robot must turn to look |

Suggested wording of the contribution:

> *Following HEATS and Gao et al., we search region by region. We show that for a ground robot
> with a fixed 1 m camera, two changes inside each region, sequencing exploration before
> inspection and choosing the inspection method per unseen patch, improve search time and
> completeness over HEATS-style blended viewpoints.*

**Risk.** The differences are *inside* regions, which is narrower than "a new search strategy". If
the gains are small the paper reads as incremental. If rows and viewpoints turn out to perform
about the same on this robot, the per-patch choice adds little, and the story should be rethought
before investing in the full build. The ablations below give that early warning.

### 8.5 Experiment: proposed method, baselines, ablations

| Variant | Regions | Inside a region | Inspection | Role |
|---|---|---|---|---|
| **Proposed** | ✓ | Sequenced | Rows + viewpoints by patch | The method |
| **D: HEATS-style** | ✓ | Blended | Viewpoints | State-of-the-art baseline |
| **B: explore-then-sweep** | ✗ | Sequenced (whole map) | Rows | Classic baseline; also "no regions" ablation |
| **C: interleaved** | ✗ | Switching (fixed trigger) | Rows | The earlier method, as a baseline |
| **E: camera-range exploration** | ✗ | Greedy | Viewpoints (nearest seen/unseen boundary) | Star-Searcher's FUEL-3m-style control |
| **Ablation 1** | ✓ | **Sequenced** | Viewpoints | Sequencing vs blending (vs D) |
| **Ablation 2** | ✓ | Sequenced | **Rows only** | Whether viewpoints help (vs Proposed) |
| *Optional: small tiles* | 1–2 m tiles | Sequenced | Rows + viewpoints | Whether rooms beat small units |

Arm A (frontier-only) still comes free from B's runs.

**Budget (hardware only, ~2 runs/hour):**

- **Essential:** Proposed, D, B × 5 runs = 15 runs.
- **Recommended:** C and E × 5 = 10 runs.
- **Ablations:** 1 and 2 × 3 runs = 6 runs; small tiles × 3 = 3 more if affordable.
- **Total:** 25–34 runs, about 13–17 hours of lab time before failures. Same block protocol as
  §6.6 (shuffled order per block, battery check, paired layouts, frozen commit).

The §7.5 primitive pilot is now mostly answered by Ablation 2 (rows only) and Ablation 1
(viewpoints only) inside the full design. It remains useful as a cheap early warning (~6 h) if
run before the full build.

### 8.6 Paper framing

**Research question (replaces §6.1):**

> *How should a ground robot with a fixed short-range camera and a 2D LiDAR organise its search
> for floor targets?*

**Proposed answer:** region by region, exploring each region before inspecting it, with the
inspection method chosen per unseen patch.

**Extended abstract vs full paper.** Two pages fit one question. The abstract can carry this
single question with a compact result if the essential runs are done in time; otherwise it
reports the inspection-method comparison (ablations 1 and 2 plus rows/viewpoints) as the
motivating result, and the full paper (IROS/RA-L, like the papers built on) carries the complete
comparison with the HEATS-style baseline as its centrepiece. Decide once the deadline is known.

---

## 9. Work order and open items

### 9.1 Order

1. **Shared prerequisites** (affect every arm):
   - row-level abandonment (N consecutive failures in a row → skip the rest of the row);
   - occlusion in the swept mask, plus the ~0.48 m near-field blind zone;
   - a working switching trigger (suggested: "un-swept floor added since the last sweep exceeds
     X m²");
   - detect-only mode and detection-to-cube matching;
   - fixed coverage denominator.
2. **Decide the test arena:** several rooms or partitions (§8.3 condition).
3. **Build the proposed method:** region segmentation with frozen boundaries (and the open-space
   fallback), inside vs exit frontier classification, region "done" tracking, and the per-patch
   choice between rows and a viewpoint planner. Optionally run the rows vs viewpoints early
   warning first (§8.5).
4. **Build arm E, then arm D** (D shares the region code and the viewpoint planner).
5. **Hardware fixes (required before any counted run):** Kinect tilt and IR intrinsics, then
   check whether stalls drop. Stalls hit short frequent trips hardest, which arm D makes most, so
   this protects D from an unfair disadvantage.
6. **Layouts, taped start mark, tagged commit; run the blocks.**
7. **End-to-end collection demonstration** with the best strategy.

### 9.2 Open items

- Pending decisions: arm D fidelity (full vs light); abstract scope (depends on the deadline);
  whether to run the rows vs viewpoints early warning before the full build.
- Measure the real LiDAR mount height (URDF 0.24 m vs ~12 cm observed).
- Measure YOLO's frame rate on the Jetson.
- Read Gao et al. (ICRA 2024) and Kim et al. (RA-L 2022) closely before claiming
  retrieval-during-search or ground-robot target search as new.
- Search "coverage-aware costmap" / "inspection cost layer" before claiming the cost layer as new.
- Coverage-layer changes still uncommitted on `research-2`: `botzilla_coverage_layer/`,
  `frontier_explorer_node.py`, `executor_node.py`, `swept_mask.py`, `test_swept_mask.py`,
  `nav2_params.yaml`, `executor.launch.py`, `package.xml`, `run_full_mission.sh`.

---

## 10. References

1. Y. Luo, Z. Zhuang, N. Pan, C. Feng, S. Shen, F. Gao, H. Cheng, B. Zhou, "Star-Searcher: A
   complete and efficient aerial system for autonomous target search in complex unknown
   environments," *IEEE Robotics and Automation Letters*, vol. 9, no. 5, pp. 4329–4336, 2024.
2. H. Zhang, Y. Wang, W. Zhang, Y. Wang, H. Chen, "HEATS: A hierarchical framework for efficient
   autonomous target search with mobile manipulators," *IEEE/RSJ International Conference on
   Intelligent Robots and Systems (IROS)*, 2025 (arXiv:2503.07986).
3. J. Gao, P. Xie, X. Gao, Z. Sun, J. Wang, M. Q.-H. Meng, "Indoor exploration and simultaneous
   trolley collection through task-oriented environment partitioning," *ICRA*, 2024
   (arXiv:2309.11107).
4. M. Naazare, F. G. Rosas, D. Schulz, "Online next-best-view planner for 3D-exploration and
   inspection with a mobile manipulator robot," *IEEE Robotics and Automation Letters*, vol. 7,
   no. 2, pp. 3779–3786, 2022.
5. B. Zhou, Y. Zhang, X. Chen, S. Shen, "FUEL: Fast UAV exploration using incremental frontier
   structure and hierarchical planning," *IEEE Robotics and Automation Letters*, vol. 6, no. 2,
   pp. 779–786, 2021.
6. H. Kim, H. Kim, S. Lee, H. Lee, "Autonomous exploration in a cluttered environment for a mobile
   robot with 2D-map segmentation and object detection," *IEEE Robotics and Automation Letters*,
   vol. 7, no. 3, pp. 6343–6350, 2022.
7. L. Heng, A. Gotovos, A. Krause, M. Pollefeys, "Efficient visual exploration and coverage with a
   micro aerial vehicle in unknown environments," *ICRA*, pp. 1071–1078, 2015.
8. M. Luperto, T. P. Kucner, A. Tassi, M. Magnusson, F. Amigoni, "Robust structure identification
   and room segmentation of cluttered indoor environments from occupancy grid maps," *IEEE
   Robotics and Automation Letters*, vol. 7, no. 3, pp. 7974–7981, 2022.
9. B. Yamauchi, "A frontier-based approach for autonomous exploration," *CIRA*, pp. 146–151, 1997.
10. M. Selin, M. Tiger, D. Duberg, F. Heintz, P. Jensfelt, "Efficient autonomous exploration
    planning of large-scale 3-D environments," *IEEE Robotics and Automation Letters*, vol. 4,
    no. 2, pp. 1699–1706, 2019 (AEP).
11. R. T. Marler, J. S. Arora, "The weighted sum method for multi-objective optimization: new
    insights," *Structural and Multidisciplinary Optimization*, vol. 41, no. 6, pp. 853–862, 2010.
12. K. Helsgaun, "An effective implementation of the Lin–Kernighan traveling salesman heuristic,"
    *European Journal of Operational Research*, vol. 126, no. 1, pp. 106–130, 2000.
13. Y. Gabriely, E. Rimon, "Spiral-STC: An on-line coverage algorithm of grid environments by a
    mobile robot," *ICRA*, 2002.
14. E. Galceran, M. Carreras, "A survey on coverage path planning for robotics," *Robotics and
    Autonomous Systems*, vol. 61, no. 12, pp. 1258–1276, 2013.
15. L. Fermin-Leon, J. Neira, J. A. Castellanos, "Incremental contour-based topological
    segmentation for robot exploration," *ICRA*, pp. 2554–2561, 2017 (HEATS' region segmentation).
