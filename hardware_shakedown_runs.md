# Hardware Shakedown Runs (2026-09-25)

The first real-robot runs of the new search arms: one 5-minute run per arm, to check that
each arm behaves correctly on hardware **before** any counted runs. These are not results.
Each arm has one run, the runs are short, the code changed between runs (see the notes on
each run), and there were no cubes or layout. Use them to find problems, not to compare
methods.

Related: `research_discussion.md` (design and plan), `tools/analyze_runs.py` (metrics),
`tools/timed_run.sh` (how the runs were started).

---

## 1. Setup

| Item | Value |
|---|---|
| Place | The lab: one L-shaped space, no doorway between the two arms. About 59 m² of floor mapped (most mapped in any run). |
| Start | Same taped start mark and heading every run; the robot was reset by hand between runs. |
| Timer | 5 min from HOME latched (`tools/timed_run.sh LABEL 5 ...`), then SIGTERM. Metrics cut at exactly 5 min from the first EXPLORING state (`analyze_runs.py --budget-min 5`). |
| Mode | `--detect-only --metrics`. No `--layout`, so there are no per-cube inspected/detected times. |
| Planner | GridBased (NavFn) for frontier/transit goals, SweepStraight for row legs, coverage cost off. The defaults for every arm, so only the arm changed. |
| Floor area | Not tape-measured yet. See §4 for the fixed denominator used instead. |
| Battery | 14.7 V at the start of the day, 14.0 V at run 4 (4S Li-ion, ~16.5 V full). |

---

## 2. The runs

| # | Run dir | Arm | Flags | Commit | Dirty | Start |
|---|---|---|---|---|---|---|
| 0 | `20260925-154415` | Proposed (region, mixed), **old tiling** | `--strategy region --inspection mixed` | c8fa0f9 | no | 15:44 |
| 1 | `20260925-162128` | Proposed (region, mixed) | `--strategy region --inspection mixed` | c8fa0f9 + shape split | yes | 16:21 |
| 2 | `20260925-173335` | D: HEATS-style | `--strategy heats` | f1912d7 | no | 17:33 |
| 3 | `20260925-174630` | E: camera-greedy | `--strategy camera_greedy` | f1912d7 + look clearance 45 | yes | 17:46 |
| 4 | `20260925-175551` | B: explore-then-sweep | `--strategy sweep --policy exhaustion` | f1912d7 + look clearance 45 | yes | 17:55 |
| 5 | `20260925-181023` | C: interleaved, fixed (area) trigger | `--strategy sweep --policy area` | f1912d7 + look clearance 45 | yes | 18:10 |
| 5b | `20260925-182929` | C′: interleaved + viewpoints (new arm) | `--strategy interleaved --inspection viewpoints` | f1912d7 + look clearance 45 + interleaved arm | yes | 18:29 |
| 6 | `20260925-184005` | Ablation 1: region + viewpoints | `--strategy region --inspection viewpoints` | f1912d7 + look clearance 45 + interleaved arm | yes | 18:40 |
| 7 | `20260925-185103` | Ablation 2: region + rows only | `--strategy region --inspection rows` | f1912d7 + look clearance 45 + interleaved arm | yes | 18:51 |
| 7b | `20260925-190052` | Ablation 2 again (repeat of run 7) | `--strategy region --inspection rows` | same as run 7 | yes | 19:00 |
| 8 | `20260925-192138` | Ablation 3: region + one-look | `--strategy region --inspection one_look` | same as run 7 | yes | 19:21 |
| 9 | `20260925-204557` | Spin grid | `--strategy region --inspection spin_grid` | 2218500 (only this notes file uncommitted) | yes | 20:45 |
| 1c | `20260925-210557` | Proposed, §5.5 time rule | `--strategy region --inspection mixed` | 2218500 + §5.5 | yes | 21:05 |
| 1d | `20260925-211738` | Proposed, §5.6 measured costs | `--strategy region --inspection mixed` | 2218500 + §5.5–5.6 | yes | 21:17 |
| 5c | `20260925-212543` | Interleaved + mixed, §5.6 measured costs | `--strategy interleaved --inspection mixed` | 2218500 + §5.5–5.6 | yes | 21:25 |

Run 0 is **excluded**: it ran with the old world-anchored 4 m tiling (see §5.1), and the
timer failed to stop it, so it ran for about 8 minutes. It is listed because the region
bug was found in it.

The "dirty" code in runs 1, 3, 4, 5, 5b, 6, 7, 7b and 8 is the fixes in §5, which were
committed afterwards: the shape split in f1912d7; the look clearance change and the
interleaved arm in 2218500. Run 9 ran on 2218500; it is marked dirty only because this
notes file was being edited.

---

## 3. Numbers (first 5 minutes)

From `tools/analyze_runs.py --all --budget-min 5`, plus counts from each `executor.log`.
Coverage here uses each run's **own** mapped floor as the denominator (known-free cells at
that moment). §4 gives the fixed-denominator version, which is the one to quote.

| # | Arm | Coverage at 5 min (own map) | Coverage AUC (own map) | Distance | Turning | Stalls in 5 min | Looks done / aborted | Frontier goals |
|---|---|---|---|---|---|---|---|---|
| 1 | Proposed | 20.9% | 16.3% | 17.7 m | 1,691° | 2 | 0 / 0 (rows only, 4 row legs) | 4 |
| 2 | D: HEATS-style | 20.0% | 9.9% | 8.3 m | 2,198° | 2 | 40 / 4 | 0 |
| 3 | E: camera-greedy | 15.2% | 10.5% | 7.3 m | 1,719° | 2 | 21 / 0 | 0 |
| 4 | B: explore-then-sweep | 28.8% | 17.7% | 20.4 m | 1,993° | 2 | 0 / 0 | 13 |
| 5 | C: interleaved | 22.0% | 13.4% | 21.2 m | 1,889° | 4 | 0 / 0 (4 row, 6 transit legs) | 0 |
| 5b | C′: interleaved + viewpoints | 34.7% | 21.1% | 18.5 m | 3,293° | 1 | 8 / 3 | 0 |
| 6 | Ablation 1: region + viewpoints | 43.2% | 29.3% | 22.0 m | 3,684° | 2 | 7 / 2 | 2 |
| 7 | Ablation 2: region + rows only | 12.9% | 11.5% | 5.8 m | 630° | 5 | 0 / 0 (3 row, 3 transit legs) | 1 |
| 7b | Ablation 2 again | 13.9% | 10.1% | 9.8 m | 1,315° | 3 | 0 / 0 (4 row, 4 transit legs) | 0 |
| 8 | Ablation 3: region + one-look | 20.7% | 14.1% | 17.2 m | 1,744° | 1 | 0 / 0 (8 row, 8 transit legs) | 0 |
| 9 | Spin grid | 38.5% | 24.8% | 17.8 m | 3,344° | 2 | 5 / 1 | 2 |
| 1c | Proposed, §5.5 time rule | 18.6% | 13.1% | 14.3 m | 1,603° | 1 | 0 / 0 (4 row legs) | 2 |
| 1d | Proposed, §5.6 measured costs | 35.5% | 19.4% | 21.0 m | 3,066° | 2 | 10 / 2 | 1 |
| 5c | Interleaved + mixed | 42.9% | 24.6% | 25.6 m | 4,102° | 0 | 8 / 9 | 0 |
| (0) | Proposed, old tiling | 27.0% | 19.2% | 18.6 m | 1,737° | 4 | 0 / 0 | – |

Run 5 started its first sweep at 1 s (24.6 m² of unseen floor was already "new" at the start,
over the 3 m² trigger), so it swept for the whole window. Two rows were abandoned after
two consecutive failures each (y = −2.31 at 145 s, y = −1.46 at 215 s), which is the
row-abandonment rule working as intended.

Run 5b also started its first inspection bout at 1 s (23.8 m² of new unseen floor) and
spent the whole window in it, so within 5 minutes it never went back to exploring. Its
looks favoured turning: 5 full spins (303°), 2 × 180°, 1 × 120°, 6 × 60° (planned spans).
3 of 11 spins still ended in Nav2's "Collision Ahead" even with the stricter look
clearance (§5.3), so the clearance margin is not the whole story (see §6).

Run 6 selected the start region (19.8 m²), explored its inside frontiers (2 goals, one
stalled), froze it at 44 s with 36.1 m² known, and inspected it with viewpoints for the
rest of the window: 6 full spins, 2 × 240°, 2 × 120°, 1 × 60° (planned). It stayed in
its region the whole time; 2 of 9 spins aborted on "Collision Ahead".

Run 7 froze its region at 19 s (20.5 m² known) and began a 12-row pass. Every row leg it
tried stalled (6 stalls, 5 inside the window), and row abandonment dropped three rows
(y = −5.78, −4.93, −4.08). All of those rows are in the far lower arm, beside the wall
and clutter, for two reasons (see §6 item 8): the region was frozen before the L was
recognisable, so it probably included the part of the lower arm the LiDAR had glimpsed;
and the row generator always starts at the lowest row (smallest y), not the one nearest
the robot.

Run 7b repeated run 7 exactly. This time the region (17.3 m², frozen at 19 s) and its
rows stayed near the start (y = −1.5 to 0.2), not in the far lower arm, and the result
was almost the same: 6.8 m² seen, AUC 8.2%. So the far-corner rows of run 7 were not the
main cause. Even a 0.57 m row leg stalled for 30 s. `nav2.log` shows the same failure as
run 22: DWB finds "No valid trajectories out of 440" when the straight row line passes
close to furniture (the footprint would clip it), and the backup and spin recoveries then
also end in "Collision Ahead". The planner server also fell to 2 Hz (target 20 Hz) at
81 s, a sign of CPU load on the Jetson.

Run 8 froze its region at 20 s (20.3 m²) and began a 13-row pass over the large patches,
exactly as `mixed` does; the pass took the whole window, so it never reached its one-look
viewpoints (0 looks). Two rows were abandoned (y = −4.17, −0.77). Within 5 minutes,
run 8 therefore tested the same thing as run 1, and the two agree (11.0 m² each).

Run 9 explored its start region (2 frontier goals, one stalled) and froze it at 69 s
(24.4 m²). It then ran full spins at the points of a ~1 m grid inside it, nearest first:
5 done, 1 aborted on "Collision Ahead".

Every run's stalls were the 30 s no-progress watchdog on a Nav2 goal; none were
localisation or hardware faults.

---

## 4. Coverage against a fixed denominator

The percentages in §3 divide by the floor each run had mapped, so a run that mapped more
has a bigger denominator. Run 5b mapped the most (59.2 m²). Below, the same runs
are shown against that one fixed figure, and as absolute seen floor, which needs no
denominator. The figure is the largest seen so far and is updated as runs are added.
Mapped floor comes from each run's saved `map_final.npz`.

| Arm | Floor mapped by end of run | Floor seen at 5 min | Seen / own map | Seen / fixed 59.2 m² | AUC (fixed) |
|---|---|---|---|---|---|
| Proposed (region, mixed) | 52.6 m² | 11.0 m² | 20.9% | 18.6% | 13.3% |
| D: HEATS-style | 46.4 m² | 9.2 m² | 20.0% | 15.5% | 7.1% |
| E: camera-greedy | 55.0 m² | 8.4 m² | 15.2% | 14.1% | 8.7% |
| B: explore-then-sweep | 57.2 m² | 16.4 m² | 28.8% | 27.8% | 13.9% |
| C: interleaved, rows | 57.8 m² | 12.7 m² | 22.0% | 21.5% | 12.4% |
| C′: interleaved, viewpoints | 59.2 m² | 19.8 m² | 34.7% | 33.5% | 19.4% |
| Ablation 1: region, viewpoints | 50.9 m² | 22.0 m² | 43.2% | 37.2% | 24.1% |
| Ablation 2: region, rows only | 48.9 m² | 6.3 m² | 12.9% | 10.6% | 9.4% |
| Ablation 2 again (run 7b) | 50.6 m² | 6.8 m² | 13.9% | 11.6% | 8.2% |
| Ablation 3: region, one-look | 55.1 m² | 11.0 m² | 20.7% | 18.6% | 11.8% |
| Spin grid | 45.5 m² | 17.5 m² | 38.5% | 29.6% | 18.9% |
| Proposed, §5.5 time rule (run 1c) | 52.6 m² | 9.8 m² | 18.6% | 16.5% | 11.2% |
| Proposed, §5.6 measured costs (run 1d) | 54.7 m² | 19.3 m² | 35.5% | 32.6% | 15.4% |
| Interleaved + mixed (run 5c) | 57.6 m² | 24.4 m² | 42.9% | 41.3% | 22.8% |

"Floor mapped by end" is the whole run including shutdown, so slightly more than 5 min.
59.2 m² is a stand-in until the lab floor is tape-measured (both arms of the L, minus
large furniture).

---

## 5. What the runs showed, and what was changed

### 5.1 Regions: world-anchored tiles cut the L arbitrarily (run 0), fixed

- **Seen:** at ~446 s the robot drove from one arm of the L into the other.
- **Cause:**
  - The lab has no doorway, so the doorway test sees one room of ~67 m².
  - Over 40 m², the old code fell back to 4 m tiles on a grid **anchored at the start
    pose**, so the tile corners sat exactly on the start point and cut the lab into
    arbitrary squares.
  - The active region also shrank underneath the robot the moment the map passed 40 m²
    (a 24.5 m² room became a 7.8 m² tile).
  - When that tile reached 83% seen, the next nearest tile was in the other arm.
- **Fix** (`region_segmentation.py`): open space is split by **shape**, not tiles.
  - A region whose rectangularity (area / wall-aligned bounding box) is below 0.85 is cut
    at the straight line that leaves the two most rectangular parts. For an L, that is
    its two arms.
  - A rectangular region is only halved if it exceeds 40 m².
  - Cuts depend on the map only, never on the start pose.
  - The convex hull was tried first and rejected: an L's hull cuts across the inside
    corner, so a small L still looked convex.
  - Doorway splitting (gaps < 1 m) is unchanged.
- **Checked:**
  - On run 1's saved map, the upper part of the lab (~40 m²) is one region, and the lower
    arm is split off at the inside corner.
  - In run 1 the robot stayed in its arm for the whole 5 minutes.
  - Unit tests: an L splits into its arms, a furnished rectangle stays whole, and a
    100 m² open space is halved into four regardless of origin.

### 5.2 The timer could not stop a run (run 0), fixed

- `kill -INT` was ignored because a script started in the background ignores SIGINT, so
  the robot kept exploring for about 3 minutes past its timer.
- The wrapper also picked the wrong log folder: `run_logs` holds `sim-*` folders that
  sort last by name.
- `tools/timed_run.sh` now uses SIGTERM and finds the newest folder by time. Runs 1–4 all
  stopped on time.

### 5.3 Spins aborted near clutter (run 2), fixed

- 4 of 44 looks ended in Nav2's own "Collision Ahead - Exiting Spin".
- Viewpoints were allowed at published cost ≤ 50, which is 0.40 m from an obstacle, just
  inside the robot's 0.419 m turning radius.
- Now cost ≤ 45 (0.43 m). Run 3 then had 21 looks and 0 aborts.

### 5.4 The final map is now saved per run

- RTAB-Map does not keep the grid when the stack is stopped (run 0's map had to be
  rebuilt from stored scans, with odometry-only smearing).
- `mission_metrics_node` now writes `map_final.npz` (the `/map` grid plus the swept grid)
  to each run folder. §4 and §5.1 use it.
- `/explorer/regions` shows the live segmentation in RViz: active region black, other
  regions grey, done regions light.

---

### 5.5 `mixed` chose rows by patch size, now by estimated time

- **Seen:** in runs 1 and 8 the row pass took the whole 5 minutes and no viewpoint was
  ever used, while the viewpoint-only arms saw about twice the floor.
- **Cause:** `mixed` sent every unseen patch of at least 2 m² to rows. Right after a
  region is explored, the whole room is one patch, so the whole room went to rows.
- **Fix** (`search_strategies.RegionSearch._choose_rows`): for each patch ≥ 2 m², both
  primitives are timed and the faster one is used.
  - Rows: leg distance / 0.15 m/s + 4 s per stop, plus 30 s (one stall watchdog) for
    every "tight" leg whose line passes within 0.45 m of an obstacle away from its ends.
    Runs 7 and 7b showed tight legs stall nearly every time.
  - Viewpoints: `viewpoint_planning.estimate_look_time` runs the planner itself
    greedily (pick a look, mark what it sees, move there) until 90% of the patch is
    seen, capped at 20 looks.
  - Each decision is logged with both estimates, e.g. `patch 20.5 m^2: rows ~638 s
    (15 legs, 12 tight) vs viewpoints ~449 s (20 looks, 87% simulated) -> viewpoints`.
- **Checked:**
  - Offline: an open 2 × 12 m hall and an empty 6 × 6 m room get rows; a 6 × 6 m room
    with desks gets viewpoints. The lab's start region from run 6's saved map gets
    viewpoints (12 of its 15 row legs are tight).
  - Harness: both `region/mixed` and the new `interleaved/mixed` mixed rows and
    viewpoints and finished at 90% floor seen.
  - Unit tests: 100 pass. Decision time about 1–1.5 s, once per region or bout.
- `one_look` uses the same rule with one-look viewpoints; `interleaved` gets it through
  the shared inspection code.

### 5.6 Run 1c: the §5.5 rule chose rows for a patch, and rows took the whole run; costs recalibrated from measured legs

- **Run 1c** (`20260925-210557`, proposed with the §5.5 rule):
  - The frozen region (21.4 m²) had two unseen patches. The 9.9 m² patch went to
    viewpoints (rows ~342 s vs ~264 s). The 7.5 m² patch went to rows (~182 s vs ~237 s).
  - The row pass ran first and took about 290 s for 4 legs, not 182 s. Only 2 of those
    legs ended in a stall; the rest crawled. The viewpoints were never reached.
  - Result: 9.8 m² seen, AUC 11.2% (fixed 59.2 m²), 1 stall in the window.
- **Cause:** the model's leg costs were guesses (0.15 m/s + 4 s per stop). Measuring every
  leg across all of the day's runs showed how far off they were (distance from the pose
  at dispatch, time to the Nav2 result, failures included):

  | Leg | n | Succeeded | Mean distance | Mean time |
  |---|---|---|---|---|
  | Row, open (min clearance ≥ 0.45 m) | 5 | 40% | 1.77 m | 38.0 s |
  | Row, near furniture | 6 | 50% | 1.97 m | 41.7 s |
  | Row, short (< 0.9 m) | 10 | 60% | 0.61 m | 33.5 s |
  | Transit, all | 26 | 42% | 1.89 m | 32.8 s |
  | Viewpoint, short (< 0.9 m) | 45 | 89% | 0.50 m | 11.5 s |
  | Viewpoint, open | 8 | 88% | 1.69 m | 15.6 s |

  Row legs are slow **whether or not** they pass near furniture, so the "tight leg"
  penalty of §5.5 did not match the data. What does match is a large fixed cost per leg.
- **Fix:** the rows-vs-viewpoints comparison now uses fitted costs.
  - Row leg ≈ 31 s + 3.9 s/m; transit leg ≈ 27 s + 4.0 s/m.
  - Drive to a viewpoint ≈ 9.8 s + 3.4 s/m, plus the turn.
  - The tight-leg penalty is removed.
  - The viewpoint planner's own choice of looks is unchanged, so the viewpoint arms
    behave as in runs 5b, 6 and 9.
- **Decisions now:**
  - Rows win only when the legs are long and few: an open 2 × 12 m hall (~260 s vs
    ~529 s); an empty 6 × 6 m room narrowly (~612 s vs ~688 s).
  - The lab's start region gets viewpoints (run 6's map: ~591 s vs ~512 s; run 1c's
    map: ~852 s vs ~589 s).
- These fitted costs describe this robot and this lab on 2026-09-25. Refit them with the
  same script after the hardware fixes (Kinect tilt etc.), since stalls drive them.

### 5.7 Run 1d: the recalibrated `mixed` chose viewpoints

- At 62 s the frozen region (29.0 m²) was one 27.4 m² unseen patch: rows ~877 s
  (24 legs) vs viewpoints ~647 s → viewpoints. No row leg was driven.
- The looks were 7 full spins, 1 × 120°, 3 × 60° and 1 single look (planned spans);
  2 were aborted by "Collision Ahead".
- 19.3 m² seen, up from 11.0 m² (run 1, old `mixed`) and 9.8 m² (run 1c).
- AUC 15.4%, still below ablation 1's 24.1%. Almost all of that gap is the exploration
  phase: 62 s in run 1d (one frontier goal stalled 32 s) vs 44 s in run 6, so inspection
  started later. From the moment it inspected, run 1d behaved like ablation 1, which is
  what the rule should produce in this lab.

### 5.8 Run 5c: interleaved + mixed chose viewpoints, most floor seen, most spins aborted

- At 2 s the first inspection bout started (26.9 m² of new unseen floor). One 26.7 m²
  patch: rows ~1,027 s (30 legs) vs viewpoints ~680 s → viewpoints. No row leg was
  driven, and there were no stalls.
- 24.4 m² seen: the most of any run. AUC 22.8% (fixed), second only to ablation 1
  (24.1%).
- **9 of 17 spins ended in Nav2's "Collision Ahead"**, the highest rate of the day
  (run 5b 3/11, run 6 2/9, run 1d 2/12). An aborted spin still marks what it saw before
  stopping, so floor is not lost, but the wasted turns are now the largest time loss in
  the viewpoint arms. The footprint check at the spin's headings (§6 item 6) is needed
  before counted runs.

## 6. Observations to follow up

1. **Rows are slow on this robot, and they dominate "mixed" in a big room.**
   - Right after exploring, the whole arm is one big unseen patch, so the size rule
     ("rows for patches ≥ 2 m²") sends the entire 36 m² arm to rows.
   - Run 1 finished about 2 of 15 rows in 4 minutes and never reached its viewpoints.
   - Proposed change: choose rows or viewpoints per patch by **estimated time**, not
     patch size.
2. **B saw the most floor of runs 1–5** (C′ in run 5b saw more; see item 5).
   - It drives forward almost all the time, so the camera sweeps new floor as a side
     effect of mapping. It had not started its sweep phase yet.
   - One-look arms (D, E) pay a per-stop cost on this robot for a narrow view each time.
     C′'s spins pay the same stop cost but see a ring of floor per stop, which is what
     made the difference.
   - The 15-minute budget may change the order: B's sweep and the proposed method's
     viewpoints only start after the first few minutes.
3. **Stalls were ~2 per 5 minutes in arms 1–4 and 4 in arm C**, whose whole window was
   row and transit legs. This is more evidence that rows stall most on this robot.
   The hardware fixes (Kinect tilt, IR focal length) are still to do before counted runs.
4. **A wedge of floor seen through gaps between furniture** (7.3 m², LiDAR streaks) forms
   its own region. Watch whether it wastes time in longer runs.
5. **Viewpoints with spins beat everything else; region + viewpoints was best overall.**
   Run 6 (region + viewpoints) saw 22.0 m², AUC 24.1% (fixed); run 5b (interleaved +
   viewpoints) saw 19.8 m², AUC 19.4%. The same inspection primitive with region
   ordering came out ahead, which is the direction the proposed design predicts, but
   one run each cannot separate it from run-to-run spread. Run 5b alone: 19.8 m² seen in 5 minutes, AUC
   19.4%, against 11.0 m² / 13.3% for the proposed method (region, mixed) and 12.7 m² /
   12.4% for C with rows. Same timing as C, so the gain is from inspecting with
   viewpoints and spins instead of rows. It also had the fewest stalls (1). Single runs;
   the controlled comparison is ablation 1 (region + viewpoints) vs C′.
6. **Spins still abort near clutter** (3 of 11 in run 5b) with look clearance 45. A
   planner check at one cell's costmap value does not cover the robot's asymmetric
   footprint while it turns, and the local costmap sees obstacles the global one does
   not. A partly aborted spin still marks what it saw. Possible fix: check the full
   footprint at the spin's headings against the local costmap before choosing a
   viewpoint.
7. **Rows-only was the weakest arm, and it repeats** (run 7: 6.3 m², AUC 9.4%;
   run 7b: 6.8 m², AUC 8.2%). The cause is DWB rejecting every trajectory when a straight
   row passes close to furniture (see run 7b's note), not only the row order or where
   the region was.
   Together with runs 1, 5 and 6, this is consistent: on this robot in this lab,
   every arm that inspects with rows lost to the same timing with viewpoints.
8. **`mixed` and `one_look` cannot be told apart in 5 minutes here.** Both start with a
   row pass over patches ≥ 2 m², and right after exploring the frozen region is one big
   patch, so the pass took the whole window in runs 1 and 8 (no looks at all). They
   would only differ in the 15-minute budget, or if the row pass is shortened or
   replaced (see item 1).
9. **Two fixable causes behind run 7's poor rows**:
   - **Row order ignores the robot's position.** `generate_coverage_waypoints` always
     starts at the lowest row and zig-zags up, so the first row can be the farthest
     one. This affects arms B, C and the proposed method's row pass. Fix: start from
     whichever end of the row list is nearer the robot.
   - **A region frozen before its shape is known.** At 19 s the lower arm was only
     partly mapped (floor seen through the opening), so the L was not yet recognisable
     and the frozen region likely included part of the other arm. Possible fix:
     re-segment the frozen cells once more floor is mapped and drop cells that now
     belong to a different part, or delay freezing until the region's boundary has
     stopped changing.
10. **Decision time** was ~0.3–0.5 s per decision on the Jetson (0.7 s max). Acceptable,
   since decisions come every few tens of seconds.

---

## 7. Standings after all shakedown runs

Floor seen in the first 5 minutes against the fixed 59.2 m², best first by AUC. One run
per arm (two for rows only), so read the ordering as a first look, not a result.

| Arm | Inspection | Floor seen | Seen / 59.2 m² | AUC (fixed) | Stalls |
|---|---|---|---|---|---|
| Ablation 1: region + viewpoints | viewpoints with turn range | 22.0 m² | 37.2% | 24.1% | 2 |
| C′: interleaved + viewpoints | viewpoints with turn range | 19.8 m² | 33.5% | 19.4% | 1 |
| Spin grid | full spins on a 1 m grid | 17.5 m² | 29.6% | 18.9% | 2 |
| B: explore-then-sweep | (still exploring at 5 min) | 16.4 m² | 27.8% | 13.9% | 2 |
| Proposed (region, mixed) | row pass (never reached viewpoints) | 11.0 m² | 18.6% | 13.3% | 2 |
| C: interleaved + rows | rows | 12.7 m² | 21.5% | 12.4% | 4 |
| Ablation 3: region + one-look | row pass (never reached looks) | 11.0 m² | 18.6% | 11.8% | 1 |
| Ablation 2: region + rows only (2 runs) | rows | 6.3 / 6.8 m² | 10.6 / 11.6% | 9.4 / 8.2% | 5 / 3 |
| E: camera-greedy | one look per stop | 8.4 m² | 14.1% | 8.7% | 2 |
| D: HEATS-style | one look per stop | 9.2 m² | 15.5% | 7.1% | 2 |

What the shakedown says, before any counted runs:

1. **Turning at a stop beats both rows and one look per stop.** The three arms that turn
   at their stops (viewpoints with a turn range, and the spin grid) are the top three.
   Rows stall near furniture (DWB rejects every trajectory), and one-look arms pay a full
   stop for a narrow view.
2. **With the same viewpoints, region ordering came out ahead of interleaving** (24.1% vs
   19.4% AUC). This is the direction the proposed design predicts, but it is one run each.
3. **The proposed method as currently built (`mixed`) is held back by its row pass**,
   which takes the whole 5 minutes in this lab before any viewpoint. The planned change
   (choose rows or viewpoints per patch by estimated time, which here would mean
   viewpoints) would make `mixed` behave like ablation 1 in this lab.
4. **Planned viewpoints beat a fixed spin grid** (24.1% vs 18.9% AUC), which supports
   choosing viewpoints and turn ranges over spinning everywhere.

Next hardware runs, after the §5.5 fix:

| # | Arm | Flags |
|---|---|---|
| 1c | Proposed, with the §5.5 rule | done: 9.8 m², AUC 11.2% (rows chosen for one patch, see §5.6) |
| 1d | Proposed, with the §5.6 measured costs | done: 19.3 m², AUC 15.4%, chose viewpoints (§5.7) |
| 5c | Interleaved + mixed, with the §5.6 measured costs | done: 24.4 m², AUC 22.8%, chose viewpoints; 9/17 spins aborted (§5.8) |

Run 1d tests whether the recalibrated `mixed` reaches the viewpoint arms' level in this lab.
Run 5c completes the 2 × 3 grid (region / interleaved × rows / viewpoints / mixed).

Before counted runs (from §6): row order from the
robot's end, the footprint check for spins, the Kinect tilt / IR focal length fixes, a
tape-measured floor area, cube layouts, and the 15-minute budget.
