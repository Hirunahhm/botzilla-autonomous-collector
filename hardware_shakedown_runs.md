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

Run 0 is **excluded**: it ran with the old world-anchored 4 m tiling (see §5.1), and the
timer failed to stop it, so it ran for about 8 minutes. It is listed because the region
bug was found in it.

The "dirty" code in runs 1, 3, 4, 5 and 5b is the fixes in §5, which were committed afterwards.
The shape split is in f1912d7; the look clearance change is still uncommitted as of this
note.

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
5. **C′ (interleaved + viewpoints) was the best run**: 19.8 m² seen in 5 minutes, AUC
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
7. **Decision time** was ~0.3–0.5 s per decision on the Jetson (0.7 s max). Acceptable,
   since decisions come every few tens of seconds.

---

## 7. Remaining shakedown runs

| # | Arm | Flags |
|---|---|---|
| 5b | C′: interleaved + viewpoints (new arm, added after run 5) | `--strategy interleaved --inspection viewpoints` |
| 6 | Ablation 1: viewpoints only | `--strategy region --inspection viewpoints` |
| 7 | Ablation 2: rows only | `--strategy region --inspection rows` |
| 8 | Ablation 3: one-look | `--strategy region --inspection one_look` |
| 9 | Spin grid | `--strategy region --inspection spin_grid` |

Each is started with `tools/timed_run.sh <label> 5 <flags>` from the taped start pose.

Run 5b is new. Arm C only inspects with rows. C′ keeps C's timing (explore, and inspect
whenever 3 m² of new unseen floor has appeared) but inspects with viewpoints with a turn
range, the same primitive as ablation 1. So C′ vs ablation 1 isolates region-by-region
ordering, and C′ vs C isolates rows vs viewpoints. Checked on the fake-Nav2 harness
first: it finished at 91% floor seen with 16 looks.
