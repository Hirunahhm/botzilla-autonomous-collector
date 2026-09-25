# Full-Cycle Runs: 15 minutes, cubes on the floor (from 2026-09-25)

The robot runs the complete mission in each run: search, detect, chase, capture, deliver
to HOME, then continue. One 15-minute run per configuration, eleven configurations in all
(the plan agreed on 2026-09-25). Results from shorter shakedown runs, and the fixes that
led here, are in `hardware_shakedown_runs.md`.

## 1. Setup

| Item | Value |
|---|---|
| Place | The L-shaped lab (about 59 m² of mappable floor) |
| HOME | The taped start mark and heading, the same in every run (also the map origin) |
| Cubes | 6 cubes, **in the same places in every run**, put back after each run. Positions not measured yet. **4 are in the lab; 2 are in another room**, which no arm is expected to reach in 15 minutes, so 4 is the realistic maximum per run. With delivered counts tied at 4, the comparison is the delivery times (especially the 4th). |
| Timer | 15 min from HOME latched: `tools/timed_run.sh <label> 15 --collect <flags>` |
| Mode | Full mission (`--collect`, no `--detect-only`), metrics on, no `--layout` |
| Planner | GridBased for frontier/transit/delivery goals, SweepStraight for row legs, coverage cost off |

**Layout later:** once the cube positions are tape-measured from the start mark, run
`tools/analyze_runs.py <runs> --all --layout layouts/lab.yaml`. It recomputes per-cube
inspected and detected times from the logged poses, detections and `map_final.npz`. This
is valid because HOME and the cubes stayed put across runs.

**Metrics for these runs:**
- Cubes delivered within 15 min, and the time of each delivery.
- Chases started (entries into TARGETING) and chases lost.
- Floor seen (against the fixed 59.2 m² from the shakedown) and its AUC.
- Distance, turning, stalls, looks, battery.

---

## 2. Runs

| # | Run dir | Arm | Flags | Commit | Start |
|---|---|---|---|---|---|
| F1 | `20260925-222605` | Proposed (region, mixed) | `--strategy region --inspection mixed` | bdcd88e + carrying tree (dirty) | 22:26 |
| F2 | `20260925-224557` | Interleaved + mixed | `--strategy interleaved --inspection mixed` | bdcd88e + carrying tree (dirty) | 22:45 |
| (F1 old) | `20260925-214416` | Proposed, **before** the carrying fix, superseded | same | bdcd88e (clean) | 21:44 |

Runs from F2 on include the carrying recovery tree (§4, F1 finding 2).

**Excluded attempts** (not counted, logs kept):

| Run dir | What happened |
|---|---|
| `20260925-220805` | F1 repeat with the carrying tree; stopped by hand at ~1.5 min (the full-speed start-up spin sounded harsh; logs showed the same 0.4 rad/s spin rate as every run that day). |
| `20260925-221354` | F1 repeat started from a **different HOME** than F1, so its map frame does not match; aborted at ~8 min. Had delivered 2 cubes by 136 s. |

---

## 3. Results (15 minutes from HOME latched)

AUC here is on the fixed 59.2 m² basis. Floor seen at 5 / 10 / 15 min is in m².

| # | Arm | Delivered | Delivery times | Chases (lost) | Seen 5 / 10 / 15 min | Seen / 59.2 m² | AUC (fixed) | Distance | Stalls | Looks done / tried |
|---|---|---|---|---|---|---|---|---|---|---|
| F1 | Proposed (region, mixed) | **4** | 136, 239, 310, 562 s | 7 (3) | 15.0 / 24.7 / 27.4 | 46.4% | 30.8% | 57.1 m | 6 | 20 / 30 |
| F2 | Interleaved + mixed | **4** | 117, 391, 504, 608 s | 5 (1) | 17.1 / 27.7 / 29.0 | 49.0% | 35.2% | 58.7 m | 4 | 22 / 29 |
| (F1 old) | Proposed, before carrying fix | 2 (+1 in transit) | 152 s, 330 s | 6 (3) | – / – / 23.9 | 40.3% | 27.0% (known-free) | 56.2 m | 1 | 20 |

---

## 4. Run notes

### F1: proposed (region, mixed), `20260925-222605`

- Region 1 was selected at 3 s (20.7 m²) and explored at 138 s (36.3 m² frozen); its
  28.0 m² unseen patch went to viewpoints (~773 s vs ~1,241 s for rows).
- Deliveries at 136, 239, 310 and 562 s. All 4 went home with no recovery.
- 5 BackUp recoveries, all while exploring (47, 189, 207, 615, 824 s): **none while
  carrying**, so the carrying tree did its job.
- 3 chases lost: near (1.0, −2.5) while centring, (−3.8, 0.7) while driving in, and
  (0.2, 2.0) while centring.
- The whole run stayed in region 1 (74% seen at 890 s). Its last 5.5 minutes had one
  lost chase, 4 stalls and slow looks, and no further delivery. The slow-tail problem
  of the F1 findings below again kept the robot out of the rest of the L.
- Coverage of the mapped floor: 32% at 5 min, 46% at 10 min, 51% at 15 min. Battery
  14.3 → 14.1 V.

### F2: interleaved + mixed, `20260925-224557`

- The first inspection bout started at 2 s (26.3 m² of new unseen floor). The patch went
  to viewpoints (~702 s vs ~1,404 s for rows), and no row leg was driven all run.
- Deliveries at 117, 391, 504 and 608 s. No BackUp while carrying; 4 BackUps while
  exploring.
- 1 chase lost, near (2.0, −2.9) at 753 s while centring.
- It mapped the lab faster than F1 (51 m² known after 1 min vs 36 m²), as expected with
  no region restriction.
- Floor seen flattened after 10 min: 27.7 m² at 10 min, 29.0 m² at 15. The last 5 minutes
  had 3 stalls in the lower part of the lab (y ≈ −2.5 to −3.5) and no new cube.
- Time delivering: 177 s (F1: 105 s), consistent with captures further from HOME.
- Looks completed: 7 single looks, 8 × 60°, 3 × 120°, 4 full spins. Battery 14.2 → 14.0 V.

**F1 vs F2** (one run each): same deliveries (4), and F2 slightly ahead on floor seen
(29.0 vs 27.4 m²) and AUC (35.2 vs 30.8%). Both used viewpoints only. Both stopped
finding cubes after about 10 minutes; the last 2 cubes were not found by either.

### F1 old: proposed, before the carrying fix, `20260925-214416`

- 10 s: region 1 (19.1 m²) explored; the one 17.7 m² unseen patch went to viewpoints
  (~543 s vs ~771 s for rows).
- Cube 1: spotted at 54 s, captured at 63 s, delivered at 146 s. Cube 2: spotted at
  285 s, captured at 315 s, delivered at 325 s.
- 3 chases were lost while centring the cube (TARGETING timeout), each published to
  `/cube_abandoned`:
  - twice the same cube near (−0.2, −3.0), at 583 s and 601 s;
  - once a cube near (1.9, 1.6), at 800 s.
- 828 s: region 1 done at 91% seen, and the robot moved to region 2 (13.2 m²).
- 867 s: cube 3 spotted and captured; the timer ended during its delivery.
- 3 of 20 looks were aborted by "Collision Ahead"; 1 stall.

**Findings:**
1. **The region stays active too long in a furnished room.** Going from 80% to 91% seen
   took about 5 minutes (looks finding 0.2–0.6 m² each, above the 0.1 m² "worth a stop"
   floor). Region 1 used 13.8 of the 15 minutes, and the other arm of the L got one
   minute. Proposed fix: finish a region when the best remaining look's rate (m²/s) falls
   below a fraction of the region's average rate so far (a marginal-value rule), rather
   than at a fixed 90%.
2. **Nav2's BackUp recovery reversed the robot while it carried a cube**, twice (104 s
   during delivery 1, 916 s during delivery 3). The arms only hold a cube while moving
   forward, so reversing can pull it out. **Fixed before F2:** the HOME goal now uses
   `behavior_trees/navigate_to_pose_carrying.xml`, whose only recovery is
   clear-costmaps-then-Spin. If the spin is impossible, the goal aborts and the executor
   releases the cube where it is and resumes exploring. All other goals keep the
   BackUp-first recovery. F1 ran before this fix.
3. **Half of the chases were lost while centring.** These are detections at range or at
   the image edge that stop firing once the robot turns towards them. This is detector
   behaviour, so it affects every arm alike, but it costs time (each loss about 5 s plus
   the resume).
