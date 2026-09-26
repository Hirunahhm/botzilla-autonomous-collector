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
| Power | Kobuki base: its own battery (voltage logged as `/battery`). Jetson: a separate 4S pack, **not visible to software**; charge it before each session and check it by hand. |

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
| F3 | `20260926-001439` | Region + viewpoints (ablation 1) | `--strategy region --inspection viewpoints` | 4876c86 (only notes/abstract uncommitted) | 00:14 |
| F4 | `20260926-153034` | Interleaved + viewpoints (C′) | `--strategy interleaved --inspection viewpoints` | 4876c86 + delivery watchdog (dirty) | 15:30 |
| F5 | `20260926-155118` | B: explore, then sweep | `--strategy sweep --policy exhaustion` | 4876c86 + delivery watchdog (dirty) | 15:51 |
| F6 | `20260926-164237` | C: interleaved rows (area trigger) | `--strategy sweep --policy area` | 4876c86 + watchdog + count fix (dirty) | 16:42 |
| F7 | `20260926-170454` | D: HEATS-style | `--strategy heats` | 4876c86 + watchdog + count fix (dirty) | 17:04 |
| F8 | `20260926-172525` | E: camera-greedy | `--strategy camera_greedy` | 4876c86 + watchdog + count fix (dirty) | 17:25 |
| F9 | `20260926-190103` | Region + rows only (ablation 2) | `--strategy region --inspection rows` | 4876c86 + watchdog + count fix (dirty) | 19:01 |
| F10 | `20260926-194050` | Region + one look (ablation 3) | `--strategy region --inspection one_look` | 4876c86 + watchdog + count fix + per-spot limit (dirty) | 19:40 |
| F11 | `20260926-200334` | Region + spin grid | `--strategy region --inspection spin_grid` | 4876c86 + watchdog + count fix + per-spot limit (dirty) | 20:03 |
| (F1 old) | `20260925-214416` | Proposed, **before** the carrying fix, superseded | same | bdcd88e (clean) | 21:44 |

Runs from F2 on include the carrying recovery tree (§4, F1 finding 2).
**Per-spot limit from F10 on:** after 2 failures within 0.5 m of a spot (a chase lost
while targeting/approaching, or a cube released short of HOME), `executor_node` ignores
detections there for 180 s, then clears the spot. Added after F4's chase loops and F9's
stuck-capture loop. F1–F9 ran without it.

**Excluded attempts** (not counted, logs kept):

| Run dir | What happened |
|---|---|
| `20260925-220805` | F1 repeat with the carrying tree; stopped by hand at ~1.5 min (the full-speed start-up spin sounded harsh; logs showed the same 0.4 rad/s spin rate as every run that day). |
| `20260925-221354` | F1 repeat started from a **different HOME** than F1, so its map frame does not match; aborted at ~8 min. Had delivered 2 cubes by 136 s. |
| `20260925-231928` | **F3 (region + viewpoints), incomplete:** the Jetson shut down on under-voltage at ~12.7 min. By then: 3 delivered (116, 311, 504 s), 4 chases, 17.7 / 26.7 / 28.7 m² seen at 5 / 10 / 12 min, 4 stalls, 61.2 m, only 5 of 15 looks completed. Kobuki battery 14.0 → 13.8 V (this is the base's own battery, not the Jetson's). The Jetson runs from a **separate 4S pack** that no software on the robot can read; after about 4 hours of runs it most likely sagged below the carrier board's input limit under load. No `map_final.npz` (saved only at shutdown); the metrics file ends in unwritten null bytes, which the parser skips. To be rerun on a charged battery. |
| `20260926-151617` | **F4 (interleaved + viewpoints), stopped by hand at ~9 min** because the robot sat still holding a cube. Deliveries at 82 and 157 s; 3rd cube captured at 309 s. During that delivery the controller could not progress (319 s), the carrying tree's recovery spin was refused ("Collision Ahead", 330 s), and **bt_navigator then never finished the goal** (no abort, no new path, no result), so the executor waited. **Fixed before the rerun:** a delivery no-progress watchdog in `executor_node` (no progress ≥ 0.15 m towards HOME for 45 s → cancel, release the cube, resume exploring), instead of relying on the 300 s delivery timeout. |

---

## 3. Results (15 minutes from HOME latched)

AUC here is on the fixed 59.2 m² basis. Floor seen at 5 / 10 / 15 min is in m².

| # | Arm | Delivered | Delivery times | Chases (lost) | Seen 5 / 10 / 15 min | Seen / 59.2 m² | AUC (fixed) | Distance | Stalls | Looks done / tried |
|---|---|---|---|---|---|---|---|---|---|---|
| F1 | Proposed (region, mixed) | **4** | 136, 239, 310, 562 s | 7 (3) | 15.0 / 24.7 / 27.4 | 46.4% | 30.8% | 57.1 m | 6 | 20 / 30 |
| F2 | Interleaved + mixed | **4** | 117, 391, 504, 608 s | 5 (1) | 17.1 / 27.7 / 29.0 | 49.0% | 35.2% | 58.7 m | 4 | 22 / 29 |
| F3 | Region + viewpoints | **2** | 97, 262 s | 4 (2) | 20.1 / 26.9 / 30.4 | 51.4% | 36.4% | 62.5 m | 2 | 23 / 36 |
| F4 | Interleaved + viewpoints | **3** | 110, 172, 439 s | 21 (18) | 13.3 / 23.2 / 25.0 | 42.2% | 29.3% | 46.8 m | 4 | 30 / 40 |
| F5 | B: explore, then sweep | **4** (+1 released short) | 172, 259, 563, 780 s | 6 (1) | 13.8 / 18.2 / 22.4 | 37.8% | 24.3% | 54.3 m | 5 | – (no looks) |
| F6 | C: interleaved rows | **3** (+1 released short) | 595, 708, 782 s | 4 (0) | 10.4 / 21.3 / 26.4 | 44.6% | 25.8% | 59.7 m | 5 | – (rows) |
| F7 | D: HEATS-style | **2** | 546, 718 s | 5 (3) | 7.0 / 17.0 / 23.9 | 40.4% | 21.1% | 32.8 m | 2 | 111 / 116 |
| F8 | E: camera-greedy | **4** (+1 released short) | 154, 310, 422, 786 s | 6 (1) | 12.0 / 22.4 / 30.6 | 51.7% | 29.8% | 54.2 m | 2 | 55 / 62 |
| F9 | Region + rows only | **2** (+4 released short) | 714, 816 s | 8 (2) | 10.0 / 13.9 / 26.2 | 44.2% | 23.0% | 54.0 m | 4 | – (rows) |
| F10 | Region + one look | **2** | 84, 190 s | 5 (3) | 14.4 / 20.3 / 29.1 | 49.1% | 29.6% | 46.3 m | 1 | 81 / 88 |
| F11 | Region + spin grid | **5** counted (**4** distinct, +1 released short) | 118, 285*, 382, 536, 807 s | 6 (1) | 14.2 / 24.0 / 29.5 | 49.9% | 31.5% | 67.2 m | 5 | 6 / 13 |

\* re-delivery of a cube already at HOME — see §3.1.

### 3.1 Correction: re-collected cubes at HOME

`executor_node` suppresses detections only while the **robot** is within 1 m of HOME. A
cube already delivered can still be seen from further away, chased, and "delivered"
again. Checking every capture's position (robot pose when DELIVERING starts):

| Run | Capture near HOME | Counted | **Distinct cubes delivered** |
|---|---|---|---|
| F1 | 224 s, 0.51 m from HOME | 4 | **3** (136, 310, 562 s) |
| F6 | 693 s, 0.37 m from HOME | 3 | **2** (595, 782 s) |
| F11 | 270 s, 0.49 m from HOME | 5 | **4** (118, 382, 536, 807 s) |

All other full runs had no capture within 1.2 m of HOME. **Fix needed before repeats:**
ignore detections whose projected position is within ~1 m of HOME.

### 3.2 Summary (distinct cubes, one run per arm)

| Run | Arm | Distinct cubes | 4th cube at | Seen at 15 min | AUC |
|---|---|---|---|---|---|
| F2 | Interleaved + mixed | **4** | **608 s** | 29.0 m² | 35.2% |
| F5 | B: explore then sweep | **4** | 780 s | 22.4 m² | 24.3% |
| F8 | E: camera-greedy | **4** | 786 s | 30.6 m² | 29.8% |
| F11 | Region + spin grid | **4** | 807 s | 29.5 m² | 31.5% |
| F1 | Proposed: region + mixed | 3 | – | 27.4 m² | 30.8% |
| F4 | Interleaved + viewpoints | 3 | – | 25.0 m² | 29.3% |
| F3 | Region + viewpoints | 2 | – | 30.4 m² | **36.4%** |
| F6 | C: interleaved rows | 2 | – | 26.4 m² | 25.8% |
| F7 | D: HEATS-style | 2 | – | 23.9 m² | 21.1% |
| F9 | Region + rows only | 2 | – | 26.2 m² | 23.0% |
| F10 | Region + one look | 2 | – | 29.1 m² | 29.6% |

- Coverage: arms that stop and turn lead (AUC 30–36%); rows (23–26%) and single looks
  (21–30%) trail. Consistent with the shakedown.
- Cube counts are noisy at n = 1: lost chases (detector), a wedged cube (F9) and chase
  loops (F4) move them by 1–2 cubes, independently of the search strategy.
- Interleaved + mixed (F2) delivered all 4 reachable cubes fastest, with the
  second-best coverage.
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

### F3: region + viewpoints (ablation 1), `20260926-001439`

- Rerun of the incomplete F3 on a fully charged Jetson pack, with NoMachine stopped
  first (it had restarted with the Jetson at top priority).
- Region 1 was selected at 4 s (20.7 m²) and explored at 48 s (36.7 m² frozen); the rest
  of the run was spent in it, inspecting with viewpoints.
- Deliveries at 97 and 262 s. Then the same cube near (−3.4, −1.0) was chased and lost
  twice while centring (281 s, 295 s), and revisit points were published for it.
- **After 300 s no detection within 1 m occurred at all:** 31 detections in the last
  10 minutes, all beyond 1 m, while floor seen rose from 20 to 30 m². The
  lower delivery count against F1/F2 is therefore a detection outcome, not a search
  outcome.
- The highest floor seen and AUC of the full runs so far (30.4 m², 36.4%). Looks
  completed: 6 single, 7 × 60°, 5 × 120°, 2 × 180°, 1 × 240°, 2 full spins. 2 stalls; 4
  BackUps, all while exploring. Kobuki battery 15.3 → 15.0 V.
- With the cube positions measured, `--layout` will show whether the camera ever pointed
  at the two cubes it did not deliver (strategy miss) or pointed at them and YOLO did not
  fire (detector miss).

### F4: interleaved + viewpoints (C′), `20260926-153034`

- Rerun after the delivery watchdog was added; every delivery reached HOME on its own, so
  the watchdog did not trigger.
- Deliveries at 110, 172 and 439 s.
- **Chase loops from 626 s to the end:** the same two spots were chased and lost over and
  over, (1.75, −4.26) five times and (1.88, −3.07) twelve times. 104 s was spent centring
  on cubes (F1: 26 s, F2: 9 s, F3: 15 s). Each loop: YOLO fires at ~1 m, the robot turns
  to centre, the detection drops within ~5 s, the chase is abandoned, the cube is seen
  again at once. The executor has no memory of lost chases (the explorer's revisit limit
  of 2 per spot does not stop the executor reacting to raw detections).
- The spot near (1.9, −3.0) also cost a lost chase in F2 (2.0, −2.9) and F1 (1.0, −2.5):
  something there repeatedly triggers YOLO but cannot be captured. To check: whether it
  is one of the six cubes and reachable, or a false positive.
- Floor seen 25.0 m², AUC 29.3%, the lowest of the full runs, largely because the
  last 4.5 minutes were spent in the loops. 4 stalls; Kobuki battery 14.6 → 14.4 V.
- Proposed fix (not applied yet): after 2 lost chases within 0.5 m of a spot, ignore
  detections projected there for a few minutes.

### F5: B, explore then sweep, `20260926-155118`

- Never finished exploring within the 15 minutes (no switch to its sweep phase), so this
  run is frontier exploration plus collection.
- Deliveries at HOME at 172, 259, 563 and 780 s.
- **The delivery watchdog worked on hardware:** at 706 s, 45 s without progress towards
  HOME (still 5.16 m away), it cancelled the goal and released the cube; the robot
  re-captured the same cube at 719 s and delivered it at 775 s.
- **Count correction:** the executor counted every release as a delivery, so F5's log
  says 5. The true count is 4, plus 1 release short of HOME. F1–F4 are unaffected (every
  counted release there followed "Arrived HOME"). **Fixed after F5:** only a release at
  HOME counts; short releases are counted separately (`released_short` in
  `/mission/status`).
- Floor seen 22.4 m², AUC 24.3%, the lowest of the full runs: exploration drives along
  frontiers, and the camera sees floor only as a side effect.
- 221 s spent delivering (long trips from the lower part of the lab); 5 stalls, 4 of
  them on frontier goals in the lower part (y ≈ −2.4 to −4.5); 1 lost chase. Kobuki
  battery 14.4 → 14.2 V.

### F6: C, interleaved rows, `20260926-164237`

- First run with the delivery-count fix.
- The first sweep started at 1 s over all 26 m² then visible (26 row waypoints) and lasted
  until 669 s. One row was abandoned after two failures (y = −2.29); 5 stalls in all.
- The delivery watchdog released one cube at 346 s, 3.14 m from HOME, after 45 s without
  progress; it was counted as released short, not delivered.
- Deliveries at HOME at 595, 708 and 782 s; no chase lost.
- Least floor seen at 5 min (10.4 m²) of all full runs, rising to 26.4 m² by 15 min;
  AUC 25.8%. Kobuki battery 14.1 → 13.9 V.

### F7: D, HEATS-style, `20260926-170454`

- HEATS tour over 2 regions from 7 s; 116 one-look viewpoints (111 completed), only
  32.8 m driven. Coverage built slowly: 7.0 m² at 5 min, the least of all full runs; AUC
  21.1%.
- First cube found at 506 s; deliveries at HOME at 546 and 718 s.
- **Chased a cube already at HOME:** three lost chases at 849–889 s at about
  (0.35, 0.13), under 0.4 m from HOME, then a capture at 914 s delivered at 924 s (after
  the 900 s limit, so not counted). Almost certainly a cube it had delivered.
  `executor_node` suppresses detections only while the **robot** is within 1 m of HOME,
  not detections **located** at HOME; D lingers near HOME more than other arms, which
  exposed it. Fix to consider: ignore detections whose projected position is within the
  HOME radius.
- 2 stalls. Kobuki battery 13.9 → 13.7 V: charge before F8.

### F8: E, camera-greedy, `20260926-172525`

- 62 one-look viewpoints (55 completed), no regions. Slower start than the viewpoint arms
  (12.0 m² at 5 min) but the most floor seen by 15 min of any full run (30.6 m²);
  AUC 29.8%.
- Deliveries at HOME at 154, 310, 422 and 786 s; 1 chase lost near (0.04, −1.84).
- **The carrying tree's abort path worked as designed here:** at 733 s the delivery
  stalled, the recovery spin was refused ("Collision Ahead"), bt_navigator reported
  "Goal failed" at once, and the executor released the cube (released short). It was
  re-captured at 745 s and delivered at 781 s. So F4's navigator hang was intermittent;
  the delivery watchdog covers the cases where it does hang.
- 2 stalls. **Kobuki battery 13.7 → 13.4 V**, just above its ~13.2 V low point: charge
  before F9.

### F9: region + rows only (ablation 2), `20260926-190103`

- NoMachine had restarted with the Jetson and was stopped before the run.
- Region 1 (20.6 m²) frozen at 19 s; row pass of 11 runs until 643 s, ending at 61%
  seen. Region 2 (22.0 m²) then got a 9-run row pass.
- **Stuck-capture loop, 150–377 s:** four times the robot captured a cube at the same
  spot, about (−0.3, −2.6), and could neither drive on nor turn (the controller aborted, the
  carrying spin was refused, and the navigator hung again as in F4). The delivery watchdog
  released the cube after 45 s each time; the robot drove only 0.03–0.16 m per attempt.
  After each release it saw the cube again and re-captured it. About 4 minutes lost; 4
  releases short of HOME, most likely the same cube.
- Deliveries at HOME at 714 and 816 s; 2 chases lost near (−3.4, −0.9).
- Floor seen flat from 2 to 7 min (9.5 → 10.6 m²) during the loop; 26.2 m² by 15 min;
  AUC 23.0%. 4 stalls; controller at 6.4 Hz at one point (CPU load). Kobuki battery
  15.4 → 15.0 V.
- Second run to lose minutes to one cube it could not collect (F4: chase loops). Proposed
  fix: after 2 failures (lost chase or release short) within 0.5 m of a spot, ignore
  detections there for 3 minutes.

### F10: region + one look (ablation 3), `20260926-194050`

- First run with the per-spot limit. The rows-vs-viewpoints rule chose one-look
  viewpoints for the 28.8 m² patch (~929 s vs ~1,187 s for rows), so unlike the
  shakedown, this run did test one look per stop (88 looks, 81 completed).
- Deliveries at HOME at 84 and 190 s.
- **The per-spot limit worked:** after 2 lost chases near (−3.67, 0.95) (737 s, 761 s) the
  spot was ignored for 180 s; no further chases there.
- One lost chase at (0.29, −0.17), about 0.3 m from HOME, almost certainly one of its
  own delivered cubes (the HOME filter only acts while the robot is within 1 m of HOME;
  same issue as F7).
- Against F3 (same region order, turning viewpoints): AUC 29.6% vs 36.4%, 14.4 vs
  20.1 m² at 5 min, with 81 stops vs 23. Against D (F7, one look, blended score): clearly
  ahead (29.6% vs 21.1%). 1 stall. Kobuki battery 14.7 → 14.5 V.

### F11: region + spin grid, `20260926-200334`

- Full spins on a ~1 m grid inside each region; 13 spins tried, 6 completed. Region 1
  (36.6 m²) done at 599 s ("spin grid exhausted", 65% seen); then smaller regions and two
  corridors at 731 s and 849 s.
- Deliveries at HOME at 118, 382, 536 and 807 s, plus a re-delivery at 285 s of the cube
  delivered at 112 s (captured 0.49 m from HOME; see §3.1). One release short at 763 s
  (the carrying spin was refused, the goal failed at once), re-captured at 775 s.
- Floor seen 29.5 m², AUC 31.5%; 67.2 m driven, the most of any run; 5 stalls. Kobuki
  battery 14.5 → 14.3 V.

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
