# Autonomous Multi-Cube Search & Collection System
**Embedded Software Systems Project — Semester 5, ICE Stream, University of Moratuwa**

This document is the working specification for an AI coding agent (Claude Code) to reference throughout implementation. It captures the project's purpose, architecture, phased scope, and the reasoning behind key decisions so that generated code stays consistent with the design intent even across many sessions.

---

## 1. Project Summary

An autonomous mobile robot system that explores an **unknown, unmapped indoor arena** (no predefined dimensions), detects and physically **collects an arbitrary number (N) of cubes**, deposits each at a defined drop-off point, and knows when the task is complete — all without human intervention beyond issuing the start command.

Phase 2 extends this to a **two-robot leader/executor system**: a Jetson-based leader robot builds the shared map and detects cubes and obstacles; a Raspberry Pi-based executor robot receives high-level goals from the leader and physically collects the cubes.

**Not a rubber-plantation or outdoor project.** This is an indoor Kobuki-based system. (A prior, separate concept involving an outdoor rubber-estate weeding robot was abandoned after lecturer feedback that a from-scratch custom robot was out of scope for this module — see Section 9.)

---

## 2. Motivation & Framing (for report/viva use)

**The core capability being demonstrated:** autonomous search, retrieval, and delivery of objects in an environment that cannot be pre-mapped or instrumented with fixed infrastructure, using coordinated multi-robot execution.

**Why not fixed cameras + dumb robots (the "obvious" alternative):** fixed camera infrastructure is superior *when the space can be instrumented and mapped in advance and no physical manipulation is required*. This project is deliberately scoped for the complementary case — spaces that cannot be pre-instrumented (disaster response, hazardous zones, unstructured/changing environments) and tasks that require *physical retrieval*, not just detection. This framing should be used consistently in any generated documentation, code comments, or report text — it is the answer to "why build this instead of wall cameras."

**Real-world analogues (use these, not "supermarket" or "warehouse-with-fixed-shelves" framings):**
- Disaster/search-and-rescue retrieval in unmapped, changing environments
- Hazardous-zone (contamination, unexploded ordnance) object retrieval
- Warehouse/logistics retrieval in unstructured or dynamically-changing storage
- Agricultural debris/object collection in unstructured fields

**Honest positioning:** this project demonstrates the *navigation, exploration, multi-object task management, and coordination architecture*. It does not claim production-ready perception or manipulation for messy real-world objects — the cube is a stand-in target chosen for tractable detection and grasping.

**Novelty positioning (important — do not overclaim):** the individual techniques (SLAM, Nav2, YOLO detection, frontier exploration, leader/executor coordination) are all well-established and not novel research contributions. The project's legitimate claim is:
1. It generalizes a prior fixed-arena, hardcoded-quadrant system (BotZilla — see Section 9) to unknown, unmapped environments with an unknown number of targets.
2. Within its own cohort (9 peer Kobuki projects reviewed — see Section 10), it is the only one that physically collects/manipulates objects rather than stopping at detection, tracking, or navigation.
3. An optional measured comparison (Section 8) can add a small genuine empirical contribution on top of the engineering system.

Do not present this project as "novel" or "unique" in absolute terms. Present it as "a generalization of an existing system, addressing a gap not covered by cohort peers, with sound engineering execution."

---

## 3. Hardware

| Component | Role | Notes |
|---|---|---|
| Kobuki QBot (x2 in Phase 2) | Mobile base | Differential drive |
| Kinect v1 (Xbox 360 RGB-D) | RGB + depth camera | Min usable range ~0.55m (tuned) |
| 2D LiDAR | 360° range sensing | New addition — not present in BotZilla |
| Jetson Orin Nano Super | Compute — **Leader robot** | JetPack 7.2, ROS 2 Jazzy |
| Raspberry Pi (4 or 5) | Compute — **Executor robot** (Phase 2 only) | Same ROS 2 Jazzy stack |
| Simple gripper arm | Cube manipulation | Reused/adapted from BotZilla |

### Key Hardware Decisions & Why

- **No Isaac ROS.** JetPack 7.2 / ROS 2 Jazzy support for Isaac ROS is immature specifically on Orin Nano-class hardware (validated primarily on newer Thor-class boards). Additionally, with a 2D LiDAR present, `slam_toolbox` + Nav2 already solve SLAM/navigation reliably — Isaac ROS's SLAM/Nvblox tools are largely redundant here. GPU acceleration is instead obtained directly via **TensorRT** (YOLO export) — same underlying acceleration, without the framework/version risk.
- **Stay on JetPack 7.2 / ROS 2 Jazzy — do not downgrade to 6.2/Humble.** BotZilla's working Kobuki + Kinect bring-up is already on Jazzy. Downgrading would require porting working code for a benefit (Isaac ROS reliability) that is not needed.
- **2D LiDAR limitations to keep in mind:** it senses only a single horizontal plane. Low obstacles (e.g. cubes themselves) and overhanging obstacles are invisible to it — this is actually convenient for cube detection (LiDAR won't clutter the obstacle costmap with cubes) but means low/overhanging obstacle safety relies on the Kinect if the arena has any. Default assumption: arena has normal-height walls/obstacles, LiDAR-only costmap is sufficient for Phase 1.

---

## 4. Software Stack

```
JetPack 7.2 + ROS 2 Jazzy
├── Kobuki driver — custom pyserial driver (from BotZilla, NOT kobuki_core/kobuki_ros)
├── Kinect bridge — freenect-based (from BotZilla)
├── 2D LiDAR driver → slam_toolbox (SLAM) + Nav2 (navigation, costmap, path planning)
├── YOLOv8 (cube detection) → exported to TensorRT for GPU inference on Jetson
└── Isaac ROS — NOT USED
```

**No retraining required for obstacle detection.** Obstacles are handled geometrically via LiDAR → costmap (Nav2). This requires no ML model of any kind.

**No retraining required for cube detection (default assumption).** Reuse BotZilla's existing trained YOLOv8 model (`runs/best-fit/best.pt` in the BotZilla repo). Verify detection rate against actual cubes/lighting in Week 1 testing; only fine-tune if verification shows the existing model underperforms.

---

## 5. Reused Codebase — BotZilla

**Source repos (already cloned/reviewed):**
- Main: `https://github.com/IntellisenseLab/final-project-botzilla`
- Raspberry Pi optimized branch: `test/rasberrypi`

BotZilla was a prior project (same student, different team, CS3340 module) — an autonomous cube detection/collection robot on Kobuki + Kinect + Raspberry Pi 4/5, using YOLOv8 Nano, PCL, SMACH state machine, ROS 2 Jazzy.

### Directly Reusable (port with minimal changes)
- `KobukiDriver.py` + `kobuki_base_node.py` — hand-rolled pyserial driver (protocol-level, not a ROS2 package dependency), handles `/cmd_vel` → motors and encoders → `/odom`. Fully portable to Jetson — no ROS2-distro dependency issues.
- `kinect_bridge.py` — freenect-based RGB+depth publisher to standard `sensor_msgs/Image` topics.
- Depth decode math in `yolo_node.py` — raw 11-bit Kinect value → meters conversion; `KINECT_MIN_RANGE_M = 0.55` constant (already tuned/verified).
- Package skeleton pattern: `botzilla_bringup` / `botzilla_control` / `botzilla_perception` split.

### Reusable But Must Be Upgraded
- YOLO inference — BotZilla runs plain Ultralytics on CPU/Pi. **Export the same model to TensorRT** for GPU-accelerated inference on the Jetson.

### Explicitly Drop
- `LD_PRELOAD=noreset.so` workaround in `kinect_bridge.py` — this fixes a Raspberry Pi 5-specific RP1 USB controller bug. The Jetson has different USB hardware; do not carry this over. Test kinect_bridge without it first; only investigate a Jetson-specific fix if the same reset symptom appears.
- Cube-quadrant hardcoded search logic — replaced by frontier-based exploration (Section 6).
- AprilTag docking/localization logic, gripper-arm-specific SMACH sequence for the fixed single-cube task — replaced by the new N-cube state machine (Section 6).

---

## 6. Architecture Principle: Leader/Executor Split (Design From Day One)

**This must be built into Phase 1 even though Phase 1 is single-robot.** The "brain" (planning/decision-making) must be architecturally separated from the "executor" (goal execution) from the very first implementation, so that Phase 2 (adding a second physical robot) is a bolt-on rather than a rewrite.

```
PLANNER (the "leader" role)
├── Frontier exploration decision-making
├── Cube detection → map-frame localization
├── Task state machine (explore / navigate-to-cube / collect / navigate-to-drop / repeat)
└── Issues high-level goals: "go to (x,y)" / "collect cube at (x,y)" / "go to drop point"
        │
        ▼  (goals, not raw velocity commands)
EXECUTOR (the "member" role)
├── Local Nav2 (executes a given goal, avoids obstacles via local costmap)
├── Own odometry
└── Reports back: goal reached / cube collected / obstacle blocking / status
```

In Phase 1, the Jetson robot plays both roles (it commands itself). In Phase 2, the Jetson remains the planner; the Pi robot becomes a second executor receiving goals over the network. **Never design the executor to receive raw `cmd_vel` streams from the leader over the network** — network latency/dropout makes this unsafe. Always send discrete goals; the executor plans and drives locally.

---

## 7. Phase 1 — Single Robot (Target: 5 weeks; do not compress below 4)

**Scope:** explore unknown arena (no predefined dimensions), avoid static obstacles, detect and collect N cubes (N unknown in advance), deposit each at a defined drop point, terminate when no more cubes are found.

### Week 1 — Hardware Bring-Up
- Confirm JetPack 7.2 + ROS 2 Jazzy on Jetson.
- Clone BotZilla; strip cube-quadrant, AprilTag, and fixed-sequence SMACH logic; retain `botzilla_bringup`/`botzilla_control`/`botzilla_perception` skeleton.
- Bring up Kobuki (drive via `/cmd_vel`, confirm `/odom`), Kinect (RGB+depth topics, test without `noreset.so` first), and the new 2D LiDAR (`/scan`) individually.
- **Deliverable:** all three sensors publish correctly; robot drivable via teleop.

### Week 2 — SLAM
- Integrate `slam_toolbox` with LiDAR.
- Get the TF tree correct: `map → odom → base_link → sensor frames`. This is the most common silent-failure point in the whole project — get it right now.
- **Deliverable:** live map builds in RViz while driving; robot localizes on its own map.

### Week 3 — Navigation & Static Obstacle Avoidance
- Bring up Nav2 on top of SLAM. Goal-to-goal navigation via RViz, static obstacle avoidance via LiDAR-fed costmap.
- Tune costmap inflation and Kobuki kinematics for smooth motion.
- **Deliverable:** click a goal, robot navigates there autonomously, avoiding static obstacles. (This is the safety-net demo layer — if everything after this slips, this alone is still a legitimate partial demo.)

### Week 4 — Cube Perception & Localization
- Port BotZilla's YOLOv8 cube model; export to TensorRT.
- Verify detection against real cubes/lighting; fine-tune only if needed (see Section 4).
- Convert detected cube (image position + depth) into a **map-frame (x,y) coordinate** via TF — this turns a detection into a navigable goal.
- **Deliverable:** robot detects a cube, publishes its map-frame position, visible in RViz.

### Week 5 — Collection Brain: Frontier Exploration + N-Cube Loop
Priority order (must-have → stretch):
1. **Must-have:** frontier-based exploration — robot autonomously explores the unknown arena without a predefined map or hardcoded dimensions (this is the key generalization over BotZilla).
2. **Must-have:** full state machine — EXPLORE → cube detected → NAVIGATE-TO-CUBE → COLLECT (reuse/adapt BotZilla gripper logic) → NAVIGATE-TO-DROP → resume EXPLORE.
3. **Must-have:** N-cube generalization — track collected cubes (avoid re-grabbing), loop until no new cubes found for some exploration threshold, then terminate cleanly.
4. **Stretch (only if time allows):** enable Nav2 dynamic obstacle avoidance via continuously-updating local costmap (see Section 8 for what "dynamic" realistically means here).
- **Deliverable:** robot autonomously explores an unknown arena, finds and collects an unknown number of cubes, drops each at the defined point, and stops when done.

**Note on the leader/executor split:** even though Phase 1 is single-robot, the state machine above must be written as "planner issues goals → executor (Nav2) executes them" per Section 6, not as a monolithic script.

---

## 8. Dynamic Obstacle Avoidance — Scope Definition (Important)

"Dynamic obstacle avoidance" has an easy version and a hard version. **Only the easy version is in scope for this project.**

**In scope (achievable, mostly configuration not new code):** Nav2's local costmap already updates continuously from LiDAR data. When a new obstacle appears (a person walks through, an object moves), it becomes occupied in the local costmap and the local planner reroutes or stops. This requires enabling/tuning the existing costmap obstacle layer and inflation parameters — not building new prediction logic.

**Out of scope (do not attempt):** predictive motion modeling of moving obstacles, "social navigation," or any approach requiring anticipation of where an obstacle will move next. This is a real open research problem and is explicitly not a goal here.

**Why it matters for Phase 2:** once a second robot exists, each robot is a dynamic obstacle to the other. The in-scope reactive version (costmap-based stop/reroute) is sufficient to prevent collisions between two slow-moving robots — the hard predictive version is not required even then.

---

## 9. Phase 2 — Two-Robot Leader/Executor System (Stretch Goal, Weeks 6–7)

**Gate condition: only attempt Phase 2 if Phase 1 is stable and reliable by end of Week 5.** If Phase 1 is still flaky, spend Weeks 6–7 hardening Phase 1 instead. Do not let Phase 2 risk compromise a working Phase 1 demo.

### Roles
- **Jetson robot = Leader.** Runs SLAM, builds the shared map, detects cubes and obstacles, runs the frontier exploration and task-allocation logic, maintains the list of collected/pending cubes.
- **Raspberry Pi robot = Executor.** Receives high-level goals from the leader ("navigate to (x,y) and collect the cube there"). Runs its own local Nav2 instance for goal execution and obstacle avoidance — does **not** receive raw velocity commands from the leader.

### Key Design Decisions
- **Shared map, not raw obstacle list.** The leader shares its Nav2 costmap/occupancy map directly (not a hand-maintained list of coordinates) — this is the artifact Nav2 already produces and the executor already knows how to consume.
- **Coordinate frame alignment: use known start positions.** Do not attempt full multi-robot SLAM map-merging (a genuinely hard, FYP-or-beyond problem) — start both robots from pre-defined, known poses so their frames align by construction.
- **Executor still needs a minimal local safety reflex**, even though it doesn't need to *map* obstacles itself. It must react to unmapped/dynamic obstacles it encounters directly — most importantly, **the other robot**, which is a moving obstacle invisible to any pre-built map. A simple "stop if something is within N cm dead ahead" check (using Kobuki bump/cliff sensors and/or Kinect depth) is sufficient; this is not the same as full obstacle detection/mapping.
- **Task allocation:** leader assigns cube-collection goals to the executor as cubes are found; simplest approach is direct assignment (leader finds cube → sends goal to executor) rather than a complex bidding/auction system.

### Phase 2 Sub-Steps (in order)
1. Get the Pi robot running its own working Kobuki+Kinect+Nav2 bring-up independently (its own Phase-1-equivalent, simplified — it does not need its own SLAM/exploration, just goal execution).
2. Establish Jetson ↔ Pi ROS 2 communication (multi-machine DDS) and confirm the shared map/goal messages work.
3. Add task allocation logic on the leader.
4. Add the executor's minimal local safety reflex.
5. Only then: attempt dynamic obstacle avoidance refinement per Section 8.

---

## 10. Cohort Context (for differentiation — reference only, not to be treated as competitive benchmarking in code)

Nine peer Kobuki-based ROS 2 projects from the same cohort were reviewed (IntellisenseLab GitHub org). Summary of overlap and gaps, for report/viva framing:

**Common across most peer projects (i.e. NOT a valid differentiator on its own):** LiDAR SLAM (`slam_toolbox`), Nav2/A* navigation, Kobuki driver bring-up, Kinect RGB-D integration. At least 6 of 8 reviewed projects have this combination.

**What no peer project does (this project's actual differentiation):**
- Physical manipulation/collection of objects — every peer project stops at detection, tracking, or navigation-to-object. None has a gripper or completes a physical pick-and-deposit loop.
- True frontier-based autonomous exploration of an unmapped space — peer projects generally pre-map (often via teleop) before autonomous navigation to known goals.
- Multi-object, unknown-quantity completion tracking ("find and collect all N, know when done") — peer projects operate on single, pre-specified goals/targets.

**Closest overlapping peer project:** `final-project-nextrones` — YOLOv8 + Kinect depth + TF projection + SLAM + Nav2, navigates to a selected detected object. Does not manipulate/collect, operates on single-goal selection rather than multi-object completion, runs on Raspberry Pi only (no TensorRT/edge-GPU optimization).

---

## 11. Optional: The "Measured Question" (Small Genuine Contribution)

If time allows (do not prioritize over Phase 1/2 core functionality), add one narrow, measured comparison to give the project a small evidence-based contribution beyond "we built a working system." Two candidates, pick one:

1. **Exploration strategy comparison:** frontier-only exploration vs. a simple perception-biased variant (bias toward areas near previously-found cubes) — measure time/distance to collect all N cubes across repeated runs.
2. **Edge performance characterization:** measure actual FPS/latency of YOLO+TensorRT running concurrently with SLAM+Nav2 on the Jetson Orin Nano under load — a real number that peer project `nextrones` (Pi-only, no GPU optimization) did not produce.

Either is a legitimate, small, honest empirical result — not a claim of novel algorithmic contribution.

---

## 12. Summary of Explicit Non-Goals

To prevent scope creep during implementation, the following are explicitly **out of scope** for this project:

- Isaac ROS (any package)
- Full multi-robot SLAM / map-merging
- Predictive/social dynamic obstacle avoidance
- Open-vocabulary or language-conditioned object search
- Retraining YOLO from scratch (only fine-tune if the reused BotZilla model demonstrably underperforms)
- Cloud-based inference or any cloud dependency (all inference is on-device/edge)
- Retail/supermarket, wall-mounted camera infrastructure, or any non-mobile-robot sensing approach
- Outdoor/rubber-plantation deployment (a separate, earlier concept — not part of this project)
