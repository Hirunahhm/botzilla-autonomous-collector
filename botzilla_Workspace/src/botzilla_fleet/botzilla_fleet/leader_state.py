"""
leader_state.py — what the leader is doing, for the collector's right-of-way rule.

The fleet manager runs on the leader, classifies it from its own map pose and its Nav2
NavigateToPose feedback, and publishes the result on /<ns>/fleet/leader_state:

  MOVING  it translated or turned recently (spinning to inspect a cube counts: in run 16
          the collector drove in next to a leader that was only turning, because by
          position alone it looked parked);
  STUCK   Nav2 is in recovery, or holds a goal the leader is not moving towards, and it
          has not got anywhere lately;
  PARKED  neither: stopped of its own accord (inspecting, between goals).

The collector used to guess this from the leader's position alone and could not tell
STUCK from PARKED. It treated both as "parked, plan past it", which drove it straight
back into a leader that was stuck next to it (run 16, multi_robot_runs.md) — the closer
it got, the more stuck the leader was. Now only PARKED is planned past; a STUCK leader is
given room.

ROS-independent (plain floats, seconds), unit-tested in test/test_leader_state.py.
"""
from collections import deque
import math

MOVING = 'MOVING'
PARKED = 'PARKED'
STUCK = 'STUCK'

# Moved this far, or turned this much, within MOVING_WINDOW_S = moving.
MOVING_M = 0.08
MOVING_RAD = 0.35
MOVING_WINDOW_S = 1.5
# A recovery (number_of_recoveries going up) within this long marks the leader STUCK,
# unless it has since got STUCK_ESCAPE_M away from where the recovery started.
# number_of_recoveries also counts the instant costmap-clearing subtree (see
# frontier_explorer_node RECOVERY_BUDGET_S), so a single clear followed by real progress
# must not keep it STUCK: the escape distance ends it.
STUCK_HOLD_S = 8.0
STUCK_ESCAPE_M = 0.25
# Holding a Nav2 goal (feedback within FEEDBACK_FRESH_S) while not translating
# STILL_GOAL_M for this long is STUCK too: DWB with no valid trajectory just sits there
# before the BT's first recovery, and a turn-in-place to line up takes well under this.
STILL_WITH_GOAL_S = 8.0
STILL_GOAL_M = 0.05
FEEDBACK_FRESH_S = 1.0
# Once STUCK, it stays STUCK until the leader has got STUCK_ESCAPE_M from where it got
# stuck, or STUCK_RELEASE_S have passed with nothing stuck about it. Without this the
# state flipped to PARKED in the gap between the explorer cancelling a stuck goal and
# sending the next one (no goal, no recoveries, no motion), and the collector "planned
# past" a leader that had been stuck seconds earlier (run 17, 201 s and 241 s).
STUCK_RELEASE_S = 15.0


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class LeaderStateTracker:
    """Feed it poses and Nav2 feedback; ask it for the state."""

    def __init__(self):
        self._poses = deque()           # (t, x, y, yaw)
        self._recoveries = None         # last number_of_recoveries seen
        self._recovery_t = None         # time of the last increment
        self._recovery_xy = None        # where the leader was then
        self._feedback_t = None
        self._stuck_xy = None           # where the current STUCK spell began
        self._stuck_t = None            # last time it was stuck by the raw test
        self._history_s = max(MOVING_WINDOW_S, STILL_WITH_GOAL_S) + 1.0

    def pose(self, t, x, y, yaw):
        self._poses.append((t, x, y, yaw))
        while self._poses and t - self._poses[0][0] > self._history_s:
            self._poses.popleft()

    def feedback(self, t, number_of_recoveries):
        """One NavigateToPose feedback message; a new goal restarts the count at 0."""
        self._feedback_t = t
        if self._recoveries is not None and number_of_recoveries > self._recoveries:
            self._recovery_t = t
            self._recovery_xy = self._poses[-1][1:3] if self._poses else None
        self._recoveries = number_of_recoveries

    def _window(self, t, span):
        return [p for p in self._poses if t - p[0] <= span]

    def _translated(self, t, span):
        w = self._window(t, span)
        if len(w) < 2:
            return 0.0
        x0, y0 = w[0][1:3]
        return max(math.hypot(p[1] - x0, p[2] - y0) for p in w)

    def _turned(self, t, span):
        w = self._window(t, span)
        total = 0.0
        for a, b in zip(w, w[1:]):
            total += abs(_wrap(b[3] - a[3]))
        return total

    def moving(self, t):
        return (self._translated(t, MOVING_WINDOW_S) > MOVING_M
                or self._turned(t, MOVING_WINDOW_S) > MOVING_RAD)

    def stuck(self, t):
        if self._recovery_t is not None and t - self._recovery_t <= STUCK_HOLD_S:
            here = self._poses[-1][1:3] if self._poses else None
            if (here is None or self._recovery_xy is None
                    or math.hypot(here[0] - self._recovery_xy[0],
                                  here[1] - self._recovery_xy[1]) < STUCK_ESCAPE_M):
                return True
        goal_active = self._feedback_t is not None and t - self._feedback_t <= FEEDBACK_FRESH_S
        if goal_active and self._poses and t - self._poses[0][0] >= STILL_WITH_GOAL_S:
            return self._translated(t, STILL_WITH_GOAL_S) < STILL_GOAL_M
        return False

    def _sticky_stuck(self, t):
        """Return True while stuck by the raw test, or still within its release window."""
        here = self._poses[-1][1:3] if self._poses else None
        if self.stuck(t):
            if self._stuck_xy is None:
                self._stuck_xy = here
            self._stuck_t = t
            return True
        if self._stuck_t is None:
            return False
        escaped = (here is not None and self._stuck_xy is not None
                   and math.hypot(here[0] - self._stuck_xy[0],
                                  here[1] - self._stuck_xy[1]) >= STUCK_ESCAPE_M)
        if escaped or t - self._stuck_t > STUCK_RELEASE_S:
            self._stuck_xy = self._stuck_t = None
            return False
        return True

    def state(self, t):
        # STUCK first: a recovery's BackUp/Spin moves the robot, and it is still stuck.
        if self._sticky_stuck(t):
            return STUCK
        if self.moving(t):
            return MOVING
        return PARKED
