"""
right_of_way.py — the fleet manager's coordination rules between the two robots.

ROS-free (plain floats, seconds), unit-tested in test/test_right_of_way.py. The fleet
manager is the only node that sees both robots, so these live with it:

  path_ahead        the next stretch of the leader's route, drawn as soft cost in the
                    collector's grid so it plans out of the leader's way BEFORE they meet
                    (prioritized planning: the leader goes first, the collector plans
                    around it). Never lethal: a lethal stripe across the collector's own
                    spot would freeze it, like a cube mark under a robot did in run 9.
  EscapeLatch       stop drawing the leader in the collector's grid while the two
                    overlap, so the collector can move off. Once a robot's footprint is
                    inside the other's lethal mark every Nav2 move is refused, even moving
                    away (run 16: 13 back-ups refused, "Collision Ahead", for 6 minutes).
                    Only the COLLECTOR is released, and only while it is YIELDING: the
                    leader keeps the collector's mark, so it cannot creep further in.
  GiveWay           pause the leader's exploration while a collector carrying a cube is
                    close. A carrying collector must not reverse (it would drop the cube)
                    and never yields, and the leader had no rule for that meeting either;
                    the leader can always wait.
"""
import math

# ---- path_ahead -----------------------------------------------------------------------
PATH_AHEAD_M = 1.5
PATH_STEP_M = 0.15


def path_ahead(path_xy, robot_xy, length_m=PATH_AHEAD_M, step_m=PATH_STEP_M):
    """Return points every step_m along path_xy, from its point nearest robot_xy.

    Covers length_m; [] for an empty path or a robot more than 1 m off it (a stale plan).
    """
    if not path_xy:
        return []
    rx, ry = robot_xy
    k0 = min(range(len(path_xy)),
             key=lambda k: math.hypot(path_xy[k][0] - rx, path_xy[k][1] - ry))
    if math.hypot(path_xy[k0][0] - rx, path_xy[k0][1] - ry) > 1.0:
        return []
    out = [path_xy[k0]]
    travelled, since = 0.0, 0.0
    for a, b in zip(path_xy[k0:], path_xy[k0 + 1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        travelled += seg
        since += seg
        if since >= step_m:
            out.append(b)
            since = 0.0
        if travelled >= length_m:
            break
    return out


# ---- EscapeLatch ----------------------------------------------------------------------
# Separation beyond the overlap distance needed before the leader is drawn again.
ESCAPE_CLEAR_MARGIN_M = 0.10
YIELDING_STATE = 'YIELDING'     # collector_node Collector.YIELDING


def footprint_distance(point_xy, pose, rect):
    """Distance from a point to a rectangle ((x0, x1), (y0, y1)) at pose; 0 inside."""
    x, y, yaw = pose
    (x0, x1), (y0, y1) = rect
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = point_xy[0] - x, point_xy[1] - y
    u, v = c * dx + s * dy, -s * dx + c * dy
    du = max(x0 - u, u - x1, 0.0)
    dv = max(y0 - v, v - y1, 0.0)
    return math.hypot(du, dv)


class EscapeLatch:
    """Decide whether to draw the leader in the collector's grid.

    Only while the collector is YIELDING (moving off for the leader). In run 21 the latch
    stayed on after the collector had switched back to GOING; with fleet_scan_filter also
    removing the leader from its scans, the collector then had no sign of the leader at
    all and drove back to 0.32 m from it. Any other state draws the leader, always.
    """

    def __init__(self):
        self.escaping = False

    def update(self, leader_xy, collector_pose, collector_footprint, leader_radius,
               collector_state=YIELDING_STATE):
        """Return True to draw the leader; False while a yielding collector is inside it."""
        if (leader_xy is None or collector_pose is None
                or collector_state != YIELDING_STATE):
            self.escaping = False
            return True
        d = footprint_distance(leader_xy, collector_pose, collector_footprint)
        if d < leader_radius:
            self.escaping = True
        elif d >= leader_radius + ESCAPE_CLEAR_MARGIN_M:
            self.escaping = False
        return not self.escaping


# ---- GiveWay --------------------------------------------------------------------------
CARRYING_STATES = ('CAPTURING', 'DELIVERING', 'DETACHING')
GIVE_WAY_M = 1.2           # pause the leader when a carrying collector is this close
GIVE_WAY_CLEAR_M = 1.6     # resume once it is this far
GIVE_WAY_MAX_S = 30.0      # never hold the leader longer than this...
GIVE_WAY_COOLDOWN_S = 15.0  # ...nor pause it again this soon after


class GiveWay:
    """Pause/resume decisions for the leader's exploration (see module docstring)."""

    def __init__(self):
        self.paused_since = None
        self._cooldown_until = 0.0

    @property
    def paused(self):
        return self.paused_since is not None

    def update(self, t, collector_state, distance):
        """Return 'pause', 'resume' or None. distance None = collector pose unknown."""
        carrying = collector_state in CARRYING_STATES
        if self.paused:
            if (not carrying or distance is None or distance > GIVE_WAY_CLEAR_M
                    or t - self.paused_since > GIVE_WAY_MAX_S):
                self.paused_since = None
                self._cooldown_until = t + GIVE_WAY_COOLDOWN_S
                return 'resume'
            return None
        if (carrying and distance is not None and distance < GIVE_WAY_M
                and t >= self._cooldown_until):
            self.paused_since = t
            return 'pause'
        return None
