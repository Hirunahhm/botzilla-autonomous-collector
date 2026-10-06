"""
cube_registry.py — the leader's list of cubes it has seen, and which one to send next.

ROS-free, so it is unit tested directly (test/test_cube_registry.py); fleet_manager_node
feeds it projected detections and collector reports.

A cube estimate goes through
    pending (unconfirmed) -> pending (confirmed) -> assigned -> collected
    assigned -> pending again, after a failure
    assigned -> failed, after FAILURE_LIMIT failures; parked for RETRY_AFTER_S, then
                pending again

Why each rule exists:
- CONFIRM_SIGHTINGS: one YOLO frame is not evidence. The executor's spot limit exists
  because single false detections cost minutes of chasing on hardware (see
  executor_node SPOT_FAIL_LIMIT); sending a second robot across the room for one costs
  more.
- Exclusion zones: cubes delivered to HOME pile up there, and the leader's camera keeps
  seeing them. Without the HOME zone every delivered cube would come straight back as a
  new task — the same failure HOME_CUBE_SUPPRESS_RADIUS_M fixes in executor_node. The
  zone around the collector itself keeps the cube it is pushing or carrying from being
  registered as another one wherever it currently is.
- Collected entries never absorb new sightings: something seen later at a spot whose
  cube was delivered is a different cube (or noise), and must be able to become a task.
- Failed entries DO absorb sightings, so the same unreachable cube is not re-registered
  under a new id and retried straight away.
"""
import math

MERGE_RADIUS_M = 0.4
CONFIRM_SIGHTINGS = 2
FAILURE_LIMIT = 2
RETRY_AFTER_S = 180.0


class Cube:
    __slots__ = ('id', 'x', 'y', 'sightings', 'first_seen', 'last_seen', 'status',
                 'failures', 'retry_at', 'detail')

    def __init__(self, cube_id, x, y, t):
        self.id = cube_id
        self.x = x
        self.y = y
        self.sightings = 1
        self.first_seen = t
        self.last_seen = t
        self.status = 'pending'
        self.failures = 0
        self.retry_at = None
        self.detail = ''

    def as_dict(self):
        return {k: getattr(self, k) for k in self.__slots__}


def _inside(x, y, zones):
    return any(math.hypot(x - zx, y - zy) < r for zx, zy, r in zones)


class CubeRegistry:
    def __init__(self, merge_radius_m=MERGE_RADIUS_M, confirm_sightings=CONFIRM_SIGHTINGS,
                 failure_limit=FAILURE_LIMIT, retry_after_s=RETRY_AFTER_S):
        self.merge_radius_m = merge_radius_m
        self.confirm_sightings = confirm_sightings
        self.failure_limit = failure_limit
        self.retry_after_s = retry_after_s
        self.cubes = {}
        self._next_id = 1

    # -- observations ------------------------------------------------------------

    def observe(self, x, y, t, exclusions=()):
        """Merge one map-frame detection. Returns the Cube it went to, or None if excluded.

        exclusions: iterable of (x, y, radius) zones where detections are ignored.
        """
        if _inside(x, y, exclusions):
            return None
        best, best_d = None, self.merge_radius_m
        for cube in self.cubes.values():
            if cube.status == 'collected':
                continue
            d = math.hypot(cube.x - x, cube.y - y)
            if d < best_d:
                best, best_d = cube, d
        if best is None:
            best = Cube(self._next_id, x, y, t)
            self.cubes[best.id] = best
            self._next_id += 1
            return best
        # Running mean, but an assigned cube's target is left alone: the collector is
        # already driving to it, and a moving target only confuses the leader's log.
        if best.status != 'assigned':
            n = best.sightings
            best.x = (best.x * n + x) / (n + 1)
            best.y = (best.y * n + y) / (n + 1)
        best.sightings += 1
        best.last_seen = t
        return best

    def drop_inside(self, zones):
        """Forget unassigned, uncollected cubes inside zones (e.g. a HOME learnt late)."""
        gone = [c.id for c in self.cubes.values()
                if c.status in ('pending', 'failed') and _inside(c.x, c.y, zones)]
        for cid in gone:
            del self.cubes[cid]
        return gone

    # -- allocation --------------------------------------------------------------

    def _refresh(self, t):
        for cube in self.cubes.values():
            if cube.status == 'failed' and cube.retry_at is not None and t >= cube.retry_at:
                cube.status, cube.failures, cube.retry_at = 'pending', 0, None

    def confirmed(self, cube):
        return cube.sightings >= self.confirm_sightings

    def next_task(self, robot_xy, t):
        """Return the next cube for a collector at robot_xy, or None.

        Fewest failures first, then nearest: a cube that has just failed is retried only
        once nothing untried is left, so one bad spot cannot hold up the others.
        """
        self._refresh(t)
        ready = [c for c in self.cubes.values()
                 if c.status == 'pending' and self.confirmed(c)]
        if not ready:
            return None
        rx, ry = robot_xy
        return min(ready, key=lambda c: (c.failures, math.hypot(c.x - rx, c.y - ry), c.id))

    def assign(self, cube_id):
        self.cubes[cube_id].status = 'assigned'

    def report(self, cube_id, collected, t, detail=''):
        """Record a collector's result for an assigned cube."""
        cube = self.cubes.get(cube_id)
        if cube is None:
            return None
        cube.detail = detail
        if collected:
            cube.status = 'collected'
            return cube
        cube.failures += 1
        if cube.failures >= self.failure_limit:
            cube.status = 'failed'
            cube.retry_at = t + self.retry_after_s
        else:
            cube.status = 'pending'
        return cube

    def relocate(self, cube_id, x, y):
        """Move a cube's estimate to where it is now known to be (released short of HOME)."""
        cube = self.cubes.get(cube_id)
        if cube is not None and cube.status != 'collected':
            cube.x, cube.y = x, y
        return cube

    def unassign(self, cube_id):
        """Put an assigned cube back without counting a failure (collector went silent)."""
        cube = self.cubes.get(cube_id)
        if cube is not None and cube.status == 'assigned':
            cube.status = 'pending'

    def counts(self):
        out = {'pending': 0, 'unconfirmed': 0, 'assigned': 0, 'collected': 0, 'failed': 0}
        for c in self.cubes.values():
            if c.status == 'pending' and not self.confirmed(c):
                out['unconfirmed'] += 1
            else:
                out[c.status] += 1
        return out
