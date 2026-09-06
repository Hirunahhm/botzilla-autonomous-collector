"""
frontier_detection.py.

Pure, ROS-independent frontier-detection logic for Milestone 4
(PHASE_1_IMPLEMENTATION_PLAN.md) — deliberately has no rclpy/message imports so it can be
unit-tested with plain Python lists (see test/test_frontier_detection.py) and called
directly from frontier_explorer_node.py with an OccupancyGrid's raw fields.

Grid convention (matches nav_msgs/msg/OccupancyGrid exactly): `data` is a flat, row-major
sequence indexed by `row * width + col`, where `row` corresponds to the grid's Y index and
`col` to its X index (the message spec's "cell (1,0) is listed second" == index 1 == col 1,
row 0; "cell (0,1) is at index == width" == row 1, col 0). Cell values: `< 0` = unknown,
`0..OCCUPIED_THRESHOLD-1` = free, `>= OCCUPIED_THRESHOLD` = occupied. OCCUPIED_THRESHOLD=65
matches the convention already used elsewhere in this project's map-analysis tooling.
"""

import math

OCCUPIED_THRESHOLD = 65

# 8-connectivity: two frontier cells touching only diagonally (e.g. tracing around a
# corner or a pillar) still count as adjacent, both for "does this free cell border
# unknown space" and for clustering. With 4-connectivity only, a diagonal-only pair
# gets split into two separate 1-cell clusters and both can be silently discarded as
# noise (min_cluster_size) even though combined they'd clear it.
_NEIGHBORS_8 = (
    (-1, -1), (-1, 0), (-1, 1),
    (0, -1),           (0, 1),
    (1, -1),  (1, 0),  (1, 1),
)


def _is_free(value):
    return 0 <= value < OCCUPIED_THRESHOLD


def _is_unknown(value):
    return value < 0


def find_frontiers(data, width, height, min_cluster_size=4):
    """Find frontier clusters: connected groups of free cells that border unknown space.

    Returns a list of (row, col, cell_count) tuples, one per cluster, in grid-cell
    coordinates (not world coordinates — see grid_to_world). Clusters smaller than
    min_cluster_size are discarded as noise (isolated frontier cells at map edges).

    row/col is the cluster's actual member cell nearest its centroid, NOT the raw mean
    position. The mean of a non-convex cluster (an L-shape, a cluster that wraps a
    corner or a pillar) can land outside the cluster entirely — sometimes on a
    non-free or unknown cell — which sends Nav2 an unreachable goal every time that
    cluster shape recurs, and exploration stalls on that spot until it blacklists.
    Snapping to the nearest real member cell guarantees the target is always an
    actual free, in-cluster cell.
    """
    def idx(r, c):
        return r * width + c

    frontier_mask = [False] * (width * height)
    for r in range(height):
        for c in range(width):
            i = idx(r, c)
            if not _is_free(data[i]):
                continue
            for dr, dc in _NEIGHBORS_8:
                nr, nc = r + dr, c + dc
                if 0 <= nr < height and 0 <= nc < width and _is_unknown(data[idx(nr, nc)]):
                    frontier_mask[i] = True
                    break

    visited = [False] * (width * height)
    clusters = []
    for r in range(height):
        for c in range(width):
            i = idx(r, c)
            if not frontier_mask[i] or visited[i]:
                continue
            # Flood-fill this cluster (8-connectivity).
            stack = [(r, c)]
            visited[i] = True
            cells = []
            while stack:
                cr, cc = stack.pop()
                cells.append((cr, cc))
                for dr, dc in _NEIGHBORS_8:
                    nr, nc = cr + dr, cc + dc
                    if 0 <= nr < height and 0 <= nc < width:
                        ni = idx(nr, nc)
                        if frontier_mask[ni] and not visited[ni]:
                            visited[ni] = True
                            stack.append((nr, nc))
            if len(cells) >= min_cluster_size:
                mean_r = sum(p[0] for p in cells) / len(cells)
                mean_c = sum(p[1] for p in cells) / len(cells)
                target_r, target_c = min(
                    cells, key=lambda p: (p[0] - mean_r) ** 2 + (p[1] - mean_c) ** 2
                )
                clusters.append((target_r, target_c, len(cells)))

    return clusters


def find_low_cost_point(costmap, width, height, row, col, max_cost=50, search_radius=10,
                        footprint_ok=None):
    """Find the nearest cell to (row, col) whose inflated costmap cost is safely low.

    A frontier cell is, by definition, adjacent to unknown space — and unknown space in
    a bounded arena is almost always adjacent to a wall just beyond sensor range. With
    inflation applied, that puts most raw frontier cells inside the wall's inflation
    halo: free in the raw SLAM map (cost 0) but LETHAL_OBSTACLE(100) or
    INSCRIBED_INFLATED_OBSTACLE(99) in the actual global_costmap the planner uses,
    which is why ComputePathToPose was observed live returning NO_VALID_PATH for the
    large majority of raw frontier targets in a compact arena. Rather than send Nav2 a
    goal it can only ever reject, expand outward (BFS, nearest-first) from the frontier
    cell and hand back the first cell whose cost is below max_cost — well under the 99
    inscribed cutoff, so the result is a point Nav2 can actually plan into, not just one
    that scrapes under the lethal threshold.

    costmap is a flat row-major int8 array matching nav_msgs/msg/OccupancyGrid.data
    (as published on /global_costmap/costmap): 0-100 = cost, -1 = unknown/unseen by the
    costmap. search_radius bounds the BFS in cells — a frontier with no low-cost cell
    anywhere nearby is treated as genuinely unreachable rather than searched forever.

    `footprint_ok`, when given, is called as footprint_ok(row, col) on each candidate
    that already passes the cost test, and the cell is only accepted if it also returns
    True. This is where the robot's real, asymmetric outline gets a say: max_cost is an
    inflation-derived scalar and inflation is built on the inscribed circle, so on its
    own it will happily hand back a cell whose clearance is less than the body's forward
    reach. Ordering matters — the cheap scalar test runs first and rejects most cells, so
    the polygon test only runs on the few that survive.

    Returns (row, col) of the nearest qualifying cell, or None if none exists within
    search_radius.
    """
    def idx(r, c):
        return r * width + c

    if not (0 <= row < height and 0 <= col < width):
        return None

    visited = {(row, col)}
    queue = [(row, col)]
    head = 0
    while head < len(queue):
        r, c = queue[head]
        head += 1
        cost = costmap[idx(r, c)]
        if 0 <= cost < max_cost and (footprint_ok is None or footprint_ok(r, c)):
            return (r, c)
        if abs(r - row) >= search_radius or abs(c - col) >= search_radius:
            continue
        for dr, dc in _NEIGHBORS_8:
            nr, nc = r + dr, c + dc
            if 0 <= nr < height and 0 <= nc < width and (nr, nc) not in visited:
                visited.add((nr, nc))
                queue.append((nr, nc))

    return None


# Robot outline in base_link metres, (x forward, y left), matching the `footprint`
# in nav2_params.yaml. It is deliberately NOT a circle: the body reaches 0.36 m ahead
# of base_link but only 0.22 m behind, and Nav2's inflation model cannot represent
# that — inflation is driven by the inscribed radius (the largest circle that fits,
# ~0.215 m here), which ignores the forward overhang entirely. Any scalar cost
# threshold therefore inherits a blind spot in front of the robot; see
# footprint_clear for the test that closes it.
DEFAULT_FOOTPRINT_M = (
    (0.36, 0.215),
    (0.36, -0.215),
    (-0.22, -0.215),
    (-0.22, 0.215),
)

# A footprint test must score against LETHAL cells only, never the inscribed band (99).
# A cell is 99 exactly when a lethal obstacle lies within the inscribed radius of it, so
# testing the whole polygon against >=99 demands footprint_extent + inscribed_radius of
# clearance — the same double-counting that in_collision's docstring records as having
# pinned the robot in open floor in an endless backup/spin loop. Nav2's own
# FootprintCollisionChecker scores the polygon against lethal for this reason.
LETHAL_COST = 100


def _point_in_polygon(x, y, polygon):
    """Ray-casting point-in-polygon test. polygon is a sequence of (x, y) vertices."""
    inside = False
    n = len(polygon)
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if (yi > y) != (yj > y):
            x_cross = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x < x_cross:
                inside = not inside
        j = i
    return inside


def footprint_clear(
    costmap, width, height, row, col, yaw, resolution,
    footprint_m=DEFAULT_FOOTPRINT_M, lethal_cost=LETHAL_COST,
):
    """Whether the robot's actual polygon, centred on this cell at heading `yaw`, is clear.

    This is the orientation-aware test that a scalar costmap threshold cannot express.
    The footprint is rotated by `yaw`, converted from metres to cell offsets, and every
    grid cell whose centre falls inside the resulting polygon is checked for a lethal
    obstacle. Because the outline is asymmetric, the answer genuinely depends on which
    way the robot is pointing — a goal 0.30 m from a wall is fine approached side-on and
    a collision approached nose-first.

    Cells outside the map are treated as clear: unknown space past the map edge is not
    evidence of an obstacle, matching in_collision and find_low_cost_point. Unknown
    cells (< 0) inside the map are likewise not treated as lethal.

    `costmap` is the published /global_costmap/costmap 0-100 scale (see
    find_low_cost_point). Returns True when nothing lethal lies under the footprint.
    """
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    # Rotate into world axes, then express in cell offsets from the centre cell. Cell
    # centres are one resolution apart, so a world offset of d metres is d/resolution
    # cells — no half-cell term, since both ends are centres (see grid_to_world).
    polygon = []
    for px, py in footprint_m:
        wx = px * cos_yaw - py * sin_yaw
        wy = px * sin_yaw + py * cos_yaw
        polygon.append((wx / resolution, wy / resolution))

    d_cols = [p[0] for p in polygon]
    d_rows = [p[1] for p in polygon]
    for d_row in range(int(math.floor(min(d_rows))), int(math.ceil(max(d_rows))) + 1):
        for d_col in range(int(math.floor(min(d_cols))), int(math.ceil(max(d_cols))) + 1):
            if not _point_in_polygon(d_col, d_row, polygon):
                continue
            r, c = row + d_row, col + d_col
            if not (0 <= r < height and 0 <= c < width):
                continue
            if costmap[r * width + c] >= lethal_cost:
                return False
    return True


def in_collision(costmap, width, height, row, col, inscribed_cost=99):
    """Whether a circular robot centred on this cell is in collision, per Nav2's own test.

    Nav2's inflation layer already encodes the footprint: a cell reaches
    INSCRIBED_INFLATED_OBSTACLE (253 internally, 99 as published on the OccupancyGrid)
    exactly when a lethal obstacle lies within the robot's inscribed radius of it. So for
    a circular footprint — which is what nav2_params.yaml configures via robot_radius —
    the collision test is a single lookup of the centre cell, and that is precisely the
    test the planner itself applies when deciding whether a pose is valid.

    This replaced an earlier check that scanned every cell within robot_radius for cost
    >= 99. That double-counted the inflation: requiring cells a full robot_radius away to
    also be clear of the inscribed band demands robot_radius + inscribed_radius (0.40 m
    with this project's tuning) of clearance where the robot only needs 0.20 m. Confirmed
    live, it reported the robot as footprint-blocked while it sat in open floor with its
    centre cell reading cost 0 and no lethal cell anywhere within its actual footprint,
    which sent it into an endless backup/spin recovery loop in perfectly drivable space.

    Cells outside the map are treated as not-in-collision (unknown space beyond the map
    edge is not evidence of an obstacle), matching how unknown cells are handled
    elsewhere in this module.
    """
    if not (0 <= row < height and 0 <= col < width):
        return False
    return costmap[row * width + col] >= inscribed_cost


def grid_to_world(row, col, resolution, origin_x, origin_y):
    """Grid-cell coordinates -> world coordinates (cell center), per OccupancyGrid.info."""
    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (row + 0.5) * resolution
    return x, y


def world_to_grid(x, y, resolution, origin_x, origin_y):
    """World coordinates -> grid-cell coordinates (inverse of grid_to_world)."""
    col = int((x - origin_x) / resolution)
    row = int((y - origin_y) / resolution)
    return row, col


def select_target(frontiers_world, robot_x, robot_y):
    """Pick the nearest frontier (by Euclidean distance) to the robot.

    frontiers_world: list of (x, y) world-coordinate tuples.
    Returns (x, y), or None if frontiers_world is empty (signals exploration complete).
    """
    if not frontiers_world:
        return None
    return min(
        frontiers_world,
        key=lambda p: (p[0] - robot_x) ** 2 + (p[1] - robot_y) ** 2,
    )


def distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)
