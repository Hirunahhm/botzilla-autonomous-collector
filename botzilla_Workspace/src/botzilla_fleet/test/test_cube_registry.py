from botzilla_fleet.cube_registry import CubeRegistry


def test_confirmation_needs_two_sightings():
    r = CubeRegistry()
    r.observe(1.0, 1.0, 0.0)
    assert r.next_task((0, 0), 1.0) is None
    r.observe(1.1, 1.0, 1.0)
    cube = r.next_task((0, 0), 2.0)
    assert cube is not None and len(r.cubes) == 1
    assert abs(cube.x - 1.05) < 1e-9


def test_separate_cubes_and_nearest_first():
    r = CubeRegistry(confirm_sightings=1)
    r.observe(3.0, 0.0, 0.0)
    r.observe(1.0, 0.0, 0.0)
    assert len(r.cubes) == 2
    assert r.next_task((0, 0), 0.0).x == 1.0
    assert r.next_task((4, 0), 0.0).x == 3.0


def test_exclusion_zone_ignores_detections():
    r = CubeRegistry(confirm_sightings=1)
    assert r.observe(0.3, 0.0, 0.0, exclusions=[(0.0, 0.0, 1.0)]) is None
    assert not r.cubes


def test_drop_inside_removes_pending_only():
    r = CubeRegistry(confirm_sightings=1)
    a = r.observe(0.2, 0.0, 0.0)
    b = r.observe(0.0, 0.5, 0.0)
    r.assign(b.id)
    assert r.drop_inside([(0.0, 0.0, 1.0)]) == [a.id]
    assert list(r.cubes) == [b.id]


def test_collected_cube_does_not_absorb_new_sightings():
    r = CubeRegistry(confirm_sightings=1)
    c = r.observe(1.0, 1.0, 0.0)
    r.assign(c.id)
    r.report(c.id, True, 5.0)
    d = r.observe(1.0, 1.0, 6.0)
    assert d.id != c.id and d.status == 'pending'


def test_failures_park_then_retry():
    r = CubeRegistry(confirm_sightings=1, failure_limit=2, retry_after_s=100.0)
    c = r.observe(1.0, 1.0, 0.0)
    r.assign(c.id)
    r.report(c.id, False, 1.0)
    assert c.status == 'pending'
    r.assign(c.id)
    r.report(c.id, False, 2.0)
    assert c.status == 'failed'
    # Failed cubes still absorb sightings, so they are not re-registered.
    assert r.observe(1.1, 1.0, 3.0).id == c.id
    assert r.next_task((0, 0), 50.0) is None
    assert r.next_task((0, 0), 103.0).id == c.id


def test_assigned_target_is_frozen():
    r = CubeRegistry(confirm_sightings=1)
    c = r.observe(1.0, 1.0, 0.0)
    r.assign(c.id)
    r.observe(1.3, 1.0, 1.0)
    assert c.x == 1.0 and c.sightings == 2


def test_unassign_returns_without_failure():
    r = CubeRegistry(confirm_sightings=1)
    c = r.observe(1.0, 1.0, 0.0)
    r.assign(c.id)
    r.unassign(c.id)
    assert c.status == 'pending' and c.failures == 0


def test_failed_cube_goes_after_untried_ones():
    r = CubeRegistry(confirm_sightings=1, failure_limit=3)
    near = r.observe(1.0, 0.0, 0.0)
    far = r.observe(4.0, 0.0, 0.0)
    r.assign(near.id)
    r.report(near.id, False, 1.0)
    assert r.next_task((0, 0), 2.0).id == far.id
