import os

from botzilla_fleet.params_rewrite import collector_nav2_params, namespaced
import yaml

NAV2 = os.path.join(os.path.dirname(__file__), '..', '..', 'botzilla_navigation',
                    'config', 'nav2_params.yaml')


def load():
    with open(NAV2) as f:
        return yaml.safe_load(f)


def walk(node, path=()):
    if isinstance(node, dict):
        for k, v in node.items():
            yield from walk(v, path + (k,))
    elif isinstance(node, list):
        for v in node:
            yield from walk(v, path)
    else:
        yield path, node


def test_absolute_topics_moved_into_namespace():
    p = collector_nav2_params(load(), 'bz2')
    for path, val in walk(p):
        if path and path[-1].endswith('topic') and isinstance(val, str):
            assert not val.startswith('/') or val == '/map' or val.startswith('/bz2/'), \
                (path, val)
    # Costmap layers must name the robot's topic absolutely: a relative 'scan' in a
    # costmap resolves to /bz2/local_costmap/scan (see params_rewrite docstring).
    lc = p['local_costmap']['local_costmap']['ros__parameters']
    assert lc['voxel_layer']['scan']['topic'] == '/bz2/scan'
    assert p['bt_navigator']['ros__parameters']['odom_topic'] == '/bz2/odom'


def test_leader_only_plugins_removed():
    p = collector_nav2_params(load(), 'bz2')
    planner = p['planner_server']['ros__parameters']
    assert 'SweepStraight' not in planner['planner_plugins']
    assert 'SweepStraight' not in planner
    gc = p['global_costmap']['global_costmap']['ros__parameters']
    assert 'coverage_layer' not in gc['plugins'] and 'coverage_layer' not in gc
    assert gc['static_layer']['map_topic'] == '/map'


def test_leader_params_untouched_and_amcl_merged():
    base = load()
    before = yaml.safe_dump(base)
    p = collector_nav2_params(base, 'bz2', {'amcl': {'ros__parameters': {'x': 1}}})
    assert yaml.safe_dump(base) == before
    assert p['amcl']['ros__parameters']['x'] == 1


def test_namespaced():
    assert namespaced({'a': 1}, '/bz2/') == {'bz2': {'a': 1}}
    assert namespaced({'a': 1}, '') == {'a': 1}


def test_fleet_sources_point_at_the_collectors_own_topics():
    # The leader reads /fleet/robot_obstacles (the collector's body); the collector's
    # copy must read the leader's body on /bz2/fleet/robot_obstacles, and its cube
    # source must not be the leader's /fleet/cube_obstacles (its target would block it).
    p = collector_nav2_params(load(), 'bz2')
    lc = p['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']
    gc = p['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']
    for layer in (lc, gc):
        assert layer['fleet_robots']['topic'] == '/bz2/fleet/robot_obstacles'
        assert layer['fleet_cubes']['topic'] == '/bz2/fleet/cube_obstacles'
