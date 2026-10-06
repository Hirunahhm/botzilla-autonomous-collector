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
            assert not val.startswith('/') or val.startswith('/bz2/'), \
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
    assert gc['static_layer']['map_topic'] == '/bz2/fleet/map'


def test_leader_params_untouched_and_amcl_merged():
    base = load()
    before = yaml.safe_dump(base)
    p = collector_nav2_params(base, 'bz2', {'amcl': {'ros__parameters': {'x': 1}}})
    assert yaml.safe_dump(base) == before
    assert p['amcl']['ros__parameters']['x'] == 1


def test_namespaced():
    assert namespaced({'a': 1}, '/bz2/') == {'bz2': {'a': 1}}
    assert namespaced({'a': 1}, '') == {'a': 1}


def test_fleet_layer_reads_the_collectors_own_grid():
    # The leader's fleet_layer reads /fleet/obstacle_grid (the collector + cubes); the
    # collector's must read its own grid (the leader + cubes other than its target).
    p = collector_nav2_params(load(), 'bz2')
    for costmap in ('local_costmap', 'global_costmap'):
        params = p[costmap][costmap]['ros__parameters']
        assert 'fleet_layer' in params['plugins']
        # After inflation: the other robot and the cubes are not inflated (see
        # fleet_manager_node OBSTACLE_GRID_HZ for the deadlock that caused).
        assert params['plugins'].index('fleet_layer') > params['plugins'].index('inflation_layer')
        assert params['fleet_layer']['topic'] == '/bz2/fleet/obstacle_grid'
        assert params['fleet_layer']['lethal'] is True


def test_amcl_and_static_layers_use_the_cleaned_collector_map():
    p = collector_nav2_params(load(), 'bz2', {'amcl': {'ros__parameters': {'map_topic': '/map'}}})
    assert p['amcl']['ros__parameters']['map_topic'] == '/bz2/fleet/map'


def test_collector_gets_the_arm_polygon_leader_keeps_the_circle():
    base = load()
    for costmap in ('local_costmap', 'global_costmap'):
        leader = base[costmap][costmap]['ros__parameters']
        assert 'robot_radius' in leader and 'footprint' not in leader
    p = collector_nav2_params(base, 'bz2')
    for costmap in ('local_costmap', 'global_costmap'):
        params = p[costmap][costmap]['ros__parameters']
        assert 'robot_radius' not in params
        assert params['footprint'].startswith('[[0.36, 0.215]')
