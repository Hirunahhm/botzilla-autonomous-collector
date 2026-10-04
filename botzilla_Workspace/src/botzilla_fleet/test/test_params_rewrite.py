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


def test_no_absolute_topics_except_map():
    p = collector_nav2_params(load())
    for path, val in walk(p):
        if path and path[-1].endswith('topic') and isinstance(val, str):
            assert not val.startswith('/') or val == '/map', (path, val)


def test_leader_only_plugins_removed():
    p = collector_nav2_params(load())
    planner = p['planner_server']['ros__parameters']
    assert 'SweepStraight' not in planner['planner_plugins']
    assert 'SweepStraight' not in planner
    gc = p['global_costmap']['global_costmap']['ros__parameters']
    assert 'coverage_layer' not in gc['plugins'] and 'coverage_layer' not in gc
    assert gc['static_layer']['map_topic'] == '/map'


def test_leader_params_untouched_and_amcl_merged():
    base = load()
    before = yaml.safe_dump(base)
    p = collector_nav2_params(base, {'amcl': {'ros__parameters': {'x': 1}}})
    assert yaml.safe_dump(base) == before
    assert p['amcl']['ros__parameters']['x'] == 1


def test_namespaced():
    assert namespaced({'a': 1}, '/bz2/') == {'bz2': {'a': 1}}
    assert namespaced({'a': 1}, '') == {'a': 1}
