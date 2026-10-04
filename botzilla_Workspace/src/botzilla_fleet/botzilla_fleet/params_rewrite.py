"""
params_rewrite.py — derive the collector's Nav2/EKF parameters from the leader's files.

ROS-free; tested in test/test_params_rewrite.py.

The collector runs the same Nav2 tuning as the leader (botzilla_navigation's
nav2_params.yaml) — a separate copy would drift — with only what has to differ:

- Namespacing. A params file's node keys ("controller_server:") match un-namespaced
  nodes only; the collector's nodes are /bz2/controller_server etc. Nesting the whole
  file under the namespace key is how nav2_bringup's RewrittenYaml(root_key=...) does it.
- Topics. Absolute topic names ("/scan") would read the LEADER's sensors over the shared
  DDS graph. Every *topic parameter is made relative so it resolves in the namespace —
  except the map, which is deliberately the leader's /map.
- Plugins the collector does not use: the SweepStraight planner and the coverage cost
  layer belong to the leader's search, and the coverage layer reads the leader's
  /coverage_cost_map.
- AMCL, which the leader does not run (it has RTAB-Map).
"""
import copy

LEADER_MAP_TOPIC = '/map'
COLLECTOR_PLANNERS_DROP = ('SweepStraight',)
COLLECTOR_COSTMAP_LAYERS_DROP = ('coverage_layer',)


def _relativise_topics(node):
    if isinstance(node, dict):
        for key, val in node.items():
            if (isinstance(val, str) and key.endswith('topic') and key != 'map_topic'
                    and val.startswith('/')):
                node[key] = val.lstrip('/')
            else:
                _relativise_topics(val)
    elif isinstance(node, list):
        for item in node:
            _relativise_topics(item)


def _drop_plugins(params, list_key, names):
    plugins = params.get(list_key)
    if not isinstance(plugins, list):
        return
    params[list_key] = [p for p in plugins if p not in names]
    for name in names:
        params.pop(name, None)


def collector_nav2_params(base, amcl=None):
    """Return the collector's (un-namespaced) Nav2 params dict from the leader's."""
    p = copy.deepcopy(base)
    _relativise_topics(p)

    planner = p.get('planner_server', {}).get('ros__parameters', {})
    _drop_plugins(planner, 'planner_plugins', COLLECTOR_PLANNERS_DROP)

    gc = p.get('global_costmap', {}).get('global_costmap', {}).get('ros__parameters', {})
    _drop_plugins(gc, 'plugins', COLLECTOR_COSTMAP_LAYERS_DROP)
    for costmap in ('global_costmap', 'local_costmap'):
        params = p.get(costmap, {}).get(costmap, {}).get('ros__parameters', {})
        if isinstance(params.get('static_layer'), dict):
            params['static_layer']['map_topic'] = LEADER_MAP_TOPIC

    if amcl:
        p.update(copy.deepcopy(amcl))
    return p


def namespaced(params, ns):
    """Nest a params dict under a namespace key, so it matches /ns/<node> names."""
    ns = ns.strip('/')
    return {ns: params} if ns else params
