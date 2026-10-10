"""
params_rewrite.py — derive the collector's Nav2/EKF parameters from the leader's files.

ROS-free; tested in test/test_params_rewrite.py.

The collector runs the same Nav2 tuning as the leader (botzilla_navigation's
nav2_params.yaml) — a separate copy would drift — with only what has to differ:

- Namespacing. A params file's node keys ("controller_server:") match un-namespaced
  nodes only; the collector's nodes are /bz2/controller_server etc. Nesting the whole
  file under the namespace key is how nav2_bringup's RewrittenYaml(root_key=...) does it.
- Topics. Absolute topic names ("/scan") would read the LEADER's sensors over the shared
  DDS graph, so every absolute *topic parameter is moved into the namespace ("/scan" ->
  "/bz2/scan") — except the map, which is the leader's map as republished for the
  collector (COLLECTOR_MAP_TOPIC). NOT made
  relative: a costmap is its own node in a sub-namespace (/bz2/local_costmap/
  local_costmap), and its layers resolve a relative "scan" to /bz2/local_costmap/scan,
  which nothing publishes. Measured on Jazzy by activating the namespaced
  controller_server: that is exactly what it subscribed to, i.e. a costmap blind to
  every obstacle. Topics the leader's file already gives as relative (behavior_server's
  local_costmap/costmap_raw) are meant relative to the server and are left alone.
- Plugins the collector does not use: the SweepStraight planner and the coverage cost
  layer belong to the leader's search, and the coverage layer reads the leader's
  /coverage_cost_map.
- AMCL, which the leader does not run (it has RTAB-Map).
"""
import copy

# Not the leader's /map itself but the fleet manager's copy of it, with the collector's
# own footprint and HOME cleared: the leader's LiDAR maps the parked collector as an
# obstacle, and with the raw /map the collector started inside it and its HOME was
# lethal, so every plan from the start and every delivery failed (2026-10-05).
COLLECTOR_MAP_TOPIC = 'fleet/map'      # resolved under the collector's namespace
COLLECTOR_PLANNERS_DROP = ('SweepStraight',)
COLLECTOR_COSTMAP_LAYERS_DROP = ('coverage_layer',)
# The collector carries the grabber arms; the leader is now a bare circular Kobuki
# (robot_radius in nav2_params.yaml). The collector's costmaps get the measured arm
# polygon back, padded by 0.05 m on every side exactly as the leader's was while it had
# the arms (see the derivation in nav2_params.yaml's local_costmap).
COLLECTOR_FOOTPRINT = '[[0.36, 0.215], [0.36, -0.215], [-0.22, -0.215], [-0.22, 0.215]]'


def _namespace_topics(node, ns):
    if isinstance(node, dict):
        for key, val in node.items():
            if (isinstance(val, str) and key.endswith('topic') and key != 'map_topic'
                    and val.startswith('/')):
                node[key] = f'/{ns}{val}'
            else:
                _namespace_topics(val, ns)
    elif isinstance(node, list):
        for item in node:
            _namespace_topics(item, ns)


def _drop_plugins(params, list_key, names):
    plugins = params.get(list_key)
    if not isinstance(plugins, list):
        return
    params[list_key] = [p for p in plugins if p not in names]
    for name in names:
        params.pop(name, None)


def collector_nav2_params(base, ns, amcl=None):
    """Return the collector's Nav2 params dict (node keys not yet nested under ns)."""
    ns = ns.strip('/')
    p = copy.deepcopy(base)
    _namespace_topics(p, ns)

    # bt_navigator waits only 1 s (default) for each action server while loading its tree;
    # on the loaded Pi, discovering compute_path_to_pose took longer, so activation
    # failed and the lifecycle manager aborted the whole bring-up
    # (run_logs/collector-20261005-212145 on the Pi).
    bt = p.setdefault('bt_navigator', {}).setdefault('ros__parameters', {})
    bt['wait_for_service_timeout'] = 10000
    # Same machine, same cause, at run time: each BT action node waits only
    # default_server_timeout (20 ms) for its server to acknowledge a goal. On the Pi the
    # planner missed that, ComputePathToPose failed, and the delivery goal was aborted
    # 30 ms after capture (collector run of 2026-10-05 22:11, task 2). 1000 ms still
    # timed out 12 times in run 8 of multi_robot_runs.md, so 5000.
    bt['default_server_timeout'] = 5000

    planner = p.get('planner_server', {}).get('ros__parameters', {})
    _drop_plugins(planner, 'planner_plugins', COLLECTOR_PLANNERS_DROP)

    gc = p.get('global_costmap', {}).get('global_costmap', {}).get('ros__parameters', {})
    _drop_plugins(gc, 'plugins', COLLECTOR_COSTMAP_LAYERS_DROP)
    for costmap in ('global_costmap', 'local_costmap'):
        params = p.get(costmap, {}).get(costmap, {}).get('ros__parameters', {})
        if params:
            params.pop('robot_radius', None)
            params['footprint'] = COLLECTOR_FOOTPRINT
        if isinstance(params.get('static_layer'), dict):
            params['static_layer']['map_topic'] = f'/{ns}/{COLLECTOR_MAP_TOPIC}'

    if amcl:
        p.update(copy.deepcopy(amcl))
        a = p.get('amcl', {}).get('ros__parameters', {})
        if 'map_topic' in a:
            a['map_topic'] = f'/{ns}/{COLLECTOR_MAP_TOPIC}'
    return p


def namespaced(params, ns):
    """Nest a params dict under a namespace key, so it matches /ns/<node> names."""
    ns = ns.strip('/')
    return {ns: params} if ns else params
