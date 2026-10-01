"""
nav2.launch.py

Milestone 3 (PHASE_1_IMPLEMENTATION_PLAN.md, Option A): Nav2 goal-following on
top of the map RTAB-Map builds (rtabmap.launch.py must already be running and
publishing /map + map->odom TF — this file does not launch SLAM/localization).

A slim, hand-picked subset of nav2_bringup's navigation_launch.py — only the
nodes actually needed for a single NavigateToPose goal: controller_server,
planner_server, behavior_server, bt_navigator. Skips
route_server/collision_monitor/docking_server/smoother_server/waypoint_follower,
which navigation_launch.py always brings up regardless of whether they're
configured or needed, adding failure surface (e.g. route_server expects a
routing graph file we don't have) for no benefit at this milestone.

cmd_vel chain: controller_server/behavior_server -> 'cmd_vel_nav' ->
botzilla_control's velocity_smoother -> 'cmd_vel' -> kobuki_base_node.

History, because this chain changed twice. Originally the servers published
straight to 'cmd_vel'. nav2_velocity_smoother was tried in between and removed:
on hardware it silently stopped republishing — controller_server kept publishing
to 'cmd_vel_nav' while '/cmd_vel' had no publisher at all, the node reporting
lifecycle state active and logging no error, so every navigation goal stalled.
The note left behind then was that the smoother was optional because "DWB's own
accel limits already bound the output".

That last part turned out to be wrong, and is why a smoother is back. DWB's
default trajectory generator (StandardTrajectoryGenerator) samples the entire
velocity range every cycle regardless of current velocity; acc_lim_* only shape
the simulated trajectory and never bound the emitted command. Measured on
hardware: 24 angular sign changes in 80 s, with visible jerking. nav2_params.yaml
now selects LimitedAccelGenerator to fix that inside DWB, but behavior_server's
BackUp/Spin bypass DWB entirely — so the smoother covers them. It is our own node
(botzilla_control/velocity_smoother.py), built so the earlier silent failure
cannot recur: it publishes on a timer, decays to zero on input timeout rather
than latching, and logs its in/out rates.

Simulation parameters: passing use_sim_time:=true additionally layers
config/nav2_params_sim.yaml on top of the main params file (ROS 2 merges multiple
params files in order, last one winning). That overlay carries only the values that
must differ in Gazebo — see its own header — rather than a full duplicate config that
would drift out of sync with the hardware tuning. Applied automatically so it cannot
be forgotten: the values it corrects cause a stall that looks like a planner bug.

Usage (hardware is the default; pass use_sim_time:=true for Gazebo):
  ros2 launch botzilla_navigation nav2.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SIM_PARAMS_FILENAME = 'nav2_params_sim.yaml'

# velocity_smoother collapses an opposing command smaller than its reversal deadband to
# zero, on the reasoning that the physical base could not have executed it anyway (measured
# mechanical deadband: 0.009 m/s, 0.121 rad/s). Simulation has no such deadband, so those
# same commands ARE executable there and discarding them deadlocks the robot: DWB emits
# alternating +/-0.021 rad/s heading corrections, every one is collapsed, the robot never
# turns, and the correction never stops being needed. Confirmed live in Gazebo with 2.37 m
# of clear floor ahead and zero ObstacleFootprint vetoes — the robot simply stops.
#
# Near-zero rather than exactly zero: the reversal-confirmation path (3 cycles) is still
# wanted for genuine direction changes, and only the "too small to be real intent"
# shortcut needs disabling. Hardware keeps the node's own defaults.
SIM_REVERSAL_DEADBAND_X = 0.002       # m/s
SIM_REVERSAL_DEADBAND_THETA = 0.005   # rad/s


def _resolve_param_files(context):
    """Build the ordered params list, appending the sim overlay when running on sim time.

    Kept in an OpaqueFunction because the decision needs the *resolved* value of
    use_sim_time, which a LaunchConfiguration substitution cannot provide while the
    launch description is still being constructed.
    """
    pkg_share = get_package_share_directory('botzilla_navigation')
    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in (
        'true', '1', 'yes'
    )
    files = [params_file]
    if use_sim_time:
        files.append(os.path.join(pkg_share, 'config', SIM_PARAMS_FILENAME))
    return files, use_sim_time


def _launch_nav2(context, *_args, **_kwargs):
    param_files, use_sim_time = _resolve_param_files(context)
    autostart = LaunchConfiguration('autostart')
    params = param_files + [{'use_sim_time': use_sim_time}]
    collision_monitor = (
        LaunchConfiguration('collision_monitor').perform(context).lower() == 'true'
    )

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]
    servers = [
        ('nav2_controller', 'controller_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
    ]
    # The two nodes that publish velocity are redirected to 'cmd_vel_nav' so
    # velocity_smoother can sit between them and the base (see SMOOTHER note below).
    # planner_server/bt_navigator publish no velocity, so remapping them is harmless but
    # pointless — keep the remap targeted so the topic graph stays readable.
    velocity_publishers = {'controller_server', 'behavior_server'}
    # Recovery order: BackUp before Spin. Set here rather than in nav2_params.yaml
    # because the value has to be an absolute path, which only resolves at launch.
    # Scoped to bt_navigator alone for the same reason the cmd_vel remap is scoped —
    # keep per-node settings off nodes that have no use for them.
    #
    # Without this, bt_navigator falls back to nav2's built-in tree and the reorder
    # reaches almost nothing: in run 18, 32 of 41 goals (78%) went through the DEFAULT
    # tree, and only 9 through navigate_to_pose_sweep_straight.xml. See that file and
    # navigate_to_pose_backup_first.xml for why Spin-first is wrong on this robot.
    default_bt = os.path.join(
        get_package_share_directory('botzilla_navigation'),
        'behavior_trees', 'navigate_to_pose_backup_first.xml',
    )
    nodes = [
        Node(
            package=pkg, executable=exe, name=exe, output='screen',
            parameters=(
                params + [{'default_nav_to_pose_bt_xml': default_bt}]
                if exe == 'bt_navigator' else params
            ),
            remappings=([('cmd_vel', 'cmd_vel_nav')] if exe in velocity_publishers else []),
        )
        for pkg, exe in servers
    ]

    # SMOOTHER: 'cmd_vel_nav' -> 'cmd_vel'.
    #
    # A smoother used to live here (nav2_velocity_smoother) and was removed because it
    # silently stopped republishing on hardware — see this file's module docstring. That
    # docstring's conclusion was "if smoothing is wanted later, do it in a node we own and
    # can instrument rather than reintroducing a silent failure point", and this is that
    # node: botzilla_control's velocity_smoother, which publishes on a timer (so /cmd_vel
    # always has a live publisher), decays to zero on input timeout instead of latching,
    # and logs input/output rates periodically so the old silent failure is observable.
    #
    # It is NOT redundant with DWB's LimitedAccelGenerator: behavior_server's BackUp and
    # Spin publish velocity directly and bypass DWB's trajectory generator entirely, and
    # those recovery behaviours were the most violent part of the observed motion.
    #
    # Deliberately not a lifecycle node: it must be transporting velocity before and after
    # the managed nodes transition, and adding it to lifecycle_nodes would make the whole
    # navigation bringup fail if it were absent.
    # Reversal deadbands are overridden ONLY on sim time — see SIM_REVERSAL_DEADBAND_*.
    # On hardware the parameters are left unset so the node's own measured defaults apply.
    smoother_params = {'use_sim_time': use_sim_time}
    if use_sim_time:
        smoother_params['reversal_deadband_x'] = SIM_REVERSAL_DEADBAND_X
        smoother_params['reversal_deadband_theta'] = SIM_REVERSAL_DEADBAND_THETA
    # With the collision monitor enabled it is spliced in ahead of the smoother, so the
    # smoother's input moves from 'cmd_vel_nav' to the monitor's checked output. With it
    # disabled nothing is remapped and the chain is exactly as it was.
    smoother_remaps = [('cmd_vel_nav', 'cmd_vel_safe')] if collision_monitor else []
    nodes.append(Node(
        package='botzilla_control',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[smoother_params],
        remappings=smoother_remaps,
    ))

    # COLLISION MONITOR (opt-in): 'cmd_vel_nav' -> 'cmd_vel_safe'. See
    # config/collision_monitor.yaml for why it is off by default and what it guards
    # against — chiefly that the arms stop the bumpers from ever triggering, so without
    # it nothing checks for contact independently of the costmap.
    if collision_monitor:
        monitor_params = os.path.join(
            get_package_share_directory('botzilla_navigation'),
            'config', 'collision_monitor.yaml',
        )
        nodes.append(Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[monitor_params, {'use_sim_time': use_sim_time}],
        ))
        lifecycle_nodes.append('collision_monitor')

    nodes.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes,
        }],
    ))
    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('botzilla_navigation')
    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description=(
            'Use simulation (Gazebo) clock. Defaults false: on hardware nothing publishes '
            '/clock, so a stray true freezes ROS time and every wall-timer-driven Nav2 node '
            'silently stops firing with no error logged.'
        ),
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the Nav2 parameters file',
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically bring the lifecycle nodes up to the active state',
    )
    collision_monitor_arg = DeclareLaunchArgument(
        'collision_monitor',
        default_value='false',
        description=(
            'Splice nav2_collision_monitor into the cmd_vel chain as an independent, '
            'scan-based stop before contact. Off by default: it changes the chain that '
            "this file's docstring records a third-party node silently breaking."
        ),
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        autostart_arg,
        collision_monitor_arg,
        OpaqueFunction(function=_launch_nav2),
    ])
