source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="hirunahhm.local:11811"
export ROS_SUPER_CLIENT=true

unset ROS_LOCALHOST_ONLY
unset ROS_AUTOMATIC_DISCOVERY_RANGE
ros2 daemon stop
ros2 topic list

# (One-time only) copy the RViz config
scp hirunahhm@hirunahhm.local:~/botzilla_nav_debug.rviz ~/

# Launch RViz
rviz2 -d ~/botzilla_nav_debug.rviz


# ---------------------------------------------------------------------------
# Two-robot runs: the COLLECTOR's view in a second RViz window
# ---------------------------------------------------------------------------
# Open a second terminal on the laptop and run the same six export/unset lines as
# above (the collector joins the leader's discovery server, so the laptop reaches both
# robots through hirunahhm.local). Then:

# (One-time only, and again whenever the config changes) copy the collector's config
scp hirunahhm@hirunahhm.local:~/Desktop/Projects/sem5/final-project-botzilla/tools/rviz/botzilla_collector.rviz ~/

# Launch RViz on the collector's TF tree. The remaps are required: the collector
# publishes its TF on /bz2/tf, and with the leader's /tf the robot model and scan
# would be drawn at the leader's pose.
rviz2 -d ~/botzilla_collector.rviz --ros-args -r /tf:=/bz2/tf -r /tf_static:=/bz2/tf_static

# What it shows: the collector's map (the leader's map with its own floor cleared),
# its global/local costmaps, plan, LiDAR scan, footprint, AMCL particles and pose,
# the fleet's cube markers, and the leader drawn as an obstacle.
# Tools: "2D Pose Estimate" re-seeds the collector's AMCL (/bz2/initialpose) if its
# particle cloud is off; "2D Goal Pose" sends it a test goal (/bz2/goal_pose).
# "AMCL particles" needs nav2_rviz_plugins on the laptop:
#   sudo apt install ros-jazzy-nav2-rviz-plugins
# Everything here streams from the Pi over the hotspot; avoid adding camera images.
