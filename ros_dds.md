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
