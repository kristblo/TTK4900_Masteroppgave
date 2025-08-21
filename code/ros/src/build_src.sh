#! /bin/bash
colcon build --packages-select orca_moveit_config orca_moveit_config
colcon build --packages-select orca_ctrl_msgs
colcon build --packages-select control_listener
echo "built"
source ./install/setup.bash
echo "sourced"
