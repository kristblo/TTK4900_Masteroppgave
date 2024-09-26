#! /bin/bash
colcon build --packages-select orca_moveit_config orca_moveit_config
echo "built"
source ./install/setup.bash
echo "sourced"
