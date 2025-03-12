#!/bin/bash
cd $PROJECT_ROOT/src/client/ros2
source /opt/ros/humble/setup.bash
colcon build
source install/local_setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml