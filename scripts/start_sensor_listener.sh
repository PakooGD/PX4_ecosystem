#!/bin/bash
cd $PROJECT_ROOT/src/client/ros2
source /opt/ros/humble/setup.bash
colcon build
source install/local_setup.bash
ros2 launch px4_ros_com sensor_combined_listener.launch.py