#!/bin/bash
cd $PROJECT_ROOT/src/client/ros2
source /opt/ros/humble/setup.bash
colcon build
cd ../
source /opt/ros/humble/setup.bash
source install/local_setup.bash
#launch anything here
ros2 launch px4_ros_com sensor_combined_listener.launch.py
ros2 run px4_websocket_bridge px4_websocket_node


