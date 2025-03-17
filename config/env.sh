# Переменные окружения
export PROJECT_ROOT=~/PX4_ecosystem
export CLIENT_DIR="$PROJECT_ROOT/src/client"
export SERVER_DIR="$PROJECT_ROOT/src/server"
export QT_PATH=/usr/local/Qt-6.8.2/bin:$PATH
export PATH=$PATH:$PROJECT_ROOT/src/client/PX4-Autopilot/Tools
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$PROJECT_ROOT/src/client/PX4-Autopilot/Tools/simulation/gz/models
source /opt/ros/humble/setup.bash