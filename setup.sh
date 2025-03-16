#!/bin/bash

# Установка глобальных переменных
PROJECT_ROOT=$(pwd)
CLIENT_DIR="$PROJECT_ROOT/src/client"
SERVER_DIR="$PROJECT_ROOT/src/server"

# Функция для проверки и установки пакетов
install_package() {
    local package_name=$1
    local install_command=$2

    if ! command -v "$package_name" &> /dev/null; then
        echo "Установка $package_name..."
        eval "$install_command"
    else
        echo "$package_name уже установлен."
    fi
}

######################################## Dependencies ########################################################

# Установка Node.js через nvm
install_package "node" "curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash && \. \"$HOME/.nvm/nvm.sh\" && nvm install 22 && corepack enable"

# Установка yq
install_package "yq" "sudo snap install yq"

# Установка python
install_package "pip" "pip install --user -U empy==3.3.4 pyros-genmsg setuptools"

######################################## ROS 2 Humble ########################################################

echo "Установка ROS 2 Humble..."

# Проверка, установлен ли ROS 2
if ! command -v ros2 &> /dev/null; then
    sudo apt update && sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install -y curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update && sudo apt upgrade -y
    sudo apt install -y ros-humble-desktop ros-dev-tools
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source /opt/ros/humble/setup.bash
else
    echo "ROS 2 уже установлен."
fi

# Клонирование репозиториев PX4 для ROS 2
echo "Клонирование px4_msgs и px4_ros_com..."

mkdir -p "$CLIENT_DIR/ros2/src"
cd "$CLIENT_DIR/ros2/src"
if [ ! -d "px4_msgs" ]; then
    git clone https://github.com/PX4/px4_msgs.git --recursive
else
    echo "px4_msgs уже клонирован."
fi

if [ ! -d "px4_ros_com" ]; then
    git clone https://github.com/PX4/px4_ros_com.git --recursive
else
    echo "px4_ros_com уже клонирован."
fi

# Установка Foxglove Bridge
echo "Клонирование Foxglove Bridge..."

if [ ! -d "ros-foxglove-bridge" ]; then
    git clone https://github.com/foxglove/ros-foxglove-bridge.git --recursive
else
    echo "Foxglove Bridge уже установлен."
fi

cd "$CLIENT_DIR/ros2"
source /opt/ros/humble/setup.bash
colcon build
source install/local_setup.bash

cd "$PROJECT_ROOT"

######################################## PX4-Autopilot ########################################################

echo "Установка PX4-Autopilot и Gazebo Harmonic..."

if [ ! -d "$CLIENT_DIR/PX4-Autopilot" ]; then
    cd "$CLIENT_DIR"
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    cd "$CLIENT_DIR/PX4-Autopilot"
    bash ./Tools/setup/ubuntu.sh
    sudo apt-get update
    sudo apt-get install -y curl lsb-release gnupg
    sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    sudo apt-get update
    sudo apt-get install -y gz-harmonic
    make px4_sitl
else
    echo "PX4-Autopilot и Gazebo Harmonic уже установлены."
fi
cd "$PROJECT_ROOT"

######################################## Micro XRCE-DDS Agent ########################################################

echo "Установка Micro XRCE-DDS Agent..."

if [ ! -d "$CLIENT_DIR/micro_xrce_dds" ]; then
    mkdir -p "$CLIENT_DIR/micro_xrce_dds"
    cd "$CLIENT_DIR/micro_xrce_dds"
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git .
    mkdir build
else
    echo "Micro XRCE-DDS Agent уже установлен."
fi
cd "$PROJECT_ROOT"

######################################## QGroundControl ########################################################

echo "Установка QGroundControl..."

if [ ! -d "$CLIENT_DIR/qgroundcontrol" ]; then
    cd "$CLIENT_DIR"
    sudo usermod -a -G dialout $USER
    sudo apt-get remove -y modemmanager
    sudo apt install -y gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl libqt5gui5 libfuse2 wget libxcb-cursor0
    sudo apt install -y build-essential libgl1-mesa-dev libssl-dev libxkbcommon-dev libxcb-xinerama0-dev libxcb-xinerama0
    git clone --recursive -j8 https://github.com/mavlink/qgroundcontrol.git 
    git submodule update --recursive
        cd "$CLIENT_DIR/qgroundcontrol"
    sudo bash ./tools/setup/install-dependencies-debian.sh
else
    echo "QGroundControl уже установлен."
fi
cd "$PROJECT_ROOT"

######################################## FOXGLOVE ########################################################

echo "Установка Foxglove Studio..."

if [ ! -d "$CLIENT_DIR/foxglove-opensource" ]; then
    cd "$CLIENT_DIR"
    git clone https://github.com/AD-EYE/foxglove-opensource.git
    cd "$CLIENT_DIR/foxglove-opensource"
    yarn install
else
    echo "Foxglove Studio уже установлен."
fi

cd "$PROJECT_ROOT"


######################################## SERVER ########################################################

echo "Установка зависимостей серверна..."

cd "$SERVER_DIR"
# Установка зависимостей
if [ ! -d "$SERVER_DIR/node_modules" ]; then
    npm install
else
    echo "Зависимости сервера уже установлены."
fi

cd "$PROJECT_ROOT"

######################################## ENV-setup ########################################################

echo "Настройка переменных окружения..."

# Делаем все скрипты в директории scripts исполняемыми
echo "Делаем все скрипты в директории scripts исполняемыми..."
chmod +x "$PROJECT_ROOT/scripts"/*.sh

# Добавляем переменные окружения в .bashrc
echo "Добавляем переменные окружения в .bashrc, если отсутствуют..."
if ! grep -q "source $PROJECT_ROOT/config/env.sh" ~/.bashrc; then
    echo "source $PROJECT_ROOT/config/env.sh" >> ~/.bashrc
fi
if ! grep -q "source $PROJECT_ROOT/config/aliases.sh" ~/.bashrc; then
    echo "source $PROJECT_ROOT/config/aliases.sh" >> ~/.bashrc
fi
source ~/.bashrc

echo "Установка завершена!"