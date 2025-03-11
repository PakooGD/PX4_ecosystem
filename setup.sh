#!/bin/bash

######################################## Dependencies ########################################################

# Node.js
if ! command -v node &> /dev/null; then
    curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
    \. "$HOME/.nvm/nvm.sh"
    nvm install 22
    corepack enable
fi

######################################## ROS 2 Humble ########################################################

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

######################################## PX4-Autopilot ########################################################

git clone https://github.com/PX4/PX4-Autopilot.git --recursive src/client/px4
bash ./src/client/px4/Tools/setup/ubuntu.sh
cd src/client/px4
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
make px4_sitl
cd ../../../

######################################## Micro XRCE-DDS Agent ########################################################

pip install --user -U empy==3.3.4 pyros-genmsg setuptools
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git src/client/client/micro_xrce_dds
cd src/client/micro_xrce_dds
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
cd ../../../

######################################## QGroundControl ########################################################

sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 -y
sudo apt install wget -y
sudo apt install libxcb-cursor0 -y
git clone --recursive -j8 https://github.com/mavlink/qgroundcontrol.git src/client/qgroundcontrol
cd src/client/qgroundcontrol
git submodule update --recursive
sudo bash ./tools/setup/install-dependencies-debian.sh
~/Qt/6.8.2/gcc_64/bin/qt-cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cd ../../../

######################################## FOXGLOVE ########################################################

# Установка Foxglove Studio
git clone https://github.com/dagar/foxglove-studio.git src/client/foxglove
cd src/client/foxglove
yarn install
cd ../../../

# Установка Foxglove Bridge
sudo apt install ros-humble-foxglove-bridge

######################################## SERVER ########################################################

cd src/server

# Установка зависимостей
yarn install

# Инициализация TypeScript
if [ ! -f "tsconfig.json" ]; then
  npx tsc --init
fi

######################################## ENV-setup ########################################################

echo "source $(pwd)/config/env.sh" >> ~/.bashrc
source ~/.bashrc


