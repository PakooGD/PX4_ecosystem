# Установка
1. Клонируйте репозиторий: `git clone https://github.com/yourusername/project.git`
2. Запустите скрипт установки: `./scripts/setup.sh`
3. Настройте алиасы: `source config/aliases.sh`

# Запуск
- Запустите Gazebo: `start_gazebo`
- Запустите PX4: `start_px4`
- Запустите сервер: `start_server`


A. Installation
1. Install ROS2 Humble

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
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc


2. Install PX4-Autopilot

cd
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
make px4_sitl

3. Setup Micro XRCE-DDS Agent & Client

cd
pip install --user -U empy==3.3.4 pyros-genmsg setuptools
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/

Далее установим node.js
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
\. "$HOME/.nvm/nvm.sh"
nvm install 22
npm install -g yarn

4. Download and Install QGC (Optional but Recommended)

cd
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 -y
sudo apt install wget -y
sudo apt install libxcb-cursor0 -y
git clone --recursive -j8 https://github.com/mavlink/qgroundcontrol.git
git submodule update --recursive
sudo bash ./qgroundcontrol/tools/setup/install-dependencies-debian.sh
cd qgroundcontrol
~/Qt/6.8.2/gcc_64/bin/qt-cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
./build/Debug/QGroundControl

!!!!The required version of Qt is 6.8.2 (only).
https://www.qt.io/offline-installers ///
Можно загрузить оффлайн архив и установить вручную:
- sudo apt update
- sudo apt install build-essential libgl1-mesa-dev libssl-dev libxkbcommon-dev libxcb-xinerama0-dev libxcb-xinerama0
- tar -xvf qt-everywhere-src-6.8.2.tar.xz
- cd qt-everywhere-src-6.8.2
- ./configure -prefix /opt/Qt/6.8.2 -opensource -confirm-license -nomake examples -nomake tests или mkdir build >> cd build >> cmake -GNinja -DCMAKE_INSTALL_PREFIX=/opt/Qt/6.8.2 -DQT_BUILD_EXAMPLES=OFF -DQT_BUILD_TESTS=OFF ..
- make -j$(nproc) ИЛИ ninja -j$(nproc)
- sudo make install ИЛИ sudo ninja install
- export PATH=/opt/Qt/6.8.2/bin:$PATH
- qmake --version - проверка установки

5. Download foxglove studio

git clone https://github.com/dagar/foxglove-studio.git
cd foxglove-studio
yarn install
yarn build:packages
yarn start


или

docker build -t foxglove-studio .
docker run -it --rm foxglove-studio

sudo apt install ros-$ROS_DISTRO-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

B. Simulation
1. Open a new terminal and run MicroXRCEAgent:

MicroXRCEAgent udp4 -p 8888

2. In another terminal run QGC:(Only if installed)

./build/Debug/QGroundControl

3. In another terminal start gz_x500 simulated drone:

cd ~/PX4-Autopilot && make px4_sitl gz_x500
