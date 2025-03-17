#!/bin/bash

######################################## Настройка переменных окружения ########################################################

echo "Настройка переменных окружения..."

PROJECT_ROOT=$(pwd)
LOG_FILE="$PROJECT_ROOT/install.log"
exec > >(tee -a "$LOG_FILE") 2>&1
echo "Начало установки: $(date)"

# Делаем все скрипты в директории scripts исполняемыми
echo "Делаем все скрипты в директории scripts исполняемыми..."
chmod +x "$PROJECT_ROOT/scripts"/*.sh

# Добавление пользователя в группу dialout
if ! groups $USER | grep -q "\bdialout\b"; then
    echo "Добавление пользователя $USER в группу dialout..."
    sudo usermod -a -G dialout $USER
else
    echo "Пользователь $USER уже в группе dialout."
fi

# Добавляем переменные окружения в .bashrc
echo "Добавляем переменные окружения в .bashrc, если отсутствуют..."
if ! grep -q "source $PROJECT_ROOT/config/env.sh" ~/.bashrc; then
    echo "source $PROJECT_ROOT/config/env.sh" >> ~/.bashrc
fi
if ! grep -q "source $PROJECT_ROOT/config/aliases.sh" ~/.bashrc; then
    echo "source $PROJECT_ROOT/config/aliases.sh" >> ~/.bashrc
fi
source ~/.bashrc

######################################## Helpers ########################################################

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

# Функция для установки пакетов через apt с проверкой
install_apt_package() {
    local package_name=$1

    if ! dpkg -l | grep -q "^ii  $package_name "; then
        sudo apt-get install -y $package_name
    else
        echo "$package_name уже установлен."
    fi
}

# Функция для клонирования или обновления репозитория
clone_repo() {
    local repo_url=$1          # URL репозитория
    local target_dir=$2        # Целевая директория для клонирования
    local repo_name=${3:-$(basename "$repo_url" .git)}  # Имя папки (по умолчанию извлекается из URL)

    # Создаем целевую папку, если её нет
    mkdir -p "$target_dir"
    cd "$target_dir" || { echo "Ошибка: не удалось перейти в $target_dir"; return 1; }

    if [ -d "$repo_name" ] && [ -d "$repo_name/.git" ]; then
        echo "$repo_name уже клонирован и является Git-репозиторием."
        cd "$repo_name"
        echo "Обновление $repo_name..."
        git fetch --all --prune  # Обновляем информацию о всех ветках и удаляем устаревшие
        git reset --hard origin/main  # Сбрасываем изменения и переходим на последний коммит
        git submodule update --init --recursive  # Обновляем подмодули
        cd ..
    else
        echo "Клонирование $repo_name..."
        git clone "$repo_url" "$repo_name" --recursive -j8 || { echo "Ошибка: не удалось клонировать $repo_name"; return 1; }
    fi
}

######################################## Dependencies ########################################################

echo "Установка зависимостей и базовая настройка..."

# Установка Node.js через nvm
install_package "node" "curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash && \. \"$HOME/.nvm/nvm.sh\" && nvm install 22 && corepack enable"

# Установка yq
install_package "yq" "sudo snap install yq"

# Установка python
install_package "pip" "pip install --user -U empy==3.3.4 pyros-genmsg setuptools"

echo "Получение обновлений..."

sudo apt-get update

echo "Удаление modemmanager..."
# Удаление modemmanager
if dpkg -l | grep -q "^ii  modemmanager "; then
    sudo apt-get remove -y modemmanager
else
    echo "modemmanager уже удалён."
fi

install_apt_package "ros-humble-desktop"
install_apt_package "ros-dev-tools"
install_apt_package "curl"
install_apt_package "locales"cle
install_apt_package "lsb-release"
install_apt_package "gnupg"
install_apt_package "gnupg2"
install_apt_package "software-properties-common"
install_apt_package "gstreamer1.0-plugins-bad"
install_apt_package "gstreamer1.0-libav"
install_apt_package "gstreamer1.0-gl"
install_apt_package "libqt5gui5"
install_apt_package "libfuse2"
install_apt_package "wget"
install_apt_package "libxcb-cursor0"
install_apt_package "build-essential"
install_apt_package "libgl1-mesa-dev"
install_apt_package "libssl-dev"
install_apt_package "libxkbcommon-dev"
install_apt_package "libxcb-xinerama0-dev"

echo "Все зависимости установлены и система обновлена."

######################################## Создаем локали для английского языка ########################################################

echo "Настраиваем локали для английского языка с кодировкой UTF-8..."

# Проверка и генерация локалей, если они ещё не сгенерированы
if ! locale -a | grep -q "en_US.utf8"; then
    echo "Локаль en_US.UTF-8 не найдена. Генерация локалей..."
    sudo locale-gen en_US en_US.UTF-8
else
    echo "Локаль en_US.UTF-8 уже сгенерирована."
fi

# Проверка и обновление глобальных настроек локали, если они ещё не настроены
if ! grep -q "LANG=en_US.UTF-8" /etc/default/locale || ! grep -q "LC_ALL=en_US.UTF-8" /etc/default/locale; then
    echo "Глобальные настройки локали не найдены. Настройка локали..."
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
else
    echo "Глобальные настройки локали уже настроены."
fi

# Проверка и установка локали для текущей сессии, если она ещё не установлена
if [ "$LANG" != "en_US.UTF-8" ]; then
    echo "Локаль для текущей сессии не настроена. Установка локали..."
    export LANG=en_US.UTF-8
else
    echo "Локаль для текущей сессии уже настроена."
fi

echo "Проверка и настройка локали завершены."

######################################## ROS2 Humble ########################################################

# Установка ROS 2 Humble
echo "Установка ROS 2 Humble..."

if [ ! -f /opt/ros/humble/setup.bash ]; then
    sudo add-apt-repository universe
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source /opt/ros/humble/setup.bash
else
    echo "ROS 2 Humble уже установлен."
fi

# Клонирование репозиториев для ROS 2
echo "Клонирование репозиториев для ROS 2..."

clone_repo "https://github.com/PX4/px4_msgs.git" "$CLIENT_DIR/ros2/src"
clone_repo "https://github.com/PX4/px4_ros_com.git" "$CLIENT_DIR/ros2/src"
clone_repo "https://github.com/PakooGD/dsk_controller.git" "$CLIENT_DIR/ros2/src" "px4_controller"
# clone_repo "https://github.com/foxglove/ros-foxglove-bridge.git" "$CLIENT_DIR/ros2/src"

echo "Сборка пакетов ROS2..."
cd "$CLIENT_DIR/ros2" || handle_error "Не удалось перейти в $CLIENT_DIR/ros2"
source /opt/ros/humble/setup.bash || handle_error "Не удалось загрузить setup.bash для ROS2"
colcon build || handle_error "Не удалось выполнить colcon build"
source install/local_setup.bash || handle_error "Не удалось загрузить local_setup.bash"

cd "$PROJECT_ROOT"

######################################## PX4-Autopilot ########################################################

echo "Установка PX4-Autopilot..."

clone_repo "https://github.com/PakooGD/PX4-Autopilot.git" "$CLIENT_DIR"
cd "$CLIENT_DIR/PX4-Autopilot"

bash ./Tools/setup/ubuntu.sh

# Установка  GazeboHarmonic
if command -v gz &> /dev/null; then
    echo "Gazebo Harmonic уже установлен."
else
    echo "Установка Gazebo Harmonic..."

    sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    sudo apt-get update
    sudo apt-get install gz-harmonic
fi
echo "Сборка PX4-Autopilot..."
make px4_sitl

cd "$PROJECT_ROOT"

######################################## Micro XRCE-DDS Agent ########################################################

echo "Установка Micro XRCE-DDS Agent..."

clone_repo "https://github.com/eProsima/Micro-XRCE-DDS-Agent.git" "$CLIENT_DIR"

echo "Сборка Micro XRCE-DDS Agent..."

cd $CLIENT_DIR/Micro-XRCE-DDS-Agent/
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/

cd "$PROJECT_ROOT"

######################################## QGroundControl ########################################################

echo "Установка QGroundControl..."

clone_repo "https://github.com/PakooGD/qgroundcontrol.git" "$CLIENT_DIR"

cd "$CLIENT_DIR/qgroundcontrol"
sudo bash ./tools/setup/install-dependencies-debian.sh

# Сборка QGroundControl
echo "Сборка QGroundControl..."

mkdir -p build
~/Qt/6.8.2/gcc_64/bin/qt-cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug # Qt-6.8.2 is required! Instal it through Qt obline installer: https://www.qt.io/offline-installers 
cmake --build build --config Debug

cd "$PROJECT_ROOT"

######################################## FOXGLOVE ########################################################

echo "Установка Foxglove Studio..."

clone_repo "https://github.com/PakooGD/foxglove-opensource.git" "$CLIENT_DIR" "foxglove"
if [ -d "$CLIENT_DIR/foxglove/node_modules" ]; then
    echo "Зависимости Foxglove Studio уже установлены."
else
    echo "Установка зависимостей Foxglove Studio..."
    cd "$CLIENT_DIR/foxglove"
    yarn install
fi

echo "Сборка Foxglove Studio..."
yarn web:build:dev

cd "$PROJECT_ROOT"

######################################## SERVER ########################################################

echo "Установка зависимостей серверна..."

clone_repo "https://github.com/PakooGD/dsk_server.git" "$PROJECT_ROOT/src" "server"

# Установка серверных зависимостей
if [ -d "$SERVER_DIR/node_modules" ]; then
    echo "Зависимости сервера уже установлены."
else
    echo "Установка зависимостей сервера..."
    cd "$SERVER_DIR"
    npm install
fi

cd "$PROJECT_ROOT"

echo "Установка завершение"