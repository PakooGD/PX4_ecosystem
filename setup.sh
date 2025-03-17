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
        sudo apt-get install --no-install-recommends -y $package_name 
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

######################################## Установка зависимостей ########################################################

echo "Установка зависимостей и базовая настройка..."

# Обновление списка пакетов
sudo apt-get update

# Установка Python 3 и необходимых зависимостей
install_apt_package "python3"              # Основной интерпретатор Python 3
install_apt_package "python3-pip"          # Менеджер пакетов для Python
install_apt_package "python3-venv"         # Создание виртуальных окружений Python
install_apt_package "python3-numpy"        # Библиотека для работы с массивами и математикой
install_apt_package "python3-pyparsing"    # Библиотека для парсинга текста
install_apt_package "python3-serial"       # Библиотека для работы с последовательными портами
install_apt_package "python-is-python3"    # Символическая ссылка python -> python3

# Установка базовых инструментов для сборки
install_apt_package "build-essential"      # Основные инструменты для компиляции (gcc, make и др.)
install_apt_package "ccache"               # Кэш для ускорения повторной компиляции
install_apt_package "g++"                  # Компилятор C++
install_apt_package "gdb"                  # Отладчик GNU
install_apt_package "gawk"                 # Утилита для обработки текста
install_apt_package "make"                 # Утилита для управления сборкой
install_apt_package "cmake"                # Система управления сборкой (уже установлена выше)
install_apt_package "ninja-build"          # Альтернативная система сборки
install_apt_package "libtool"              # Утилита для управления библиотеками
install_apt_package "libxml2-dev"          # Библиотека для работы с XML
install_apt_package "libxslt1-dev"         # Библиотека для работы с XSLT
install_apt_package "libtool-bin"          # Утилиты для работы с libtool
install_apt_package "zip"                  # Утилита для работы с ZIP-архивами
install_apt_package "default-jre"          # Java Runtime Environment (для некоторых инструментов)
install_apt_package "socat"                # Утилита для работы с сокетами

# Установка графических зависимостей
install_apt_package "libgl1-mesa-dev"      # Библиотеки OpenGL
install_apt_package "libosmesa6-dev"       # Библиотеки для программной реализации OpenGL
install_apt_package "libqt5gui5"           # Графическая библиотека Qt5
install_apt_package "libxcb-cursor0"       # Библиотека для работы с курсором в X11
install_apt_package "libxkbcommon-dev"     # Библиотека для работы с клавиатурой в X11
install_apt_package "libxcb-xinerama0-dev" # Библиотека для работы с многомониторными системами в X11

# Установка мультимедийных зависимостей
install_apt_package "ffmpeg"               # Утилиты для работы с видео и аудио
install_apt_package "libsm6"               # Библиотека для работы с X11 Session Management
install_apt_package "libxext6"             # Библиотека для расширений X11

# Установка ROS 2 зависимостей
install_apt_package "ros-humble-desktop"   # ROS 2 Humble Desktop (полный набор инструментов)
install_apt_package "ros-dev-tools"        # Инструменты для разработки под ROS
install_apt_package "ros-humble-ros-gz"    # Интеграция ROS с Gazebo

# Установка colcon (инструмент для сборки ROS 2)
install_apt_package "python3-colcon-common-extensions"  # Расширения для colcon

# Установка дополнительных утилит
install_apt_package "curl"                 # Утилита для работы с HTTP-запросами
install_apt_package "libssl-dev"           # Утилита для работы с SSL и TLS 
install_apt_package "locales"              # Поддержка локалей
install_apt_package "lsb-release"          # Утилита для получения информации о дистрибутиве
install_apt_package "gnupg"                # Утилита для работы с GPG
install_apt_package "gnupg2"                # Утилита для работы с GPG2
install_apt_package "software-properties-common"  # Утилита для управления репозиториями
install_apt_package "wget"                 # Утилита для загрузки файлов
install_apt_package "libfuse2"              #  УТилита для работы с файловым пространством

# Установка зависимостей для работы с GStreamer
install_apt_package "gstreamer1.0-plugins-bad"  # Плагины GStreamer (нестабильные)
install_apt_package "gstreamer1.0-libav"        # Плагины GStreamer для работы с libav
install_apt_package "gstreamer1.0-gl"           # Плагины GStreamer для работы с OpenGL

# Установка CMake версии 3.21+
install_package "cmake" "wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null && sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main' && sudo apt-get update && sudo apt-get install -y cmake"

# Установка Node.js через nvm
install_package "node" "curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash && \. \"$HOME/.nvm/nvm.sh\" && nvm install 22 && corepack enable"

# Установка yq (утилита для работы с YAML)
install_package "yq" "sudo snap install yq"

# Установка empy
install_package "pip" "pip install --user -U empy==3.3.4 pyros-genmsg setuptools"


# Удаление modemmanager (может конфликтовать с ROS)
if dpkg -l | grep -q "^ii  modemmanager "; then
    sudo apt-get remove -y modemmanager
else
    echo "modemmanager уже удалён."
fi

######################################## Установка Qt 6.8.2 ########################################################


# echo "Проверка и установка Qt 6.8.2..."

# # Целевая версия Qt
# TARGET_QT_VERSION="6.8.2"

# # Проверяем, установлен ли Qt и его версию
# if command -v qmake &> /dev/null; then
#     INSTALLED_QT_VERSION=$(qmake --version | grep -oP "Qt version \K[0-9]+\.[0-9]+\.[0-9]+")
#     if [ "$INSTALLED_QT_VERSION" == "$TARGET_QT_VERSION" ]; then
#         echo "Qt $TARGET_QT_VERSION уже установлен."
#     else
#         echo "Установлена другая версия Qt ($INSTALLED_QT_VERSION). Требуется установка Qt $TARGET_QT_VERSION."
#     fi
# else
#     echo "Qt не установлен. Начинаем установку Qt $TARGET_QT_VERSION..."
# fi

# # Если Qt не установлен или установлена неправильная версия
# if ! command -v qmake &> /dev/null || [ "$INSTALLED_QT_VERSION" != "$TARGET_QT_VERSION" ]; then
#     # libxcb-cursor0 требуется для Ubuntu

#     # Создаем директорию для Qt
#     QT_INSTALL_DIR="$HOME/Qt"
#     mkdir -p "$QT_INSTALL_DIR"
#     cd "$QT_INSTALL_DIR" || { echo "Ошибка: не удалось перейти в $QT_INSTALL_DIR"; exit 1; }

#     # Скачиваем установщик Qt
#     QT_INSTALLER_URL="https://d13lb3tujbc8s0.cloudfront.net/onlineinstallers/qt-online-installer-linux-x64-4.8.1.run"
#     QT_INSTALLER_FILE=$(basename "$QT_INSTALLER_URL")

#     if [ ! -f "$QT_INSTALLER_FILE" ]; then
#         echo "Скачивание установщика Qt..."
#         wget "$QT_INSTALLER_URL" -O "$QT_INSTALLER_FILE"
#         chmod +x "$QT_INSTALLER_FILE"
#     else
#         echo "Установщик Qt уже скачан."
#     fi

#     # Запуск установщика Qt в автоматическом режиме
#     echo "Запуск установщика Qt..."
#     ./"$QT_INSTALLER_FILE" --platform minimal --script "$PROJECT_ROOT/scripts/qt-installer-script.js" --verbose &

#     # Ожидание завершения установки
#     INSTALLER_PID=$!
#     wait $INSTALLER_PID

#     # Проверяем, успешно ли установлен Qt
#     if [ -f "$QT_INSTALL_DIR/$TARGET_QT_VERSION/gcc_64/bin/qmake" ]; then
#         echo "Qt $TARGET_QT_VERSION успешно установлен."
#     else
#         echo "Ошибка: Qt $TARGET_QT_VERSION не был установлен."
#         exit 1
#     fi
# fi

# # Добавляем Qt в PATH, если он еще не добавлен
# QT_BIN_DIR="$QT_INSTALL_DIR/$TARGET_QT_VERSION/gcc_64/bin"
# if ! grep -q "$QT_BIN_DIR" ~/.bashrc; then
#     echo "Добавление Qt в PATH..."
#     echo "export PATH=$QT_BIN_DIR:\$PATH" >> ~/.bashrc
#     source ~/.bashrc
# else
#     echo "Qt уже добавлен в PATH."
# fi

######################################## Очистка системы ########################################################

echo "Очистка системы..."

sudo apt-get clean
sudo apt-get autoremove -y
sudo apt-get autoclean
sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

echo "Все зависимости установлены и система очищена."

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