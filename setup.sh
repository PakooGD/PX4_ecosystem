#!/bin/bash

######################################## Helpers ########################################################

# Функция вывода процесса установки
echo_setup_step() {
    local is_header="${1:-false}"
    local message=$2
    local color="${3:-}"
 
    if [[ "$1" != "true" && "$1" != "false" ]]; then
        local is_header="false"  # По умолчанию is_header=false
        local message="$1"       # Первый параметр — это сообщение
        local color="${2:-}"     # Второй параметр — это цвет (опционально)
    else
        local is_header="$1"     # Первый параметр — это is_header
        local message="$2"       # Второй параметр — это сообщение
        local color="${3:-}"     # Третий параметр — это цвет (опционально)
    fi

    if [ "$is_header" = false ]; then
        local default_color="\\033[37m"
        echo -e "----- ${color:-$default_color}$message\\033[0m ------------------------------------"
        echo " "
    else
        local default_color="\\033[33m"
        echo " "
        echo -e "${color:-$default_color}#################### $message ######################\\033[0m"
        echo " "
    fi
}

create_directory() {
    local dir_path=$1
    if [ -d "$dir_path" ]; then
        echo_setup_step "Директория $dir_path уже существует." "\\033[31m"
    else
        mkdir -p "$dir_path"
        echo_setup_step "Директория $dir_path создана..."
    fi
}

# Функция для проверки и установки пакетов
install_package() {
    local package_name=$1
    local install_command=$2

    if ! command -v "$package_name" &> /dev/null; then
        echo_setup_step "Установка $package_name..."
        eval "$install_command"
    else
        echo_setup_step "$package_name уже установлен." "\\033[32m" 
    fi
}

# Функция для установки пакетов через apt с проверкой
install_apt_package() {
    local package_name=$1

    if ! dpkg -l "$package_name" | grep -q "^ii"; then
        sudo apt-get install --no-install-recommends -y $package_name 
    else
        echo_setup_step "$package_name уже установлен." "\\033[32m" 
    fi
}

# Функция для клонирования или обновления репозитория
clone_repo() {
    local repo_url=$1          # URL репозитория
    local target_dir=$2        # Целевая директория для клонирования
    local repo_name=${3:-$(basename "$repo_url" .git)}  # Имя папки (по умолчанию извлекается из URL)
    local update_submodules=${4:-false}  # Обновлять ли подмодули (по умолчанию false)

    # Создаем целевую папку, если её нет
    mkdir -p "$target_dir"
    cd "$target_dir" || { echo -e "\u001b[30mОшибка: не удалось перейти в $target_dir\u001b[0m"; return 1; }

    if [ -d "$repo_name" ] && [ -d "$repo_name/.git" ]; then
        echo -e "\u001b[35m$repo_name уже клонирован и является Git-репозиторием.\u001b[0m"
        cd "$repo_name"

        # Определяем ветку по умолчанию
        local default_branch=$(git remote show origin | grep "HEAD branch" | awk '{print $3}')
        if [ -z "$default_branch" ]; then
            default_branch="main"  # Если не удалось определить, используем "main"
        fi

        # Сохраняем текущий хэш коммита
        local current_commit=$(git rev-parse HEAD)

        # Фетчим изменения
        echo -e "\u001b[35mОбновление $repo_name (ветка $default_branch)...\u001b[0m"
        git fetch --all --prune  # Обновляем информацию о всех ветках и удаляем устаревшие

        # Сбрасываем изменения и переходим на последний коммит
        git reset --hard "origin/$default_branch"

        # Получаем новый хэш коммита
        local new_commit=$(git rev-parse HEAD)

        # Проверяем, были ли изменения
        if [ "$current_commit" != "$new_commit" ]; then
            echo -e "\u001b[32mОбнаружены изменения в репозитории $repo_name.\u001b[0m"
            # Обновляем подмодули, если нужно
            if [ "$update_submodules" = true ]; then
                echo -e "\u001b[35mОбновление подмодулей...\u001b[0m"
                git submodule update --init --recursive
            fi

            # Возвращаем true, чтобы указать, что были изменения
            return 0
        else
            echo -e "\u001b[36mИзменений в репозитории $repo_name не обнаружено.\u001b[0m"
            # Возвращаем false, чтобы указать, что изменений не было
            return 1
        fi

        cd ..
    else
        echo -e "\u001b[35mКлонирование $repo_name...\u001b[0m"
        git clone "$repo_url" "$repo_name" --recursive -j8 || { echo -e "\u001b[30mОшибка: не удалось клонировать $repo_name\u001b[0m"; return 1; }
        # Возвращаем true, так как репозиторий был клонирован впервые
        return 0
    fi
}

######################################## Проверка свободного места ########################################################

MIN_DISK_SPACE=10485760  # 10 ГБ в килобайтах
if [ $(df / | awk 'NR==2 {print $4}') -lt $MIN_DISK_SPACE ]; then
    echo_setup_step "Ошибка: недостаточно свободного места на диске." "\\033[31m"
    exit 1
fi

######################################## Настройка переменных окружения ########################################################

echo " "
echo_setup_step true "Настройка переменных окружения" 

PROJECT_ROOT=$(pwd)

LOG_FILE="$PROJECT_ROOT/install.log"
: > "$LOG_FILE" # Очищаем лог-файл перед началом записи
exec > >(tee -a "$LOG_FILE") 2>&1 # Перенаправляем вывод в лог-файл и на экран

echo_setup_step "Начало установки: $(date)"

# Делаем все скрипты в директории scripts исполняемыми
echo_setup_step "Делаем все скрипты в директории scripts исполняемыми..."
chmod +x "$PROJECT_ROOT/scripts"/*.sh

# Добавление пользователя в группу dialout
echo_setup_step "Добавление пользователя $USER в группу dialout..."
if ! groups $USER | grep -q "\bdialout\b"; then
    sudo usermod -a -G dialout $USER
else
    echo_setup_step "Пользователь $USER уже в группе dialout." "\\033[32m" 
fi

# Добавляем переменные окружения в .bashrc
echo_setup_step "Добавляем переменные окружения в .bashrc, если отсутствуют..."
if ! grep -q "source $PROJECT_ROOT/config/env.sh" ~/.bashrc; then
    echo_setup_step "source $PROJECT_ROOT/config/env.sh" >> ~/.bashrc
fi
if ! grep -q "source $PROJECT_ROOT/config/aliases.sh" ~/.bashrc; then
    echo_setup_step "source $PROJECT_ROOT/config/aliases.sh" >> ~/.bashrc
fi
source ~/.bashrc

echo "Корневая директория: $PROJECT_ROOT"
echo "Клиентская директория: $CLIENT_DIR"
echo "Серверная директория: $SERVER_DIR"

######################################## Установка зависимостей ########################################################

echo_setup_step true "Обновление системы и пакетов..." 

# Обновление списка пакетов
sudo apt-get update
sudo apt-get upgrade -y

echo_setup_step true "Установка дополнительных пакетов"

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
install_apt_package "libtool-bin"          # Утилиты для работы с libtool
install_apt_package "zip"                  # Утилита для работы с ZIP-архивами
install_apt_package "default-jre"          # Java Runtime Environment (для некоторых инструментов)
install_apt_package "socat"                # Утилита для работы с сокетами
install_apt_package "libxml2-dev"          # Библиотека для работы с XML
install_apt_package "libxslt1-dev"         # Библиотека для работы с XSLT

# Установка графических зависимостей
install_apt_package "libgl1-mesa-dev"      # Библиотеки OpenGL
install_apt_package "libosmesa6-dev"       # Библиотеки для программной реализации OpenGL
install_apt_package "libqt5gui5"           # Графическая библиотека Qt5
install_apt_package "libxcb-cursor0"       # Библиотека для работы с курсором в X11
install_apt_package "libxkbcommon-dev"     # Библиотека для работы с клавиатурой в X11
install_apt_package "libxcb-xinerama0-dev" # Библиотека для работы с многомониторными системами в X11
install_apt_package "libsm6"               # Библиотека для работы с X11 Session Management
install_apt_package "libxext6"             # Библиотека для расширений X11

# Установка мультимедийных зависимостей
install_apt_package "ffmpeg"               # Утилиты для работы с видео и аудио

# Установка ROS 2 зависимостей
install_apt_package "ros-humble-desktop"   # ROS 2 Humble Desktop (полный набор инструментов)
install_apt_package "ros-dev-tools"        # Инструменты для разработки под ROS

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

# Установка pycryptodome - для работы с шифрованием
install_package "pip" "pip install pycryptodome"


# Установка Rust и Cargo
install_package "rustup" "curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y && source $HOME/.cargo/env"

######################################## Удаление ########################################################

echo_setup_step true "Удаление потенциально конфликтующих программ и пакетов"

# Удаление modemmanager (может конфликтовать с ROS)
if dpkg -l | grep -q "^ii  modemmanager "; then
    sudo apt-get remove -y modemmanager
else
    echo_setup_step "modemmanager уже удалён." "\\033[32m" 
fi

######################################## Установка Qt 6.8.2 ########################################################


# echo_setup_step true "Проверка и установка Qt 6.8.2..."

# # Целевая версия Qt
# TARGET_QT_VERSION="6.8.2"

# # Проверяем, установлен ли Qt и его версию
# if command -v qmake &> /dev/null; then
#     INSTALLED_QT_VERSION=$(qmake --version | grep -oP "Qt version \K[0-9]+\.[0-9]+\.[0-9]+")
#     if [ "$INSTALLED_QT_VERSION" == "$TARGET_QT_VERSION" ]; then
#         echo_setup_step "Qt $TARGET_QT_VERSION уже установлен."
#     else
#         echo_setup_step "Установлена другая версия Qt ($INSTALLED_QT_VERSION). Требуется установка Qt $TARGET_QT_VERSION."
#     fi
# else
#     echo_setup_step "Qt не установлен. Начинаем установку Qt $TARGET_QT_VERSION..."
# fi

# # Если Qt не установлен или установлена неправильная версия
# if ! command -v qmake &> /dev/null || [ "$INSTALLED_QT_VERSION" != "$TARGET_QT_VERSION" ]; then
#     # libxcb-cursor0 требуется для Ubuntu

#     # Создаем директорию для Qt
#     QT_INSTALL_DIR="$HOME/Qt"
#     mkdir -p "$QT_INSTALL_DIR"
#     cd "$QT_INSTALL_DIR" || { echo_setup_step "Ошибка: не удалось перейти в $QT_INSTALL_DIR"; exit 1; }

#     # Скачиваем установщик Qt
#     QT_INSTALLER_URL="https://d13lb3tujbc8s0.cloudfront.net/onlineinstallers/qt-online-installer-linux-x64-4.8.1.run"
#     QT_INSTALLER_FILE=$(basename "$QT_INSTALLER_URL")

#     if [ ! -f "$QT_INSTALLER_FILE" ]; then
#         echo_setup_step "Скачивание установщика Qt..."
#         wget "$QT_INSTALLER_URL" -O "$QT_INSTALLER_FILE"
#         chmod +x "$QT_INSTALLER_FILE"
#     else
#         echo_setup_step "Установщик Qt уже скачан."
#     fi

#     # Запуск установщика Qt в автоматическом режиме
#     echo_setup_step "Запуск установщика Qt..."
#     ./"$QT_INSTALLER_FILE" --platform minimal --script "$PROJECT_ROOT/scripts/qt-installer-script.js" --verbose &

#     # Ожидание завершения установки
#     INSTALLER_PID=$!
#     wait $INSTALLER_PID

#     # Проверяем, успешно ли установлен Qt
#     if [ -f "$QT_INSTALL_DIR/$TARGET_QT_VERSION/gcc_64/bin/qmake" ]; then
#         echo_setup_step "Qt $TARGET_QT_VERSION успешно установлен."
#     else
#         echo_setup_step "Ошибка: Qt $TARGET_QT_VERSION не был установлен."
#         exit 1
#     fi
# fi

# # Добавляем Qt в PATH, если он еще не добавлен
# QT_BIN_DIR="$QT_INSTALL_DIR/$TARGET_QT_VERSION/gcc_64/bin"
# if ! grep -q "$QT_BIN_DIR" ~/.bashrc; then
#     echo_setup_step "Добавление Qt в PATH..."
#     echo_setup_step "export PATH=$QT_BIN_DIR:\$PATH" >> ~/.bashrc
#     source ~/.bashrc
# else
#     echo_setup_step "Qt уже добавлен в PATH."
# fi

######################################## Создаем локали для английского языка ########################################################

echo_setup_step true "Настраиваем локали для английского языка с кодировкой UTF-8..." 

# Проверка и генерация локалей, если они ещё не сгенерированы
if ! locale -a | grep -q "en_US.utf8"; then
    echo_setup_step "Локаль en_US.UTF-8 не найдена. Генерация локалей..." 
    sudo locale-gen en_US en_US.UTF-8
else
    echo_setup_step "Локаль en_US.UTF-8 уже сгенерирована." "\\033[32m" 
fi

# Проверка и обновление глобальных настроек локали, если они ещё не настроены
if ! grep -q "LANG=en_US.UTF-8" /etc/default/locale || ! grep -q "LC_ALL=en_US.UTF-8" /etc/default/locale; then
    echo_setup_step "Глобальные настройки локали не найдены. Настройка локали..."
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
else
    echo_setup_step "Глобальные настройки локали уже настроены." "\\033[32m" 
fi

# Проверка и установка локали для текущей сессии, если она ещё не установлена
if [ "$LANG" != "en_US.UTF-8" ]; then
    echo_setup_step "Локаль для текущей сессии не настроена. Установка локали..."
    export LANG=en_US.UTF-8
else
    echo_setup_step "Локаль для текущей сессии уже настроена." "\\033[32m" 
fi

######################################## Создаем директорию ########################################################

echo_setup_step true "Создаем директорию..." 

create_directory "$PROJECT_ROOT/src/client"
create_directory "$PROJECT_ROOT/src/server"

######################################## SERVER ########################################################

echo_setup_step true "Установка зависимостей сервера..." 

clone_repo "https://github.com/PakooGD/dsk_server.git" "$PROJECT_ROOT/src" "server"
cd "$SERVER_DIR"
# Установка серверных зависимостей
if [ -d "$SERVER_DIR/node_modules" ]; then
    echo " "
    echo_setup_step "Зависимости сервера уже установлены."
else
    echo_setup_step "Установка зависимостей сервера..."
    npm install
fi

cd "$PROJECT_ROOT"

######################################## ROS2 Humble ########################################################

# Установка ROS 2 Humble

echo_setup_step true "Установка ROS 2 Humble..." 

if [ ! -f /opt/ros/humble/setup.bash ]; then
    sudo add-apt-repository universe
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo_setup_step $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source /opt/ros/humble/setup.bash
else
    echo_setup_step "ROS 2 Humble уже установлен."
fi

# Клонирование репозиториев для ROS 2
echo_setup_step "Клонирование репозиториев для ROS 2..."  

clone_repo "https://github.com/PX4/px4_msgs.git" "$CLIENT_DIR/ros2/src"
clone_repo "https://github.com/PX4/px4_ros_com.git" "$CLIENT_DIR/ros2/src"
clone_repo "https://github.com/PakooGD/dsk_controller.git" "$CLIENT_DIR/ros2/src" "px4_controller"
# clone_repo "https://github.com/foxglove/ros-foxglove-bridge.git" "$CLIENT_DIR/ros2/src"

# Проверяем, были ли изменения в любом из репозиториев
if [ $? -eq 0 ]; then
    echo " "
    echo_setup_step "Сборка пакетов ROS2..."
    cd "$CLIENT_DIR/ros2" 
    source /opt/ros/humble/setup.bash 
    colcon build
    source install/local_setup.bash 
fi

cd "$PROJECT_ROOT"

######################################## PX4-Autopilot ########################################################

echo_setup_step true "Установка PX4-Autopilot..." 

clone_repo "https://github.com/PakooGD/PX4-Autopilot.git" "$CLIENT_DIR"

if [ $? -eq 0 ]; then
    cd "$CLIENT_DIR/PX4-Autopilot"
    if command -v gz &> /dev/null; then
        echo " "
        echo_setup_step "Gazebo Harmonic уже установлен."
    else
        echo " "
        echo_setup_step "Установка Gazebo Harmonic..."

        sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
        sudo apt-get update
        sudo apt-get install gz-harmonic
    fi
    bash ./Tools/setup/ubuntu.sh
    echo_setup_step "Сборка PX4-Autopilot..."
    make px4_sitl
fi

cd "$PROJECT_ROOT"

######################################## Micro XRCE-DDS Agent ########################################################

echo_setup_step true "Установка Micro XRCE-DDS Agent..." 

clone_repo "https://github.com/eProsima/Micro-XRCE-DDS-Agent.git" "$CLIENT_DIR"

if [ $? -eq 0 ]; then
    cd $CLIENT_DIR/Micro-XRCE-DDS-Agent/
    echo " "
    echo_setup_step "Сборка Micro XRCE-DDS Agent..."
    create_directory "$CLIENT_DIR/Micro-XRCE-DDS-Agent/build"
    cd build 
    cmake ..
    make
    sudo make install
    sudo ldconfig /usr/local/lib/
fi

cd "$PROJECT_ROOT"

######################################## QGroundControl ########################################################

echo_setup_step true "Установка QGroundControl..." 

clone_repo "https://github.com/PakooGD/qgroundcontrol.git" "$CLIENT_DIR"

if [ $? -eq 0 ]; then
    cd "$CLIENT_DIR/qgroundcontrol"
    sudo bash ./tools/setup/install-dependencies-debian.sh
        # Сборка QGroundControl
    echo " "
    echo_setup_step "Сборка QGroundControl..."
    create_directory "$CLIENT_DIR/qgroundcontrol/build"
    cd build
    ~/Qt/6.8.2/gcc_64/bin/qt-cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug # Qt-6.8.2 is required! Instal it through Qt obline installer: https://www.qt.io/offline-installers 
    cmake --build build --config Debug
fi

cd "$PROJECT_ROOT"

######################################## FOXGLOVE ########################################################

echo_setup_step true "Установка Foxglove Studio..."

clone_repo "https://github.com/PakooGD/foxglove-opensource.git" "$CLIENT_DIR" "foxglove"
if [ $? -eq 0 ]; then
    cd "$CLIENT_DIR/foxglove"
    if [ -d "$CLIENT_DIR/foxglove/node_modules" ]; then
        echo " "
        echo_setup_step "Зависимости Foxglove Studio уже установлены."
    else
        echo " "
        echo_setup_step "Установка зависимостей Foxglove Studio..."
        yarn install
    fi
    echo_setup_step "Сборка Foxglove Studio..."
    yarn web:build:dev
fi

cd "$PROJECT_ROOT"

######################################## Rerun ########################################################

echo_setup_step true "Установка Rerun..." 

clone_repo "https://github.com/PakooGD/rerun.git" "$CLIENT_DIR" "rerun"

if [ $? -eq 0 ]; then
    if command -v rerun &> /dev/null; then
        echo " "
        echo_setup_step "Rerun уже установлен."
    else
        echo " "
        echo_setup_step "Установка Rerun в систему..."
        cargo install --path .
        pip install rerun-sdk
    fi
    cd "$CLIENT_DIR/rerun"
    echo_setup_step "Сборка Rerun..."
    cargo build --release
fi

cd "$PROJECT_ROOT"

######################################## Очистка системы ########################################################

echo_setup_step true "Очистка системы..." 

sudo apt-get clean
sudo apt-get autoremove -y
sudo apt-get autoclean
sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
ccache -C

echo_setup_step true "Установка завершение" "\\033[32m" 