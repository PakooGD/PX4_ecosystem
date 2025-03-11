# Используем базовый образ Ubuntu 22.04
FROM ubuntu:22.04

# Установка зависимостей
RUN apt update && apt install -y \
    git cmake build-essential python3-pip \
    curl lsb-release gnupg locales \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    libqt5gui5 libfuse2 wget libxcb-cursor0

# Настройка локали
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Установка ROS 2 Humble
RUN apt install -y software-properties-common && \
    add-apt-repository universe && \
    apt update && apt install -y curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && apt install -y ros-humble-desktop ros-dev-tools

# Установка зависимостей для PX4
RUN apt update && apt install -y \
    python3-pip python3-dev python3-numpy \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libeigen3-dev libopencv-dev

# Копирование исходников PX4 из папки client
COPY client/px4 /px4
WORKDIR /px4
RUN bash ./Tools/setup/ubuntu.sh && \
    make px4_sitl

# Установка Micro XRCE-DDS Agent
COPY client/micro_xrce_dds /micro_xrce_dds
WORKDIR /micro_xrce_dds/build
RUN cmake .. && make && make install && ldconfig /usr/local/lib/

# Установка Node.js
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash && \
    export NVM_DIR="$([ -z "${XDG_CONFIG_HOME-}" ] && printf %s "${HOME}/.nvm" || printf %s "${XDG_CONFIG_HOME}/nvm")" && \
    [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh" && \
    nvm install 22 && corepack enable

# Установка QGroundControl
COPY client/qgroundcontrol /qgroundcontrol
WORKDIR /qgroundcontrol
RUN bash ./tools/setup/install-dependencies-debian.sh && \
    ~/Qt/6.8.2/gcc_64/bin/qt-cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug

# Установка Foxglove Studio
COPY client/foxglove /foxglove
WORKDIR /foxglove
RUN yarn install && yarn build:packages

# Установка Foxglove Bridge
RUN apt install -y ros-humble-foxglove-bridge

# Копирование скриптов и конфигураций
COPY scripts /scripts
COPY config /config
COPY client/server /server

# Настройка переменных окружения
RUN echo "source /config/env.sh" >> ~/.bashrc

# Рабочая директория
WORKDIR /project

# Команда по умолчанию
CMD ["bash"]