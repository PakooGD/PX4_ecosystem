# Используем базовый образ Ubuntu 22.04
FROM ubuntu:22.04

# Устанавливаем переменные окружения
ENV DEBIAN_FRONTEND=noninteractive
ENV PROJECT_ROOT=/workspace
ENV CLIENT_DIR=$PROJECT_ROOT/src/client

# Устанавливаем необходимые пакеты
RUN apt-get update && apt-get install -y \
    curl \
    git \
    build-essential \
    locales \
    software-properties-common \
    lsb-release \
    gnupg \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libqt5gui5 \
    libfuse2 \
    wget \
    libxcb-cursor0 \
    libxkbcommon-dev \
    libxcb-xinerama0-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Устанавливаем Node.js через nvm
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash \
    && . "$HOME/.nvm/nvm.sh" \
    && nvm install 22 \
    && corepack enable

# Устанавливаем ROS 2 Humble
RUN locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && apt-get update && apt-get install -y \
    locales \
    software-properties-common \
    curl \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-dev-tools

# Устанавливаем Foxglove Bridge
RUN apt-get update && apt-get install -y ros-humble-foxglove-bridge

# Рабочая директория
WORKDIR $PROJECT_ROOT

# Команда по умолчанию
CMD ["bash"]