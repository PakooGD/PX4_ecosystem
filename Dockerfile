# Используем официальный образ Ubuntu 22.04
FROM ubuntu:22.04

# Устанавливаем переменные окружения
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Устанавливаем необходимые пакеты
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    git \
    build-essential \
    cmake \
    ninja-build \
    g++ \
    gdb \
    gawk \
    make \
    libtool \
    libtool-bin \
    zip \
    default-jre \
    socat \
    libxml2-dev \
    libxslt1-dev \
    libgl1-mesa-dev \
    libosmesa6-dev \
    libqt5gui5 \
    libxcb-cursor0 \
    libxkbcommon-dev \
    libxcb-xinerama0-dev \
    libsm6 \
    libxext6 \
    ffmpeg \
    python3 \
    python3-pip \
    python3-venv \
    python3-numpy \
    python3-pyparsing \
    python3-serial \
    python-is-python3 \
    locales \
    lsb-release \
    gnupg \
    gnupg2 \
    software-properties-common \
    libfuse2 \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    && rm -rf /var/lib/apt/lists/*

# Устанавливаем ROS 2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-dev-tools \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Устанавливаем Node.js через nvm
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash \
    && . "$HOME/.nvm/nvm.sh" \
    && nvm install 22 \
    && corepack enable

# Устанавливаем yq
RUN snap install yq

# Устанавливаем Rust и Cargo
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y

# Устанавливаем empy
RUN pip install --user -U empy==3.3.4 pyros-genmsg setuptools

# Устанавливаем Gazebo Harmonic
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update && apt-get install -y gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# Устанавливаем PX4-Autopilot
RUN git clone https://github.com/PakooGD/PX4-Autopilot.git /px4-autopilot \
    && cd /px4-autopilot \
    && bash ./Tools/setup/ubuntu.sh

# Устанавливаем Micro XRCE-DDS Agent
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /micro-xrce-dds-agent \
    && cd /micro-xrce-dds-agent \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install \
    && ldconfig /usr/local/lib/

# Устанавливаем QGroundControl
RUN git clone https://github.com/PakooGD/qgroundcontrol.git /qgroundcontrol \
    && cd /qgroundcontrol \
    && bash ./tools/setup/install-dependencies-debian.sh \
    && mkdir build \
    && cd build \
    && ~/Qt/6.8.2/gcc_64/bin/qt-cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug \
    && cmake --build build --config Debug

# Устанавливаем Foxglove Studio
RUN git clone https://github.com/PakooGD/foxglove-opensource.git /foxglove \
    && cd /foxglove \
    && yarn install \
    && yarn web:build:dev

# Устанавливаем Rerun
RUN git clone https://github.com/PakooGD/rerun.git /rerun \
    && cd /rerun \
    && cargo install --path . \
    && pip install rerun-sdk

# Устанавливаем рабочий каталог
WORKDIR /workspace

# Копируем скрипты и конфигурации
COPY scripts /workspace/scripts
COPY config /workspace/config

# Устанавливаем права на выполнение скриптов
RUN chmod +x /workspace/scripts/*.sh

# Устанавливаем переменные окружения
ENV PROJECT_ROOT=/workspace
ENV CLIENT_DIR=/workspace/src/client
ENV SERVER_DIR=/workspace/src/server

# Создаем директории
RUN mkdir -p /workspace/src/client /workspace/src/server

# Устанавливаем локаль
RUN locale-gen en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Запускаем bash по умолчанию
CMD ["bash"]