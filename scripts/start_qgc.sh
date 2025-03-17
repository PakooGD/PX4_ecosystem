#!/bin/bash

# Переход в директорию QGroundControl
cd $PROJECT_ROOT/src/client/qgroundcontrol

# Сборка QGroundControl
echo "Building QGroundControl..."

mkdir -p build
~/Qt/6.8.2/gcc_64/bin/qt-cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug # Qt-6.8.2 is required! Instal it through Qt obline installer: https://www.qt.io/offline-installers 
cmake --build build --config Debug

# Запуск QGroundControl в фоновом режиме
echo "Starting QGroundControl..."
./build/Debug/QGroundControl
