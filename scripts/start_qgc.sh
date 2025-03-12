#!/bin/bash

PROJECT_ROOT=$(pwd)

cd $PROJECT_ROOT/src/client/qgroundcontrol
mkdir -p build
~/Qt/6.8.2/gcc_64/bin/qt-cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug # Qt-6.8.2 is required! Instal it through Qt obline installer: https://www.qt.io/offline-installers 
cmake --build build --config Debug
./build/Debug/QGroundControl
