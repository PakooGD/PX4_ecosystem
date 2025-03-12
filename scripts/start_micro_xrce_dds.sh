#!/bin/bash

# Установка глобальных переменных
PROJECT_ROOT=$(pwd)

cd $PROJECT_ROOT/src/client/micro_xrce_dds/build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
MicroXRCEAgent udp4 -p 8888