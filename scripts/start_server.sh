#!/bin/bash

# Чтение порта сервера из launch_params.yml
PORT=$(yq e '.server.port' $PROJECT_ROOT/config/launch_params.yml)

cd $PROJECT_ROOT/src/server
# Запуск сервера
yarn start --port $PORT