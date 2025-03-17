#!/bin/bash
PROJECT_ROOT=$(pwd)
SERVER_URL="http://127.0.0.1:3000/mavlog"  # Замените 127.0.0.1 на IP-адрес сервера
MAVLINK_PORT="udp:127.0.0.1:14550"

# Переход в директорию с mavlink_ulog_streaming.py
cd $PROJECT_ROOT/src/  # Убедитесь, что путь правильный

# Запуск mavlink_ulog_streaming.py
echo "Starting MAVLink log streaming..."
python3 mavlink_ulog_streaming.py --port=$MAVLINK_PORT --server-url=$SERVER_URL

