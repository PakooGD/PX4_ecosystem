#!/bin/bash

# Определяем PROJECT_ROOT, если он не задан
if [ -z "$PROJECT_ROOT" ]; then
    export PROJECT_ROOT=$(dirname "$(realpath "$0")")
    echo "PROJECT_ROOT установлен в: $PROJECT_ROOT"
fi

# Запрос пароля
echo "Введите пароль для выполнения команд с sudo:"
read -s password

# Сохраняем пароль во временный файл
PASSWORD_FILE=$(mktemp)
echo "$password" > "$PASSWORD_FILE"

# Массив для хранения PID запущенных процессов
declare -a PIDS

# Функция для запуска команды в фоновом режиме
run_in_background() {
    local command="$1"
    $command &
    PIDS+=($!)  # Сохраняем PID процесса
}

# Функция для запуска команды с sudo в фоновом режиме
run_with_sudo() {
    local command="$1"
    local password=$(cat "$PASSWORD_FILE")
    echo "$password" | sudo -S $command &
    PIDS+=($!)  # Сохраняем PID процесса
}

# Функция для запуска команды в новой вкладке терминала
run_in_new_tab() {
    local command="$1"
    local password=$(cat "$PASSWORD_FILE")

    if command -v gnome-terminal &> /dev/null; then
        if [[ "$command" == *"sudo"* ]]; then
            gnome-terminal --tab -- bash -c "echo '$password' | sudo -S $command; exec bash" &
        else
            gnome-terminal --tab -- bash -c "$command; exec bash" &
        fi 
        PIDS+=($!)  # Сохраняем PID процесса
    else
        echo "gnome-terminal не найден. Используем фоновый режим."
        run_in_background "$command"
    fi
}

# Функция выбора способа запуска
run_command() {
    local command="$1"
    local use_new_tab="${2:-false}"  # По умолчанию запуск в текущем терминале

    if [ "$use_new_tab" = true ]; then
        run_in_new_tab "$command"
    else
        run_in_background "$command"
    fi
}

# Функция для завершения всех процессов
cleanup() {
    echo "Завершение всех процессов..."

    # Завершение всех процессов из массива PIDS
    for pid in "${PIDS[@]}"; do
        if ps -p $pid > /dev/null; then
            echo "Завершение процесса с PID $pid..."
            kill -SIGINT $pid  # Сначала отправляем SIGINT
            sleep 1  # Даем процессу время на завершение
            if ps -p $pid > /dev/null; then
                echo "Принудительное завершение процесса с PID $pid..."
                kill -SIGKILL $pid  # Принудительно завершаем, если процесс не завершился
            fi
        fi
    done

    # Завершение Gazebo, если он запущен
    if pgrep gazebo > /dev/null; then
        echo "Завершение Gazebo..."
        pkill -SIGINT gazebo
        sleep 1
        if pgrep gazebo > /dev/null; then
            echo "Принудительное завершение Gazebo..."
            pkill -SIGKILL gazebo
        fi
    fi

    # Завершение QGroundControl, если он запущен
    if pgrep QGroundControl > /dev/null; then
        echo "Завершение QGroundControl..."
        pkill -SIGINT QGroundControl
        sleep 1
        if pgrep QGroundControl > /dev/null; then
            echo "Принудительное завершение QGroundControl..."
            pkill -SIGKILL QGroundControl
        fi
    fi

    # Удаление временного файла с паролем
    rm -f "$PASSWORD_FILE"
    echo "Все процессы завершены."
}

# Регистрируем функцию cleanup для выполнения при завершении скрипта
trap cleanup EXIT

# Запуск всех клиентских компонентов
run_command "bash $PROJECT_ROOT/scripts/start_px4.sh" true
run_command "bash $PROJECT_ROOT/scripts/start_foxglove.sh"
run_command "bash $PROJECT_ROOT/scripts/start_qgc.sh"
run_with_sudo "bash $PROJECT_ROOT/scripts/start_micro_xrce_dds.sh"

# ROS2 modules
cd $PROJECT_ROOT/src/client/ros2
source /opt/ros/humble/setup.bash
colcon build
source install/local_setup.bash

# Запуск ROS2-скриптов в новых вкладках терминала
run_command "bash $PROJECT_ROOT/scripts/start_sensor_listener.sh" 
# run_command "bash $PROJECT_ROOT/scripts/start_translation_node.sh" true  # Проблема с зависимостями
# run_command "bash $PROJECT_ROOT/scripts/start_foxglove_bridge.sh" true
# run_command "bash $PROJECT_ROOT/scripts/start_px4_websocket_bridge.sh" true
run_command "bash $PROJECT_ROOT/scripts/start_log_sender_node.sh" true
# run_command "bash $PROJECT_ROOT/scripts/start_mavlink_streaming.sh" true  # временно не работает

echo "Все компоненты запущены."

# Ожидание завершения (например, по нажатию Ctrl+C)
echo "Нажмите Ctrl+C для завершения..."
while true; do
    sleep 1
done


