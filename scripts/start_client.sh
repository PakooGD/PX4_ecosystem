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

# Функция для запуска команды в новом терминале (вкладке gnome-terminal)
run_in_gnome_terminal_tab() {
    local command="$1"
    local password=$(cat "$PASSWORD_FILE")

    if command -v gnome-terminal &> /dev/null; then
        if [[ "$command" == "bash $PROJECT_ROOT/scripts/start_micro_xrce_dds.sh" ]]; then
            gnome-terminal --tab -- bash -c "echo '$password' | sudo -S $command; exec bash" &
        else
            gnome-terminal --tab -- bash -c "$command; exec bash" &
        fi 
    else
        echo "gnome-terminal не найден.  Используйте run_in_background вместо этого."
        local command="$1"
        $command &
    fi
}

# Функция выбора способа запуска
run_command() {
  local command="$1"

  if command -v gnome-terminal &> /dev/null; then
     run_in_gnome_terminal_tab "$command"
  else
     run_in_background "$command"
  fi
}




# Запуск всех клиентских компонентов
run_command "bash $PROJECT_ROOT/scripts/start_px4.sh"
run_command "bash $PROJECT_ROOT/scripts/start_foxglove.sh"
run_command "bash $PROJECT_ROOT/scripts/start_qgc.sh"
run_command "bash $PROJECT_ROOT/scripts/start_micro_xrce_dds.sh"
# ROS2 modules
cd $PROJECT_ROOT/src/client/ros2
source /opt/ros/humble/setup.bash
colcon build
source install/local_setup.bash
run_command "bash $PROJECT_ROOT/scripts/start_sensor_listener.sh"
# run_command "bash $PROJECT_ROOT/scripts/start_translation_node.sh" Проблема с зависимостями
# run_command "bash $PROJECT_ROOT/scripts/start_foxglove_bridge.sh"
run_command "bash $PROJECT_ROOT/scripts/start_px4_websocket_bridge.sh"

# Удаляем временный файл после завершения
trap "rm -f $PASSWORD_FILE" EXIT

echo "Все компоненты запущены."

