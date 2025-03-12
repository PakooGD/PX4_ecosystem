#!/bin/bash

# Определяем PROJECT_ROOT, если он не задан
if [ -z "$PROJECT_ROOT" ]; then
    export PROJECT_ROOT=$(dirname "$(realpath "$0")")
    echo "PROJECT_ROOT установлен в: $PROJECT_ROOT"
fi

# Функция для запуска команды в новом терминале (вкладке gnome-terminal)
run_in_gnome_terminal_tab() {
    local command="$1"

    if command -v gnome-terminal &> /dev/null; then
        gnome-terminal --tab -- bash -c "$command; exec bash" &
    else
        echo "gnome-terminal не найден.  Используйте run_in_background вместо этого."
        run_in_background "$command"
    fi
}

# Функция для запуска команды в фоне (если gnome-terminal недоступен)
run_in_background() {
    local command="$1"
    $command &
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

run_command "bash $PROJECT_ROOT/scripts/start_foxglove.sh"
run_command "bash $PROJECT_ROOT/scripts/start_px4.sh"
run_command "bash $PROJECT_ROOT/scripts/start_micro_xrce_dds.sh"
run_command "bash $PROJECT_ROOT/scripts/start_qgc.sh"
# ROS2 modules
run_command "bash $PROJECT_ROOT/scripts/start_sensor_listener.sh"
run_command "bash $PROJECT_ROOT/scripts/start_foxglove_bridge.sh"


echo "Все компоненты запущены."
