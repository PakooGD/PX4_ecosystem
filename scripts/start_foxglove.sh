#!/bin/bash

# Переход в директорию проекта
cd "$PROJECT_ROOT/src/client/foxglove" || { echo "Ошибка: Не удалось перейти в директорию."; exit 1; }

# Сборка веб-приложения
echo "Сборка веб-приложения в режиме разработки..."
if ! yarn web:build:dev; then
    echo "Ошибка: Сборка веб-приложения завершилась с ошибкой."
    exit 1
fi

# Запуск веб-сервера в фоновом режиме
echo "Запуск веб-сервера..."
yarn web:serve &
SERVER_PID=$!

# Функция для проверки доступности порта
wait_for_port() {
    local port=$1
    local timeout=$2
    local start_time=$(date +%s)
    while true; do
        if nc -z localhost "$port" > /dev/null 2>&1; then
            echo "Порт $port доступен!"
            return 0
        fi
        local current_time=$(date +%s)
        local elapsed_time=$((current_time - start_time))
        if [ "$elapsed_time" -gt "$timeout" ]; then
            echo "Таймаут ожидания порта $port"
            return 1
        fi
        sleep 1
    done
}

# Ожидание доступности порта 8080
if wait_for_port 8080 10; then
    # Открытие браузера
    if command -v xdg-open &> /dev/null; then
        xdg-open http://localhost:8080/
    elif command -v open &> /dev/null; then
        open http://localhost:8080/
    elif command -v start &> /dev/null; then
        start http://localhost:8080/
    else
        echo "Не удалось открыть браузер: команда xdg-open/open/start не найдена."
        exit 1
    fi
else
    echo "Веб-сервер не запустился вовремя."
    kill $SERVER_PID  # Завершение процесса сервера
    exit 1
fi

# Ожидание завершения работы сервера (если нужно)
wait $SERVER_PID