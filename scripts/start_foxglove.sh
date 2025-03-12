   #!/bin/bash
   cd $PROJECT_ROOT/src/client/foxglove-studio
   yarn web:build:dev
   yarn web:serve &  # Запустить сервер в фоне

   # Функция для проверки доступности порта
   wait_for_port() {
     port=$1
     timeout=$2
     start_time=$(date +%s)
     while true; do
       nc -z localhost "$port" > /dev/null 2>&1
       if [ $? -eq 0 ]; then
         echo "Порт $port доступен!"
         return 0
       fi
       current_time=$(date +%s)
       elapsed_time=$((current_time - start_time))
       if [ "$elapsed_time" -gt "$timeout" ]; then
         echo "Таймаут ожидания порта $port"
         return 1
       fi
       sleep 1
     done
   }

   # Ждать, пока порт 8080 не станет доступным в течение 30 секунд
   if wait_for_port 8080 30; then
     xdg-open http://localhost:8080/
   else
     echo "Веб-сервер не запустился вовремя."
     exit 1
   fi
