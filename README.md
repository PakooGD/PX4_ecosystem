# Установка
    1. Клонируйте репозиторий: `git clone https://github.com/PakooGD/PX4_ecosystem.git`
    2. Запустите скрипт установки: `./setup.sh`
    3. Настройте алиасы: `source config/aliases.sh`

# Запуск
    - Запустите экосистему: `start_client`
    - Запустите сервер: `start_server`

# Важно
    1. Перед установкой, убедитесь, что у вас установлен Qt версии 6.8.2
    Можно скачать онлайн установщик, но понадобиться VPN: https://www.qt.io/offline-installers
    Можно загрузить оффлайн архив и установить вручную. Для этого выполните:
    - sudo apt update
    - sudo apt install build-essential libgl1-mesa-dev libssl-dev libxkbcommon-dev libxcb-xinerama0-dev libxcb-xinerama0
    - tar -xvf qt-everywhere-src-6.8.2.tar.xz
    - cd qt-everywhere-src-6.8.2
    - ./configure -prefix /opt/Qt/6.8.2 -opensource -confirm-license -nomake examples -nomake tests 
        ИЛИ 
      mkdir build >> cd build >> cmake -GNinja -DCMAKE_INSTALL_PREFIX=/opt/Qt/6.8.2 -DQT_BUILD_EXAMPLES=OFF -DQT_BUILD_TESTS=OFF ..
    - make -j$(nproc) 
        ИЛИ 
      ninja -j$(nproc)
    - sudo make install 
        ИЛИ 
      sudo ninja install
    - export PATH=/opt/Qt/6.8.2/bin:$PATH
    - qmake --version - проверка установки
    2. Убедитесь, что у вас установлен CMake (желательно, последней версии)
