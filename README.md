# Installation
* Project was built on Ununtu 22.04, it (may)will not work properly with Win/Mac and other Linux versions.
1. Clone the repository: `git clone https://github.com/PakooGD/PX4_ecosystem.git`
2. Go to the root directory and run the installation script, which will download missing dependencies and sources: `chmod +x setup.sh && ./setup.sh`

# Launch
- Start the ecosystem from root: `start_client` - will build the project, update packages and start the ecosystem
- Start the server from src/server: `npm start` - will start the server

# Important
1. Some packages may not be available without VPN in Russia and Belarus: QT6.8.2 is nessesary to build QGC, some gitlab dependencies.
2. To compile, you will need CMake version 3.21+ and Python 3.
3. Before installing, make sure you have Qt version 6.8.2 installed. You can download it from the link: https://www.qt.io/download-qt-installer-oss
* You can download the archive with the sources and compile it manually, but after compilation there may be problems when assembling QGC: https://www.qt.io/offline-installers

# Docker
1. Build: docker-compose build
2. Launch conatiners: docker-compose up
* Get access to client: docker exec -it client bash
* Get access to server: localhost:3000

