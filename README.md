# Installation
1. Clone the repository: `git clone https://github.com/PakooGD/PX4_ecosystem.git`
2. Run the installation script, which will download missing dependencies and sources: `chmod +x setup.sh && ./setup.sh`

# Launch
- Start the ecosystem: `start_client` - will build the project, update packages and start the ecosystem
- Start the server: `start_server` - will start the server

# Important
1. Some packages may not be available without VPN in Russia and Belarus
2. To compile, you will need CMake version 3.21+ and Python 3.
3. Before installing, make sure you have Qt version 6.8.2 installed. You can download it from the link: https://www.qt.io/download-qt-installer-oss
* You can download the archive with the sources and compile it manually, but after compilation there may be problems when assembling QGC: https://www.qt.io/offline-installers