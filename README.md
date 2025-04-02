# Important Info and Dependencies
1. Project was succesfully built on Ununtu 22.04, it (may)will not work properly with Win/Mac and other Linux versions coz it uses GnomeTerminal for launch and linux-related commands.
2. Some packages are unavailable without VPN in Russia and Belarus: QT6.8.2 which is nessesary to build QGC, some package's dependencies from gitlab.
3. To compile, you will need CMake version 3.21 (less than 3.5 at least) and Python 3.
4. Before installing, make sure you have Qt version 6.8.2 installed. You can download it from the link: https://www.qt.io/download-qt-installer-oss
* You can download the archive with the sources and compile it manually, but after compilation there may be problems when assembling QGC: https://www.qt.io/offline-installers
5. You must have at least 10 GB of free disk space.

# Installation
1. Clone the repository: `git clone https://github.com/PakooGD/PX4_ecosystem.git`
2. Open config/ and in the env.sh set ur directory. Actually it should work without this step, but it is better to do it.
2. Go to the root directory and run the installation script, which will download missing dependencies and sources: `chmod +x setup.sh && ./setup.sh`
* It may take some time (about 20 minutes) depending on your system configuration

# Launch
- Start the ecosystem from root: `start_client` - will build the project, update packages and start the ecosystem
- Start the server from src/server: `npm start` - will start the server

# Docker (Not Working yet)
1. Build: docker-compose build
2. Launch conatiners: docker-compose up



