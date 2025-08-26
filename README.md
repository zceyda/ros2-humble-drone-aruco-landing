# ros2-humble-drone-aruco-landing
ROS2 is a Humble-based drone control software. It features manual flight using DroneKit and autonomous landing with ArUco marker detection.

## Features
---
ROS2 Humble support. Manual control with Dronekit (w, a, s, d, e, f, q via keyboard). ArUco marker detection and precision landing algorithm. OpenCV based image processing. Speed ​​and descent commands via MAVLink. Gazebo Harmonic.

Requirements: Ubuntu 22.04 + ROS2 Humble. Python 3.10+. Dronekit, MAVProxy. OpenCV, cv_bridge, numpy, pynput. ArUco (OpenCV-contrib). Gazebo Harmonic.

## Setup
---
### 1- Check for Updates
sudo apt update && sudo apt dist-upgrade

### 2- Moving the Window to the Background
gsettings set org.gnome.shell.extensions.dash-to-dock click-action 'minimize'

### 3- Tuxedo - Monster Control Center Equivalent
echo 'deb [arch=amd64 signed-by=/usr/share/keyrings/tuxedo-archive-keyring.gpg] https://deb.tuxedocomputers.com/ubuntu jammy main' | sudo tee /etc/apt/sources.list.d/tuxedo-computers.list

sudo apt update --allow-insecure-repositories
sudo apt install --allow-unauthenticated tuxedo-archive-keyring
sudo apt install tuxedo-control-center

### 4- GDEBI Installation
sudo apt-get install gdebi -y

### 5- Installation of Basic Packages and Libraries
sudo apt install htop neofetch bpytop clang cargo libc6-i386 libc6-x32 libu2f-udev samba-common-bin exfat-fuse default-jdk curl wget unrar linux-headers-$(uname -r) linux-headers-generic git gstreamer1.0-vaapi unzip ntfs-3g p7zip htop neofetch bpytop git gcc make curl bzip2 tar

### 6- Ardupilot Installation
sudo apt-get install git
sudo apt-get install gitk git-gui
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

### 7- Ardupilot Sitl Installation and Usage
cd ardupilot
./waf configure --board sitl
sudo apt install python3-empy
sudo apt install python3-future
./waf copter
cd ArduCopter
../Tools/autotest/sim_vehicle.py --map --console

### 8- Pymavlink Mavproxy Installation
pip install --upgrade pymavlink MAVProxy

### 9- Gazebo Ardupilot Installation
sudo apt-get update
sudo apt-get install lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic

### !!!!!WE WILL CONTINUE THE OPERATIONS AFTER CLOSING THE PREVIOUS TERMINAL AND OPENING A NEW TERMINAL!!!!!
sudo apt update
sudo apt install libgz-sim8-dev rapidjson-dev
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl

git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4

export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc


export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc

### !!!!! IN 3 SEPARATE TERMİNALS!!!!!
#### --to run a gazebo--
gz sim -v4 -r iris_runway.sdf

#### --to run ardupilot sitl--
cd ~/ardupilot/Tools/autotest
python sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console

#### --to connect to Gstream--
gst-launch-1.0 udpsrc port=5600 ! application/x-rtp, encoding-name=H264,payload=96 ! \
rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink

---
This project serves as a comprehensive reference for drone control based on ROS2 Humble and Python. The developed system includes both manual control and ArUco marker-based semi-autonomous landing capabilities and can be tested in both real drones and ArduPilot SITL or Gazebo Harmonic simulation environments. The open nature of the code allows users to adapt and expand the algorithm to their own needs. The project provides a valuable resource for academics and developers researching drone control software, image processing, and autonomous landing algorithms. Any feedback, bug reports, and contributions will significantly contribute to the further development of the project.









