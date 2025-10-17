#!/bin/bash

echo "Installation von ROS"

source "rpi_detect.sh"

cd ~/work/ws_turtlebot/
# Update and upgrade system
sudo apt update && sudo apt upgrade -y

# Set locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
#BUGFIX: fixing move-it issue and set LC_NUMERIC=en_US.UTF-8 -> https://github.com/moveit/moveit2/issues/1782
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 LC_NUMERIC=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt install ros-dev-tools

# Install ROS 2 Jazzy
sudo apt update
sudo apt upgrade

sudo apt install -y ros-dev-tools

# Aktuelle Ubuntu-Version abfragen
ubuntu_version=$(lsb_release -rs)
# Verzweigung basierend auf der Version
if [ "$ubuntu_version" = "22.04" ]; then
    echo "Ihre Ubuntu-Version ist: $ubuntu_version, es wird die ROS-Distro Humble installiert!"
    
    if is_raspberry_pi; then
        sudo apt install -y ros-humble-desktop
    else
        sudo apt install -y ros-humble-desktop-full
    fi

    CMD_TEXT="source /opt/ros/humble/setup.bash"
    if ! grep -q "$CMD_TEXT" ~/.bashrc; then
        echo "$CMD_TEXT" >> ~/.bashrc
    fi
elif [ "$ubuntu_version" = "24.04" ]; then
    echo "Ihre Ubuntu-Version ist: $ubuntu_version, es wird die ROS-Distro Jazzy installiert!"
    
    if is_raspberry_pi; then
        sudo apt install -y ros-jazzy-desktop
    else
        sudo apt install -y ros-jazzy-desktop-full
    fi

    CMD_TEXT="source /opt/ros/jazzy/setup.bash"
    if ! grep -q "$CMD_TEXT" ~/.bashrc; then
        echo "$CMD_TEXT" >> ~/.bashrc
    fi
else
  echo "Ihre Ubuntu-Version ($ubuntu_version) wird nicht explizit unterstützt oder ist nicht 22.04/24.04."
fi

source ~/.bashrc

sudo apt install -y ros-${ROS_DISTRO}-ros2-control
sudo apt install -y ros-${ROS_DISTRO}-ros2-controllers
sudo apt install -y ros-${ROS_DISTRO}-*-ros2-control
sudo apt install -y ros-${ROS_DISTRO}-ros-gz
sudo apt install -y ros-${ROS_DISTRO}-dynamixel-sdk
sudo apt install -y ros-${ROS_DISTRO}-dynamixel-interfaces
sudo apt install -y python3-colcon-clean
sudo apt install -y ros-${ROS_DISTRO}-moveit
sudo apt install -y ros-${ROS_DISTRO}-tf-transformations
sudo apt install -y ros-${ROS_DISTRO}-cartographer
sudo apt install -y ros-${ROS_DISTRO}-cartographer-ros
sudo apt install -y ros-${ROS_DISTRO}-navigation2
sudo apt install -y ros-${ROS_DISTRO}-nav2-bringup
sudo apt install -y ros-${ROS_DISTRO}-rosbridge-server
sudo apt install -y ros-${ROS_DISTRO}-joint-state-publisher-gui
sudo apt install -y ros-${ROS_DISTRO}-twist-mux
sudo apt install -y ros-${ROS_DISTRO}-rqt-image-view
sudo apt install -y ros-${ROS_DISTRO}-joy 
sudo apt install -y joystick
#sudo apt install -y ros-${ROS_DISTRO}-gazebo-ros-pkgs

if ! is_raspberry_pi; then #muss für den raspberry extra gebaut werden!!! -> rpi_camera_setup.sh
    sudo apt install -y ros-${ROS_DISTRO}-camera-ros
fi


. turtlebot_setup.sh