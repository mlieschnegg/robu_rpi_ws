#!/bin/bash

#set -e

echo "Installing ROS"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/rpi_detect.sh"

append_once() {
    local line="$1"
    if ! grep -Fqx "$line" "$HOME/.bashrc" 2>/dev/null; then
        echo "$line" >> "$HOME/.bashrc"
    fi
}

install_locale() {
    sudo apt update
    sudo apt upgrade -y

    sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 LC_NUMERIC=en_US.UTF-8
    export LANG=en_US.UTF-8
}

setup_ros_repository() {
    sudo apt install -y software-properties-common curl
    sudo add-apt-repository -y universe

    export ROS_APT_SOURCE_VERSION
    ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')"

    curl -L -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "${VERSION_CODENAME}")_all.deb"

    sudo dpkg -i /tmp/ros2-apt-source.deb
    sudo apt update
    sudo apt install -y ros-dev-tools
}

install_ros_distribution() {
    local ubuntu_version
    ubuntu_version="$(lsb_release -rs)"

    case "$ubuntu_version" in
        22.04)
            if is_raspberry_pi; then
                sudo apt install -y ros-humble-desktop
            else
                sudo apt install -y ros-humble-desktop-full
            fi
            append_once 'source /opt/ros/humble/setup.bash'
            ;;
        24.04)
            if is_raspberry_pi; then
                sudo apt install -y ros-jazzy-desktop
            else
                sudo apt install -y ros-jazzy-desktop-full
            fi
            append_once 'source /opt/ros/jazzy/setup.bash'
            ;;
        *)
            echo "Unsupported Ubuntu version: $ubuntu_version"
            return 1
            ;;
    esac

    source ~/.bashrc
}

check_ros_environment() {
    if [ -z "${ROS_DISTRO:-}" ]; then
        echo "ROS_DISTRO is not set. Open a new shell or source your ROS setup first."
        return 1
    fi

    if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
        echo "ROS setup file not found: /opt/ros/$ROS_DISTRO/setup.bash"
        return 1
    fi
}

install_ros_packages() {
    check_ros_environment || return 1

    sudo apt install -y \
        "ros-${ROS_DISTRO}-ros2-control" \
        "ros-${ROS_DISTRO}-ros2-controllers" \
        "ros-${ROS_DISTRO}-ros-gz" \
        "ros-${ROS_DISTRO}-dynamixel-sdk" \
        "ros-${ROS_DISTRO}-dynamixel-interfaces" \
        "ros-${ROS_DISTRO}-moveit" \
        "ros-${ROS_DISTRO}-tf-transformations" \
        "ros-${ROS_DISTRO}-cartographer" \
        "ros-${ROS_DISTRO}-cartographer-ros" \
        "ros-${ROS_DISTRO}-navigation2" \
        "ros-${ROS_DISTRO}-nav2-bringup" \
        "ros-${ROS_DISTRO}-rosbridge-server" \
        "ros-${ROS_DISTRO}-joint-state-publisher-gui" \
        "ros-${ROS_DISTRO}-twist-mux" \
        "ros-${ROS_DISTRO}-rqt-image-view" \
        "ros-${ROS_DISTRO}-joy" \
        "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp" \
        python3-colcon-clean \
        joystick

    if ! is_raspberry_pi; then
        sudo apt install -y "ros-${ROS_DISTRO}-camera-ros"
    fi
}

setup_paths() {
    append_once 'export PATH="$PATH:$HOME/work/.robu/scripts"'
}

install_optional_pc_setup() {
    if ! is_raspberry_pi && [ -f "./turtlebot_setup.sh" ]; then
        . ./turtlebot_setup.sh
    fi
}

main() {
    install_locale
    setup_ros_repository
    install_ros_distribution
    install_ros_packages
    setup_paths
    install_optional_pc_setup

    echo "ROS setup completed."
}

main "$@"
