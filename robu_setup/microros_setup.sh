#!/bin/bash

#set -e

echo "Installing micro-ROS"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/rpi_detect.sh"

WORK_DIR="$HOME/work"
MICROROS_WS_PATH="$WORK_DIR/microros_ws"

append_once() {
    local line="$1"
    if ! grep -Fqx "$line" "$HOME/.bashrc" 2>/dev/null; then
        echo "$line" >> "$HOME/.bashrc"
    fi
}

require_ros() {
    if [ -z "${ROS_DISTRO:-}" ]; then
        echo "ROS_DISTRO is not set. Run ros_setup.sh first, then open a new shell or source /opt/ros/<distro>/setup.bash."
        exit 1
    fi

    if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
        echo "Missing ROS installation: /opt/ros/$ROS_DISTRO/setup.bash"
        exit 1
    fi
}

require_commands() {
    local missing=0
    for cmd in git colcon rosdep ros2; do
        if ! command -v "$cmd" >/dev/null 2>&1; then
            echo "Missing required command: $cmd"
            missing=1
        fi
    done
    [ "$missing" -eq 0 ] || exit 1
}

install_base_packages() {
    sudo apt update
    sudo apt install -y python3-pip python3-rosdep libusb-dev
}

prepare_ros_environment() {
    source "/opt/ros/$ROS_DISTRO/setup.bash"
}

setup_microros_workspace() {
    mkdir -p "$MICROROS_WS_PATH/src"
    cd "$MICROROS_WS_PATH" || exit 1

    if [ ! -d "$MICROROS_WS_PATH/src/micro_ros_setup" ]; then
        git clone -b "$ROS_DISTRO" https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
    fi

    if ! rosdep db >/dev/null 2>&1; then
        sudo rosdep init || true
    fi

    rosdep update
    rosdep install --from-paths src --ignore-src -y

    colcon build
    source install/local_setup.bash

    ros2 run micro_ros_setup create_agent_ws.sh
    ros2 run micro_ros_setup build_agent.sh

    append_once 'source ~/work/microros_ws/install/local_setup.bash'
}

install_teensy_rules() {
    cd "$WORK_DIR" || exit 1
    curl -fsSL -o 00-teensy.rules https://www.pjrc.com/teensy/00-teensy.rules
    sudo install -m 644 00-teensy.rules /etc/udev/rules.d/00-teensy.rules
    rm -f 00-teensy.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
}

install_raspberry_pi_tools() {
    if ! is_raspberry_pi; then
        return
    fi

    cd "$WORK_DIR" || exit 1

    curl -fsSL -o get-platformio.py \
        https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
    python3 get-platformio.py

    mkdir -p "$HOME/.local/bin"
    ln -sf "$HOME/.platformio/penv/bin/platformio" "$HOME/.local/bin/platformio"
    ln -sf "$HOME/.platformio/penv/bin/pio" "$HOME/.local/bin/pio"
    ln -sf "$HOME/.platformio/penv/bin/piodebuggdb" "$HOME/.local/bin/piodebuggdb"

    if [ ! -d "$WORK_DIR/teensy_loader_cli" ]; then
        git clone https://github.com/PaulStoffregen/teensy_loader_cli "$WORK_DIR/teensy_loader_cli"
    fi

    cd "$WORK_DIR/teensy_loader_cli" || exit 1
    make
    sudo install -m 755 teensy_loader_cli /usr/local/bin/teensy_loader_cli
}

main() {
    require_ros
    install_base_packages
    prepare_ros_environment
    require_commands
    setup_microros_workspace
    install_teensy_rules
    install_raspberry_pi_tools

    echo "micro-ROS setup completed."
}

main "$@"
