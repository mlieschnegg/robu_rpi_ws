#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup_common.sh"
source "$SCRIPT_DIR/rpi_detect.sh"

SYS_PYTHON="/usr/bin/python3"

WORK_DIR="$HOME/work"
MICROROS_WS_PATH="$WORK_DIR/microros_ws"
PLATFORMIO_BIN="$HOME/.platformio/penv/bin"

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
    for cmd in git rosdep ros2; do
        if ! command -v "$cmd" >/dev/null 2>&1; then
            echo "Missing required command: $cmd"
            missing=1
        fi
    done
    if [ ! -x /usr/bin/colcon ]; then
        echo "Missing required command: /usr/bin/colcon. Install python3-colcon-common-extensions."
        missing=1
    fi
    [ "$missing" -eq 0 ] || exit 1
}

install_base_packages() {
    sudo apt update
    sudo apt install -y python3-pip python3-venv python3-rosdep libusb-dev \
        python3-yaml python3-jinja2 python3-typeguard
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

    /usr/bin/colcon build
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

install_platformio() {
    # Use the same isolated Core on PC and Pi, even with the dev venv active.
    if [ ! -x "$PLATFORMIO_BIN/pio" ] || [ ! -x "$PLATFORMIO_BIN/platformio" ] ||
       ! "$PLATFORMIO_BIN/pio" --version >/dev/null 2>&1; then
        local installer
        if installer="$(mktemp "${TMPDIR:-/tmp}/robu-platformio.XXXXXX.py")"; then
            :
        else
            robu_record_failure "$?" "PlatformIO" "Cannot create installer temporary file"
            return 0
        fi
        if curl -fsSL -o "$installer" \
            https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py; then
            :
        else
            robu_record_failure "$?" "PlatformIO" "Core installer download failed"
            rm -f "$installer"
            return 0
        fi
        if "$SYS_PYTHON" "$installer"; then
            rm -f "$installer"
        else
            robu_record_failure "$?" "PlatformIO" "Core installer failed"
            rm -f "$installer"
            return 0
        fi
    fi
    setup_tool_paths --platformio-bin "$PLATFORMIO_BIN" --link-platformio
    "$PLATFORMIO_BIN/pio" --version
}

install_raspberry_pi_tools() {
    if ! is_raspberry_pi; then
        return
    fi

    cd "$WORK_DIR" || exit 1

    append_once 'export PATH="$PATH:$HOME/work/.robu/scripts"'

    if [ ! -d "$WORK_DIR/teensy_loader_cli" ]; then
        git clone https://github.com/PaulStoffregen/teensy_loader_cli "$WORK_DIR/teensy_loader_cli"
    fi

    if ! cd "$WORK_DIR/teensy_loader_cli"; then
        robu_record_failure 1 "Teensy loader" "Missing source directory"
        return 0
    fi
    make
    sudo install -m 755 teensy_loader_cli /usr/local/bin/teensy_loader_cli

}


main() {
    echo "Installing micro-ROS and PlatformIO"
    mkdir -p "$WORK_DIR"
    setup_tool_paths
    install_base_packages
    install_platformio
    install_teensy_rules
    install_raspberry_pi_tools
    require_ros
    prepare_ros_environment
    require_commands
    setup_microros_workspace
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    main "$@"
    if robu_setup_report; then exit 0; else exit 1; fi
fi
