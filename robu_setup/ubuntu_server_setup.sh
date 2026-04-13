#!/bin/bash

#set -e

echo "Starting Ubuntu Server setup"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export QT_QPA_PLATFORM=linuxfb
export QT_QPA_EVDEV_MOUSE_PARAMETERS=/dev/input/event2
export QT_QPA_EVDEV_TOUCHSCREEN_PARAMETERS=/dev/input/event5
export QT_QPA_EVDEV_KEYBOARD_PARAMETERS=/dev/input/event3

install_qt_packages() {
    sudo apt update
    sudo apt install -y \
        python3-pyqt5 \
        libqt5gui5t64 \
        qtbase5-private-dev \
        libinput-dev \
        libevdev2 \
        libmtdev1t64 \
        evtest
}

run_sub_setup_scripts() {
    for script in \
        robu_setup.sh \
        rpi_setup.sh \
        rpi_camera_setup.sh \
        network_setup.sh \
        cleanup.sh
    do
        if [ -f "$SCRIPT_DIR/$script" ]; then
            . "$SCRIPT_DIR/$script"
        else
            echo "Skipping missing script: $script"
        fi
    done
}

main() {
    # Add specific group memberships here if you really need them, e.g.:
    # sudo usermod -aG dialout "$USER"

    sudo apt update
    sudo apt upgrade -y

    install_qt_packages
    run_sub_setup_scripts

    echo "Ubuntu Server setup completed."
}

main "$@"
