#!/bin/bash

#set -e

echo "Starting setup for ROBU"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/rpi_detect.sh"

WORK_DIR="$HOME/work"
VENV_DIR="$WORK_DIR/.venvs/robu"

setup_keyboard() {
    sudo sed -i -E 's/^XKBLAYOUT="[^"]*"/XKBLAYOUT="de"/' /etc/default/keyboard
}

install_base_packages() {
    sudo apt install -y openssh-server
    sudo systemctl enable --now ssh

    sudo apt install -y git btop tmux neovim ncdu
    sudo apt install -y python3-pip python3-venv python3-opencv python3-numpy python3-dev
    sudo apt install -y v4l-utils p7zip-full curl software-properties-common

    if ! is_raspberry_pi; then
        sudo apt install -y terminator rpi-imager
    fi
}

setup_python_venv() {
    mkdir -p "$WORK_DIR/.venvs"

    if [ ! -d "$VENV_DIR" ]; then
        python3 -m venv --system-site-packages "$VENV_DIR"
    fi

    source "$VENV_DIR/bin/activate"
}

install_python_packages() {
    source "$VENV_DIR/bin/activate"
    python -m pip install pillow rpi_ws281x

    if is_raspberry_pi; then
        python -m pip install smbus2 RPi.GPIO
        python -m pip install git+https://github.com/sparkfun/Qwiic_VL53L5CX_Py.git
    fi
}

install_pc_tools() {
    if is_raspberry_pi; then
        return
    fi

    sudo add-apt-repository --yes ppa:kicad/kicad-9.0-releases
    sudo apt install -y --install-recommends kicad

    sudo snap install gimp
    sudo snap install code --classic

    code --install-extension platformio.platformio-ide
    code --install-extension ms-vscode-remote.remote-ssh
    code --install-extension ms-vscode-remote.remote-ssh-edit
    code --install-extension ms-vscode.remote-explorer
    code --install-extension ranch-hand-robotics.rde-pack
    code --install-extension ranch-hand-robotics.rde-ros-2
    code --install-extension ranch-hand-robotics.urdf-editor
    code --install-extension pdconsec.vscode-print

    if [ -f "$SCRIPT_DIR/link_vsc_snippets" ]; then
        . "$SCRIPT_DIR/link_vsc_snippets"
    fi

    mkdir -p "$HOME/Desktop"
    cp -r "$HOME/work/.robu/config/desktop/"*.* "$HOME/Desktop" 2>/dev/null || true
}

install_exaloop() {
    /bin/bash -c "$(curl -fsSL https://exaloop.io/install.sh)"
}

run_sub_setup_scripts() {
    for script in git_setup.sh ros_setup.sh microros_setup.sh; do
        if [ -f "$SCRIPT_DIR/$script" ]; then
            . "$SCRIPT_DIR/$script"
        else
            echo "Skipping missing script: $script"
        fi
    done

    if ! is_raspberry_pi && [ -f "$SCRIPT_DIR/machine_learning_setup.sh" ]; then
        . "$SCRIPT_DIR/machine_learning_setup.sh"
    fi
}

setup_shell_and_permissions() {
    echo "$USER ALL=(ALL) NOPASSWD:ALL" | sudo tee "/etc/sudoers.d/$USER" > /dev/null

    mkdir -p "$HOME/.colcon"
    ln -sf "$HOME/work/.robu/config/.colcon/defaults.yaml" "$HOME/.colcon/defaults.yaml"

    sudo adduser "$USER" dialout || true
    sudo adduser "$USER" video || true
    sudo adduser "$USER" kmem || true
}

apply_runtime_device_permissions() {
    if ! is_raspberry_pi; then
        return
    fi

    [ -e /dev/ttyACM0 ] && sudo chmod a+rw /dev/ttyACM0
    [ -e /dev/gpiomem ] && sudo chmod a+rw /dev/gpiomem
    [ -e /dev/i2c-1 ] && sudo chmod a+rw /dev/i2c-1
    [ -e /dev/video0 ] && sudo chmod a+rw /dev/video0
    [ -e /dev/ttyUSB0 ] && sudo chmod a+rw /dev/ttyUSB0
    [ -e /dev/mem ] && sudo chmod a+rw /dev/mem
}

install_pishrink() {
    cd "$WORK_DIR" || exit 1
    wget -O pishrink.sh https://raw.githubusercontent.com/Drewsif/PiShrink/master/pishrink.sh
    sudo install -m 755 pishrink.sh /usr/local/bin/pishrink.sh
    rm -f pishrink.sh
}

main() {
    mkdir -p "$WORK_DIR"

    setup_keyboard
    install_base_packages
    install_exaloop
    setup_python_venv
    install_python_packages
    install_pc_tools
    run_sub_setup_scripts
    setup_shell_and_permissions
    apply_runtime_device_permissions
    install_pishrink

    echo "Setup completed successfully!"
    echo "Python venv: $VENV_DIR"
    echo "Activate with: source $VENV_DIR/bin/activate"
}

main "$@"
