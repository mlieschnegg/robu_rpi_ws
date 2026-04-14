#!/bin/bash

set -e

echo "Starting setup for ROBU"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/rpi_detect.sh"

WORK_DIR="$HOME/work"
VENV_DIR="$WORK_DIR/.venvs/robu"
SYS_PYTHON="/usr/bin/python3"

setup_venv_auto_activate() {
    local line='if [ -d "$HOME/work/.venvs/robu" ] && [ -z "$VIRTUAL_ENV" ]; then source "$HOME/work/.venvs/robu/bin/activate"; fi'

    if ! grep -Fq 'work/.venvs/robu/bin/activate' "$HOME/.bashrc"; then
        echo "$line" >> "$HOME/.bashrc"
        echo "[ROBU] Added venv auto-activation to .bashrc"
    fi
}

setup_keyboard() {
    sudo sed -i -E 's/^XKBLAYOUT="[^"]*"/XKBLAYOUT="de"/' /etc/default/keyboard
}

install_base_packages() {
    sudo apt update

    sudo apt install -y openssh-server
    sudo systemctl enable --now ssh

    sudo apt install -y \
        git \
        btop \
        tmux \
        neovim \
        ncdu \
        python3-pip \
        python3-venv \
        python3-opencv \
        python3-numpy \
        python3-dev \
        python3-pillow \
        v4l-utils \
        p7zip-full \
        curl \
        wget \
        software-properties-common

    if is_raspberry_pi; then
        sudo apt install -y \
            python3-smbus2 \
            python3-rpi.gpio \
            python3-spidev \
            i2c-tools
    else
        sudo apt install -y terminator rpi-imager
    fi
}

setup_python_venv() {
    mkdir -p "$WORK_DIR/.venvs"

    if [ ! -d "$VENV_DIR" ]; then
        python3 -m venv --system-site-packages "$VENV_DIR"
    fi

    source "$VENV_DIR/bin/activate"
    python -m pip install --upgrade pip
}

install_system_python_pkg_if_missing() {
    local module_name="$1"
    local pip_name="${2:-$1}"

    if $SYS_PYTHON -c "import ${module_name}" >/dev/null 2>&1; then
        echo "[ROBU] System Python package already present: ${module_name}"
        return 0
    fi

    echo "[ROBU] Installing system Python package via pip: ${pip_name}"
    sudo $SYS_PYTHON -m pip install --break-system-packages "$pip_name"
}

install_venv_python_pkg_if_missing() {
    local module_name="$1"
    local pip_name="${2:-$1}"

    source "$VENV_DIR/bin/activate"

    if python -c "import ${module_name}" >/dev/null 2>&1; then
        echo "[ROBU] Venv Python package already present: ${module_name}"
        return 0
    fi

    echo "[ROBU] Installing venv Python package: ${pip_name}"
    python -m pip install "$pip_name"
}

install_runtime_sensor_packages() {
    if ! is_raspberry_pi; then
        return
    fi

    #
    # ROS-/Runtime-relevante Sensorpakete:
    # müssen für /usr/bin/python3 verfügbar sein
    #

    install_system_python_pkg_if_missing "rpi_ws281x" "rpi_ws281x"

    # SparkFun VL53L5CX
    install_system_python_pkg_if_missing "qwiic_vl53l5cx" "git+https://github.com/sparkfun/Qwiic_VL53L5CX_Py.git"

    # Bei Bedarf später ergänzen:
    # install_system_python_pkg_if_missing "adafruit_vl53l1x" "adafruit-circuitpython-vl53l1x"
    # install_system_python_pkg_if_missing "adafruit_vl53l0x" "adafruit-circuitpython-vl53l0x"
    # install_system_python_pkg_if_missing "adafruit_vl53l4cd" "adafruit-circuitpython-vl53l4cd"
}

install_python_packages() {
    #
    # Venv nur für Dev-/Hilfspakete
    #
    # install_venv_python_pkg_if_missing "PIL" "pillow"

    #
    # ROS-/Runtime-relevante Pakete systemweit
    #
    install_runtime_sensor_packages
}

install_pc_tools() {
    if is_raspberry_pi; then
        return
    fi

    # sudo add-apt-repository --yes ppa:kicad/kicad-9.0-releases
    # sudo apt install -y --install-recommends kicad

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
            bash "$SCRIPT_DIR/$script"
        else
            echo "Skipping missing script: $script"
        fi
    done

    if ! is_raspberry_pi && [ -f "$SCRIPT_DIR/machine_learning_setup.sh" ]; then
        bash "$SCRIPT_DIR/machine_learning_setup.sh"
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
    [ -e /dev/spidev0.0 ] && sudo chmod a+rw /dev/spidev0.0
    [ -e /dev/spidev0.1 ] && sudo chmod a+rw /dev/spidev0.1
}

install_pishrink() {
    cd "$WORK_DIR" || exit 1
    wget -O pishrink.sh https://raw.githubusercontent.com/Drewsif/PiShrink/master/pishrink.sh
    sudo install -m 755 pishrink.sh /usr/local/bin/pishrink.sh
    rm -f pishrink.sh
}

print_runtime_checks() {
    if ! is_raspberry_pi; then
        return
    fi

    echo
    echo "Runtime checks:"
    echo "  /usr/bin/python3 -c \"import smbus2\""
    echo "  /usr/bin/python3 -c \"import RPi.GPIO\""
    echo "  /usr/bin/python3 -c \"import spidev\""
    echo "  /usr/bin/python3 -c \"import rpi_ws281x\""
    echo "  /usr/bin/python3 -c \"import qwiic_vl53l5cx\""
}

main() {
    mkdir -p "$WORK_DIR"

    setup_keyboard
    install_base_packages
    install_exaloop
    setup_python_venv
    setup_venv_auto_activate
    install_python_packages
    install_pc_tools
    run_sub_setup_scripts
    setup_shell_and_permissions
    apply_runtime_device_permissions

    if is_raspberry_pi; then
        install_pishrink
    fi

    print_runtime_checks

    echo
    echo "Setup completed successfully!"
    echo "Python venv: $VENV_DIR"
    echo "Activate with: source $VENV_DIR/bin/activate"
}

main "$@"