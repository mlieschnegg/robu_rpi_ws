#!/bin/bash

set -e

echo "Starting setup for Raspberry Pi"

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

require_raspberry_pi() {
    if ! is_raspberry_pi; then
        echo "This script is only for Raspberry Pi"
        exit 1
    fi
}

install_apt_packages() {
    sudo apt update
    sudo apt install -y \
        rpi.gpio-common \
        python3-rpi.gpio \
        python3-pip \
        python3-venv \
        python3-dev \
        python3-smbus2 \
        python3-spidev \
        i2c-tools
}

ensure_group_and_membership() {
    local group_name="$1"

    if ! getent group "$group_name" > /dev/null; then
        sudo groupadd "$group_name"
    fi

    if ! id -nG "$USER" | grep -qw "$group_name"; then
        sudo usermod -aG "$group_name" "$USER"
    fi
}

setup_groups() {
    sudo adduser "$USER" dialout || true
    sudo adduser "$USER" video || true
    sudo adduser "$USER" kmem || true

    ensure_group_and_membership gpio
    ensure_group_and_membership spi

    if getent group i2c > /dev/null; then
        ensure_group_and_membership i2c
    fi
}

setup_udev_rules() {
    sudo tee /etc/udev/rules.d/99-gpio.rules > /dev/null <<'EOF'
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", MODE="0660", GROUP="gpio"
SUBSYSTEM=="gpio", KERNEL=="gpio*",     MODE="0660", GROUP="gpio"
KERNEL=="gpiomem", MODE="0660", GROUP="gpio"
EOF

    sudo tee /etc/udev/rules.d/99-spi.rules > /dev/null <<'EOF'
KERNEL=="spidev*", MODE="0660", GROUP="spi"
EOF

    sudo tee /etc/udev/rules.d/99-i2c.rules > /dev/null <<'EOF'
KERNEL=="i2c-[0-9]*", MODE="0660", GROUP="i2c"
EOF

    sudo tee /etc/udev/rules.d/99-ttyusb-acm.rules > /dev/null <<'EOF'
KERNEL=="ttyACM[0-9]*", MODE="0660", GROUP="dialout"
KERNEL=="ttyUSB[0-9]*", MODE="0660", GROUP="dialout"
EOF

    sudo udevadm control --reload-rules
    sudo udevadm trigger
}

setup_runtime_permissions() {
    [ -e /dev/ttyACM0 ] && sudo chmod a+rw /dev/ttyACM0
    [ -e /dev/gpiomem ] && sudo chmod a+rw /dev/gpiomem
    [ -e /dev/i2c-1 ] && sudo chmod a+rw /dev/i2c-1
    [ -e /dev/video0 ] && sudo chmod a+rw /dev/video0
    [ -e /dev/ttyUSB0 ] && sudo chmod a+rw /dev/ttyUSB0
    [ -e /dev/mem ] && sudo chmod a+rw /dev/mem
    [ -e /dev/spidev0.0 ] && sudo chmod a+rw /dev/spidev0.0
    [ -e /dev/spidev0.1 ] && sudo chmod a+rw /dev/spidev0.1
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

install_vl53_packages() {
    install_system_python_pkg_if_missing "adafruit_vl53l1x" "adafruit-circuitpython-vl53l1x"

    # Bei Bedarf später aktivieren:
    # install_system_python_pkg_if_missing "adafruit_vl53l0x" "adafruit-circuitpython-vl53l0x"
    # install_system_python_pkg_if_missing "adafruit_vl53l4cd" "adafruit-circuitpython-vl53l4cd"
}

install_camera_packages() {
    if ! $SYS_PYTHON -c "import picamera2" >/dev/null 2>&1; then
        echo "[ROBU] picamera2 not found in system Python."
        echo "[ROBU] Trying pip fallback for picamera2..."
        sudo $SYS_PYTHON -m pip install --break-system-packages picamera2 || true
    else
        echo "[ROBU] System Python package already present: picamera2"
    fi
}

install_python_packages() {
    #
    # ROS/runtime-relevante Python-Pakete:
    # müssen für /usr/bin/python3 verfügbar sein
    #

    install_system_python_pkg_if_missing "rpi_ws281x" "rpi_ws281x"
    install_vl53_packages
    install_camera_packages

    #
    # Venv nur für Dev-/Hilfspakete
    # Bei Bedarf aktivieren:
    #
    # install_venv_python_pkg_if_missing "pytest" "pytest"
    # install_venv_python_pkg_if_missing "black" "black"
    # install_venv_python_pkg_if_missing "flake8" "flake8"
}

setup_system_services() {
    sudo ln -sf "$HOME/work/.robu/config/service/robuboard_power_off_5v.service" \
        /etc/systemd/system/robuboard_power_off_5v.service

    sudo systemctl daemon-reload
    sudo systemctl enable robuboard_power_off_5v.service
}

setup_user_services() {
    mkdir -p "$HOME/.config/systemd/user"

    sudo loginctl enable-linger "$USER"

    ln -sf "$HOME/work/.robu/config/service/robuboard_powerswitch.service" \
        "$HOME/.config/systemd/user/robuboard_powerswitch.service"

    systemctl --user daemon-reload
    systemctl --user enable robuboard_powerswitch.service
    systemctl --user start robuboard_powerswitch.service || true
}

setup_fastdds_profile() {
    if /usr/bin/python3 -c "import importlib.util, sys; sys.exit(0 if importlib.util.find_spec('robuboard') else 1)"; then
        if /usr/bin/python3 -c "from robuboard.rpi.utils import is_robuboard_v0; print(is_robuboard_v0())" 2>/dev/null | grep -q "True"; then
            local line='export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/work/.robu/config/fastdds.xml"'
            if ! grep -Fq 'FASTRTPS_DEFAULT_PROFILES_FILE=' "$HOME/.bashrc" 2>/dev/null; then
                echo "$line" >> "$HOME/.bashrc"
            fi
        else
            echo "Skipping FASTDDS profile export: robuboard detected, but board is not v0 or check returned false."
        fi
    else
        echo "Skipping FASTDDS profile export: Python module 'robuboard' is not available yet."
    fi
}

print_runtime_checks() {
    echo
    echo "Runtime checks:"
    echo "  /usr/bin/python3 -c \"import smbus2\""
    echo "  /usr/bin/python3 -c \"import spidev\""
    echo "  /usr/bin/python3 -c \"import RPi.GPIO\""
    echo "  /usr/bin/python3 -c \"import rpi_ws281x\""
    echo "  /usr/bin/python3 -c \"import adafruit_vl53l1x\""
}

main() {
    require_raspberry_pi
    install_apt_packages
    setup_groups
    setup_udev_rules
    setup_runtime_permissions
    setup_python_venv
    setup_venv_auto_activate
    install_python_packages
    setup_system_services
    setup_user_services
    setup_fastdds_profile
    print_runtime_checks

    echo
    echo "Raspberry Pi setup completed."
    echo "Python venv: $VENV_DIR"
    echo "Activate with: source $VENV_DIR/bin/activate"
    echo "Log out and back in, or reboot, so new group memberships take effect."
}

main "$@"