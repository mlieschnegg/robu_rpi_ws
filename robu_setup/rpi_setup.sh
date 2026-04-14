#!/bin/bash

#set -e

echo "Starting setup for Raspberry Pi"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/rpi_detect.sh"

WORK_DIR="$HOME/work"
VENV_DIR="$WORK_DIR/.venvs/robu"

require_raspberry_pi() {
    if ! is_raspberry_pi; then
        echo "This script is only for Raspberry Pi"
        exit 1
    fi
}

install_apt_packages() {
    sudo apt install -y rpi.gpio-common python3-rpi.gpio
    sudo apt install -y python3-pip python3-venv python3-dev
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
}

install_python_packages() {
    source "$VENV_DIR/bin/activate"
    python -m pip install rpi_ws281x smbus2 RPi.GPIO adafruit-circuitpython-vl53l1x picamera2
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
    systemctl --user start robuboard_powerswitch.service
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

main() {
    require_raspberry_pi
    install_apt_packages
    setup_groups
    setup_udev_rules
    setup_runtime_permissions
    setup_python_venv
    install_python_packages
    setup_system_services
    setup_user_services
    setup_fastdds_profile

    echo "Raspberry Pi setup completed."
    echo "Python venv: $VENV_DIR"
    echo "Activate with: source $VENV_DIR/bin/activate"
    echo "Log out and back in, or reboot, so new group memberships take effect."
}

main "$@"
