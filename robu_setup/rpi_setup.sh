#!/bin/bash

echo "Starting Setup for RPI"

source "rpi_detect.sh"

if ! is_raspberry_pi; then
    echo "This script is only for Raspberry Pi"
    exit 1
fi


# Setup Bluetooth
sudo apt install -y bluetooth bluez libbluetooth-dev

sudo systemctl enable --now bluetooth
# systemctl status bluetooth --no-pager

# Check if bluetooth is working
hcitool dev
hcitool -i hci0 scan

# Pair with Wireless Game Controller (only once)
bluetoothctl scan on
#Game Controller 1
bluetoothctl trust 13:88:21:24:C4:E2
bluetoothctl connect 13:88:21:24:C4:E2
#Game Controller 2
bluetoothctl trust 50:AD:CE:BF:DD:9D
bluetoothctl connect 50:AD:CE:BF:DD:9D
bluetoothctl scan off


# # Ensure automatic Bluetooth connection to the controller on reboot
# cat <<EOF | sudo tee /etc/systemd/system/bt-controller-connect.service > /dev/null
# [Unit]
# Description=Connect Bluetooth Controller at boot
# After=bluetooth.target

# [Service]
# Type=oneshot
# ExecStart=/usr/bin/bluetoothctl connect 13:88:21:24:C4:E2

# [Install]
# WantedBy=multi-user.target
# EOF

# sudo systemctl enable bt-controller-connect.service