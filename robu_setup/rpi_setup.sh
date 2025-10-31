#!/bin/bash

echo "Starting Setup for RPI"

source "rpi_detect.sh"

if ! is_raspberry_pi; then
    echo "This script is only for Raspberry Pi"
    exit 1
fi

#optional: Install wiringpi for GPIO control
#sudo apt install wiringpi

sudo apt install rpi.gpio-common

#Benutzer zur Gruppe dialout hinzufügen (z.B. serielle Schnittstelle)
sudo adduser $USER dialout
sudo adduser $USER video
sudo adduser $USER kmem

#sudo adduser $USER gpio -> gibt es eventuell nicht, Gruppe erstellen plus udev rule hinzufügen
sudo chown root:$USER /dev/gpiomem

#Zugriffsrechte hinzufügen (gehen nach dem Neustart verloren)
sudo chmod a+rw /dev/ttyACM0
sudo chmod a+rw /dev/gpiomem
sudo chmod a+rw /dev/i2c-1
sudo chmod a+rw /dev/video0
sudo chmod a+rw /dev/ttyUSB0
sudo chmod a+rw /dev/mem
sudo chmod a+rw /dev/spidev0.0
sudo chmod a+rw /dev/spidev0.1

pip install rpi_ws281x --break-system-packages
pip install smbus --break-system-packages
pip install smbus2 --break-system-packages
pip install RPi.GPIO --break-system-packages


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