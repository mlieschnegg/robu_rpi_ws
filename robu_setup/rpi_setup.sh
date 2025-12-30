#!/bin/bash

echo "Starting Setup for RPI"

source "rpi_detect.sh"

if ! is_raspberry_pi; then
    echo "This script is only for Raspberry Pi"
    exit 1
fi

#optional: Install wiringpi for GPIO control
#sudo apt install wiringpi

sudo apt -y install rpi.gpio-common
sudo apt -y install python3-rpi.gpio

#Benutzer zur Gruppe dialout hinzufügen (z.B. serielle Schnittstelle)
sudo adduser $USER dialout
sudo adduser $USER video
sudo adduser $USER kmem

# 1) Gruppe gpio anlegen (falls nicht vorhanden)
if ! getent group gpio > /dev/null; then
    sudo groupadd gpio
fi

# 2) User zur Gruppe hinzufügen
if ! id -nG $USER | grep -qw "gpio"; then
    sudo usermod -aG gpio $USER
fi

# 3) udev-Regel schreiben (atomar, sauber)
sudo tee "/etc/udev/rules.d/99-gpio.rules" > /dev/null <<'EOF'
# GPIO character devices
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", MODE="0660", GROUP="gpio"
SUBSYSTEM=="gpio", KERNEL=="gpio*",     MODE="0660", GROUP="gpio"

# GPIO memory access (if available)
KERNEL=="gpiomem", MODE="0660", GROUP="gpio"
EOF

# 4) udev neu laden
sudo udevadm control --reload-rules
sudo udevadm trigger

sudo groupadd spi
sudo usermod -aG spi $USER
echo 'KERNEL=="spidev*", GROUP="spi", MODE="0660"' | sudo tee /etc/udev/rules.d/99-spi.rules > /dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger

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
pip install smbus3 --break-system-packages
pip install RPi.GPIO --break-system-packages
sudo pip3 install adafruit-circuitpython-vl53l1x --break-system-packages


#sudo cp robuboard_power_off_5v.service /etc/systemd/system/robuboard_power_off_5v.service
sudo ln -sf ~/work/.robu/config/service/robuboard_power_off_5v.service /etc/systemd/system/robuboard_power_off_5v.service
sudo systemctl daemon-reload
sudo systemctl enable robuboard_power_off_5v.service

sudo loginctl enable-linger robu
ln -sf ~/work/.robu/config/service/robuboard_powerswitch.service ~/.config/systemd/user/robuboard_powerswitch.service
systemctl --user daemon-reload
systemctl --user enable robuboard_powerswitch.service
systemctl --user start robuboard_powerswitch.service


if /usr/bin/python3 -c "from robuboard.rpi.utils import is_robuboard_v0; print(is_robuboard_v0())" | grep -q "True"; then
    CMD_TEXT="export FASTRTPS_DEFAULT_PROFILES_FILE=/home/robu/work/.robu/config/fastdds.xml"
    if ! grep -q "$CMD_TEXT" ~/.bashrc; then
        echo "$CMD_TEXT" >> ~/.bashrc
    fi
fi


# # Setup Bluetooth
# sudo apt install -y bluetooth bluez libbluetooth-dev

# sudo systemctl enable --now bluetooth
# # systemctl status bluetooth --no-pager

# # Check if bluetooth is working
# hcitool dev
# hcitool -i hci0 scan

# # Pair with Wireless Game Controller (only once)
# bluetoothctl scan on
# #Game Controller 1
# bluetoothctl trust 13:88:21:24:C4:E2
# bluetoothctl connect 13:88:21:24:C4:E2
# #Game Controller 2
# bluetoothctl trust 50:AD:CE:BF:DD:9D
# bluetoothctl connect 50:AD:CE:BF:DD:9D
# bluetoothctl scan off


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
