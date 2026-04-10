#!/bin/bash

echo "Starte Ubuntu server installation"

sudo usermod -aG $USER

QT_QPA_PLATFORM=linuxfb
QT_QPA_EVDEV_MOUSE_PARAMETERS=/dev/input/event2
QT_QPA_EVDEV_TOUCHSCREEN_PARAMETERS=/dev/input/event5
QT_QPA_EVDEV_KEYBOARD_PARAMETERS=/dev/input/event3

sudo apt install python3-pyqt5
libqt5gui5t64 \
  qtbase5-private-dev \
  libinput-dev \
  libevdev2 \
  libmtdev1t64 \
  evtest

. rpi_setup.sh
. robu_setup.sh
. rpi_camera_setup.sh
. rpi_detect.sh
. network_setup.sh

sudo apt update ; sudo apt upgrade

. cleanup.sh
