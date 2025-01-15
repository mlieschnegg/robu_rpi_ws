#!/bin/bash
#set -e #Beendet das Skript bei Fehlern

export DISPLAY=:0
export ROBU_RPI_WS=/home/robu/work/.robu

#display is always on
xset s off
xset -dpms
xset s noblank

cd $ROBU_RPI_WS
git pull
cd

# Prüfen, ob der Prozess "screen" existiert
#if pgrep screen > /dev/null; then
#    killall screen
#fi
#killall -9 /usr/bin/python3

source /opt/ros/humble/setup.bash
source /home/robu/work/.robu/install/setup.bash

/usr/bin/python3 $ROBU_RPI_WS/autostart/ros_launcher.py
