#!/bin/bash
#set -e #Beendet das Skript bei Fehlern

export DISPLAY=:0
export ROBU_RPI_WS=/home/robu/work/.robu

#display is always on
xset s off
xset -dpms
xset s noblank

cd $ROBU_RPI_WS
# git pull
if git pull | grep -q 'Already up to date.'; then
    echo "Repository is already up to date."
else
    colcon build
fi
#go to home directory
cd

# Prüfen, ob der Prozess "screen" existiert
#if pgrep screen > /dev/null; then
#    killall screen
#fi
#killall -9 /usr/bin/python3

source /opt/ros/humble/setup.bash
source /home/robu/work/.robu/install/setup.bash

# source /home/robu/work/ROBU/robu_rpi_ws/install/setup.bash
# export PYTHONPATH=/home/robu/work/ROBU/robu_rpi_ws/src/robuboard/robuboard/rpi:$PYTHONPATH
# if /usr/bin/python3 -c "from robuboard import is_robuboard; print(is_robuboard())" | grep -q "True"; then
#     ros2 run robuboard powerswitch &
# fi

/usr/bin/python3 $ROBU_RPI_WS/autostart/ros_launcher.py
