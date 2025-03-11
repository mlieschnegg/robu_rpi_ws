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
if ! (git pull | grep -q 'Already up to date.'); then
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

source /home/robu/work/microros_ws/install/local_setup.bash
source /home/robu/work/robocup-ros/install/setup.bash
source /home/robu/work/robocup-gui/install/setup.bash

if /usr/bin/python3 -c "from robuboard.rpi.utils import is_robuboard; print(is_robuboard())" | grep -q "True"; then
    echo "Robuboard detected! Starting Robuboard services..."
    ros2 run robuboard powerswitch &
    ros2 launch robuboard set_status_led.launch.py
fi

/usr/bin/python3 $ROBU_RPI_WS/autostart/ros_launcher.py
