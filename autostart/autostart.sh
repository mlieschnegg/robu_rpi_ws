#!/bin/bash
#set -e #Beendet das Skript bei Fehlern

export DISPLAY=:0
export ROBU_RPI_WS=$HOME/work/.robu
#export ROBU_RPI_WS=$HOME/work/ROBU/robu_rpi_ws

DIR_MICRO_ROS="$HOME/work/microros_ws"
DIR_ROBOCUP_ROS="$HOME/work/robocup/robocup-ros"
DIR_ROBOCUP_TEENSY="$HOME/work/robocup/robocup-teensy"
DIR_ROBOCUP_GUI="$HOME/work/robocup/robocup-gui"

#display is always on
xset s off
xset -dpms
xset s noblank

cd $ROBU_RPI_WS
# git pull
if ! (git pull | grep -q 'Already up to date.'); then
    colcon build
fi

if [[ ! -d "$ROBU_RPI_WS/install" ]]; then
    colcon build
fi

# Prüfen, ob der Prozess "screen" existiert
#if pgrep screen > /dev/null; then
#    killall screen
#fi
#killall -9 /usr/bin/python3

source /opt/ros/jazzy/setup.bash
source $ROBU_RPI_WS/install/setup.bash

if [[ -d "$DIR_MICRO_ROS" ]]; then
    source $DIR_MICRO_ROS/install/local_setup.bash
fi

if [[ -d "$DIR_ROBOCUP_ROS" ]]; then
    echo "checking repository robocup24-ros..."
    cd $DIR_ROBOCUP_ROS
    if ! (git pull | grep -q 'Already up to date.'); then
        colcon build
    fi
    source $DIR_ROBOCUP_ROS/install/setup.bash
fi

if [[ -d "$DIR_ROBOCUP_GUI" ]]; then
    echo "checking repository robocup24-gui..."
    cd $DIR_ROBOCUP_GUI
    if ! (git pull | grep -q 'Already up to date.'); then
        colcon build
    fi
    source $DIR_ROBOCUP_GUI/install/setup.bash
fi

if /usr/bin/python3 -c "from robuboard.rpi.utils import is_robuboard; print(is_robuboard())" | grep -q "True"; then
    echo "Robuboard detected! Starting Robuboard services..."
    
    if [[ -d "$DIR_ROBOCUP_TEENSY" ]]; then
        echo "checking repository robocup24-teensy..."
        cd $DIR_ROBOCUP_TEENSY
        if ! (git pull | grep -q 'Already up to date.'); then
            echo "building new firmware for teensy..."
            ros2 run robuboard build_upload_firmware_teensy
        fi
    fi

    ros2 run robuboard powerswitch &
    ros2 launch robuboard set_status_led.launch.py
fi

#go to home directory
cd

if /usr/bin/python3 -c "from robuboard.rpi.utils import is_raspberry_pi; print(is_raspberry_pi())" | grep -q "True"; then
    /usr/bin/python3 $ROBU_RPI_WS/autostart/ros_launcher.py
fi

