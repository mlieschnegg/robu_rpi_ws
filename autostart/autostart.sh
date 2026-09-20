#!/bin/bash
# Load all code before fetch/reset can replace the running script.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/update_robu.sh" || exit 1

main() {

export DISPLAY=:0
export ROBU_RPI_WS=$HOME/work/.robu
#export ROBU_RPI_WS=$HOME/work/ROBU/robu_rpi_ws

DIR_MICRO_ROS="$HOME/work/microros_ws"
DIR_ROBOCUP_ROS="$HOME/work/robocup/robocup-ros"
DIR_ROBOCUP_TEENSY="$HOME/work/robocup/robocup-teensy"
DIR_ROBOCUP_GUI="$HOME/work/robocup/robocup-gui"

# Keep the lock outside the repository so it survives a repair/reclone.
local state_dir="${XDG_STATE_HOME:-$HOME/.local/state}/robu-autostart"
mkdir -p "$state_dir" || return 1
exec 9>"$state_dir/update.lock" || return 1
flock -n 9 || { echo "[ROBU] Another autostart is already running."; return 0; }

#display is always on
xset s off
xset -dpms
xset s noblank

# Load ROS before building; do not depend on interactive shell startup files.
source /opt/ros/jazzy/setup.bash || return 1
if ! robu_update_and_build "$ROBU_RPI_WS" \
    "https://github.com/mlieschnegg/robu_rpi_ws.git" "$state_dir" \
    > "$state_dir/last-update.log" 2>&1; then
    cat "$state_dir/last-update.log"
    echo "[ROBU] Update/build failed; see $state_dir/last-update.log"
    return 1
fi
cat "$state_dir/last-update.log"
[[ -f "$ROBU_RPI_WS/install/setup.bash" ]] || {
    echo "[ROBU] No usable installation available."
    return 1
}

# Prüfen, ob der Prozess "screen" existiert
#if pgrep screen > /dev/null; then
#    killall screen
#fi
#killall -9 /usr/bin/python3

source /opt/ros/jazzy/setup.bash
source "$ROBU_RPI_WS/install/setup.bash" || return 1

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

#    ros2 run robuboard powerswitch &
#    ros2 launch robuboard set_status_led.launch.py
fi

#go to home directory
cd

if /usr/bin/python3 -c "from robuboard.rpi.utils import is_raspberry_pi; print(is_raspberry_pi())" | grep -q "True"; then
    ros2 run ssd1306 connection_display
#    /usr/bin/python3 $ROBU_RPI_WS/autostart/ros_launcher.py
fi

sudo cp $ROBU_RPI_WS/config/config.txt /boot/firmware/config.txt
sudo rm -rf ~/work/robu_lab*/
sync
}

main "$@"
