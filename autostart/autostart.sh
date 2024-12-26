#set -e #Beendet das Skript bei Fehlern

export ROBU_RPI_WS=/home/robu/work/robu_rpi_ws
export DISPLAY=:0

cd $ROBU_RPI_WS
git pull
cd

# Prüfen, ob der Prozess "screen" existiert
#if pgrep screen > /dev/null; then
#    killall screen
#fi
#killall -9 /usr/bin/python3

source /opt/ros/humble/setup.bash

/usr/bin/python3 $ROBU_RPI_WS/autostart/autostart.py
