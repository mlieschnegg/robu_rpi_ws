export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=burger
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
export GST_PLUGIN_PATH=/usr/local/lib/aarch64-linux-gnu/
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/robu/work/robu_rpi_ws/config/fastdds.xml

source /opt/ros/humble/setup.bash
source /home/robu/work/turtlebot3_ws/install/setup.bash
source /home/robu/work/microros_ws/install/local_setup.bash
source /home/robu/work/robocup-ros/install/setup.bash
source /home/robu/work/robocup-gui/install/setup.bash
