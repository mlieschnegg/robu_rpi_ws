export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=burger
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
export GST_PLUGIN_PATH=/usr/local/lib/aarch64-linux-gnu/
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/robu/work/.robu/config/fastdds.xml

#export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#export CYCLONEDDS_URI=/home/robu/work/.robu/config/cyclonedds.xml
#export CYCLONEDDS_NETWORK_INTERFACE_ADDRESS=192.168.0.234

source /opt/ros/humble/setup.bash
source /home/robu/work/turtlebot3_ws/install/setup.bash
source /home/robu/work/microros_ws/install/local_setup.bash
source /home/robu/work/robocup-ros/install/setup.bash
source /home/robu/work/robocup-gui/install/setup.bash
