echo "Installation von microros"

# https://micro.ros.org/docs/tutorials/core/teensy_with_arduino/

# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

MICROROS_WS_PATH=~/work/microros_ws
cd ~/work/

# Create a workspace and download the micro-ROS tools
mkdir -p $MICROROS_WS_PATH
cd $MICROROS_WS_PATH
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
# Update dependencies using rosdep
sudo apt update
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y
# Install pip
sudo apt-get install -y python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash

# Download micro-ROS agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build step
ros2 run micro_ros_setup build_agent.sh
#source install/local_setup.bash
CMD_TEXT="source ~/work/microros_ws/install/local_setup.bash"
if ! grep -q "$CMD_TEXT" ~/.bashrc; then
    echo "$CMD_TEXT" >> ~/.bashrc
fi
source ~/.bashrc

# test agent
# ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

#Download the Linux udev rules and copy the file to /etc/udev/rules.d.
curl https://www.pjrc.com/teensy/00-teensy.rules -O
sudo cp 00-teensy.rules /etc/udev/rules.d/
rm 00-teensy.rules