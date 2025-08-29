echo "Installation der Simulations-Unterstützung für den Turtlebot 3"

ubuntu_version=$(lsb_release -rs)
if [ "$ubuntu_version" = "22.04" ]; then
    source /opt/ros/humble/setup.bash
elif [ "$ubuntu_version" = "24.04" ]; then
    source /opt/ros/jazzy/setup.bash
fi

sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
sudo apt install -y ros-${ROS_DISTRO}-cartographer
sudo apt install -y ros-${ROS_DISTRO}-cartographer-ros
sudo apt install -y ros-${ROS_DISTRO}-navigation2
sudo apt install -y ros-${ROS_DISTRO}-nav2-bringup

source /opt/ros/${ROS_DISTRO}/setup.bash

mkdir -p ~/work/turtlebot3_ws/src
cd ~/work/turtlebot3_ws/src/
#git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/turtlebot3.git

sudo apt install -y python3-colcon-common-extensions

cd ~/work/turtlebot3_ws
colcon build --symlink-install

CMD_TEXT="source /opt/ros/${ROS_DISTRO}/setup.bash"
if ! grep -q "$CMD_TEXT" ~/.bashrc; then
    echo "$CMD_TEXT" >> ~/.bashrc
fi

CMD_TEXT="source ~/work/turtlebot3_ws/install/setup.bash"
if ! grep -q "$CMD_TEXT" ~/.bashrc; then
    echo "$CMD_TEXT" >> ~/.bashrc
fi

CMD_TEXT="export TURTLEBOT3_MODEL=burger"
if ! grep -q "$CMD_TEXT" ~/.bashrc; then
    echo "$CMD_TEXT" >> ~/.bashrc
fi

CMD_TEXT="RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
if ! grep -q "$CMD_TEXT" ~/.bashrc; then
    echo "$CMD_TEXT" >> ~/.bashrc
fi

CMD_TEXT="export ROS_DOMAIN_ID=30 #TURTLEBOT3"
if ! grep -q "$CMD_TEXT" ~/.bashrc; then
    echo "$CMD_TEXT" >> ~/.bashrc
fi

source ~/.bashrc

cd ~/work/turtlebot3_ws/src/
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/work/turtlebot3_ws && colcon build --symlink-install