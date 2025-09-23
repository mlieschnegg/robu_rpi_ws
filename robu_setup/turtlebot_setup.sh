echo "Installation der Simulations-Unterstützung für den Turtlebot 3"

source "/pfad/zu/rpi_detect.sh"

ubuntu_version=$(lsb_release -rs)
if [ "$ubuntu_version" = "22.04" ]; then
    source /opt/ros/humble/setup.bash
elif [ "$ubuntu_version" = "24.04" ]; then
    source /opt/ros/jazzy/setup.bash
fi

source /opt/ros/${ROS_DISTRO}/setup.bash

sudo apt install -y ros-${ROS_DISTRO}-cartographer
sudo apt install -y ros-${ROS_DISTRO}-cartographer-ros
sudo apt install -y ros-${ROS_DISTRO}-navigation2
sudo apt install -y ros-${ROS_DISTRO}-nav2-bringup
sudo apt install -y ros-${ROS_DISTRO}-dynamixel-sdk
sudo apt install -y ros-${ROS_DISTRO}-turtlebot3-msgs 
sudo apt install -y ros-${ROS_DISTRO}-xacro

sudo apt install -y python3-colcon-common-extensions

mkdir -p ~/work/turtlebot3_ws/src && cd ~/work/turtlebot3_ws/src

git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/turtlebot3.git

if ! is_raspberry_pi; then
    sudo apt-get update
    sudo apt-get install -y curl lsb-release gnupg
    sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    sudo apt-get update
    sudo apt-get install gz-harmonic

    git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

    cd ~/work/turtlebot3_ws && colcon build --symlink-install
else
    # sudo nano /etc/apt/apt.conf.d/20auto-upgrades
    # APT::Periodic::Update-Package-Lists "0";
    # APT::Periodic::Unattended-Upgrade "0";

    #prevent boot-up delay even if there is no network at startup
    systemctl mask systemd-networkd-wait-online.service
    #Disable Suspend and Hibernation
    sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

    sudo apt install -y python3-argcomplete libboost-system-dev build-essential
    sudo apt install -y ros-${ROS_DISTRO}-hls-lfcd-lds-driver
    sudo apt install -y libudev-dev

    git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/ld08_driver.git
    git clone -b ${ROS_DISTRO} https://github.com/ROBOTIS-GIT/coin_d4_driver

    cd ~/work/turtlebot3_ws/ && colcon build --symlink-install --parallel-workers 1

    sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger

    CMD_TEXT="echo 'export LDS_MODEL=LDS-02'"
    if ! grep -q "$CMD_TEXT" ~/.bashrc; then
        echo "$CMD_TEXT" >> ~/.bashrc
    fi

    sudo dpkg --add-architecture armhf  
    sudo apt-get update  
    sudo apt-get install libc6:armhf

    export OPENCR_PORT=/dev/ttyACM0  
    export OPENCR_MODEL=burger
    cd ~/work/turtlebot3_ws/ 
    rm -rf ./opencr_update.tar.bz2 

    wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2   
    tar -xvf opencr_update.tar.bz2 

    cd ./opencr_update  
    ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr  
fi


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
