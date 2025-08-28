# Update and upgrade system
sudo apt update && sudo apt upgrade -y

# Set locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
#BUGFIX: fixing move-it issue and set LC_NUMERIC=en_US.UTF-8 -> https://github.com/moveit/moveit2/issues/1782
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 LC_NUMERIC=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt install ros-dev-tools

# Install ROS 2 Jazzy
sudo apt update
sudo apt upgrade

sudo apt install -y ros-dev-tools ros-jazzy-desktop-full

# Add ROS 2 to bashrc for persistence
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

source ~/.bashrc

sudo apt install -y ros-${ROS_DISTRO}-ros2-control
sudo apt install -y ros-${ROS_DISTRO}-ros2-controllers
sudo apt install -y ros-${ROS_DISTRO}-*-ros2-control
sudo apt install -y ros-${ROS_DISTRO}-ros-gz
sudo apt install -y ros-${ROS_DISTRO}-dynamixel-sdk
sudo apt install -y ros-${ROS_DISTRO}-dynamixel-interfaces
sudo apt install -y python3-colcon-clean
sudo apt install -y ros-${ROS_DISTRO}-moveit
sudo apt install -y ros-${ROS_DISTRO}-tf-transformations
