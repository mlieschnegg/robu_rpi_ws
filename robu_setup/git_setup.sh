
echo "Installation von git und richten Repositories"
echo "Bitte warten..."

export ROBU_RPI_WS=~/work/.robu
export ROBOCUP_DIR=~/work/robocup
export ROBOCUP_ROS_WS=$ROBOCUP_DIR/robocup-ros
export ROBOCUP_TEENSY_WS=$ROBOCUP_DIR/robocup-teensy
export ROBOCUP_GUI_WS=$ROBOCUP_DIR/robocup-gui
export PLAYGROUND_TEENSY_WS=$ROBOCUP_DIR/playground-teensy

mkdir -p ~/work

sudo apt install -y git gh

if [[ ! -d "$ROBU_RPI_WS" ]]; then
    git clone https://github.com/mlieschnegg/robu_rpi_ws ~/work/.robu
fi


mkdir -p ~/.config/autostart
cat > ~/.config/autostart/robu.desktop <<EOF
[Desktop Entry]
Type=Application
Name=ROBU-RPI-Autostart
Comment=Startet mein Skript beim Anmelden
Exec=bash -c "$HOME/work/.robu/autostart/autostart.sh"
Terminal=false
#OnlyShowIn=KDE;
X-KDE-Autostart-enabled=true
X-GNOME-Autostart-enabled=true
StartupNotify=false
EOF

if ! grep -q "source $HOME/work/.robu/install/setup.bash" ~/.bashrc; then
    echo "source $HOME/work/.robu/install/setup.bash" >> ~/.bashrc
fi

if ! gh auth status &>/dev/null; then
    echo "GitHub authentication required. Please log in using 'gh auth login'."
    exit 1
fi

%gh auth login --with-token < ~/.gh_token

if is_raspberry_pi; then
    # Zum Kompilieren des Teensy Codes wird g++-13 benötigt
    # Ich musste dieses erneut installieren, da sonst Abhängigkeit map, etc. fehlten
    sudo apt install -y --reinstall libstdc++-13-dev g++-13
fi

mkdir -p $ROBOCUP_DIR

if [[ ! -d "$ROBOCUP_ROS_WS" ]]; then
    git clone https://github.com/mlieschnegg/robocup24-ros ROBOCUP_ROS_WS
fi

if [[ ! -d "$ROBOCUP_TEENSY_WS" ]]; then
    git clone https://github.com/mlieschnegg/robocup24-teensy ROBOCUP_TEENSY_WS
fi

if [[ ! -d "$ROBOCUP_GUI_WS" ]]; then
    git clone https://github.com/mlieschnegg/robocup24-gui ROBOCUP_GUI_WS
fi

if [[ ! -d "$PLAYGROUND_TEENSY_WS" ]]; then
    git clone https://github.com/mlieschnegg/teensy PLAYGROUND_TEENSY_WS
fi

cd $ROBOCUP_ROS_WS
colcon build
source install/local_setup.bash

cd $ROBOCUP_TEENSY_WS
pio run


# # Rust Setup - https://github.com/ros2-rust/ros2_rust
# sudo apt install -y git libclang-dev python3-pip python3-vcstool
# pip install git+https://github.com/colcon/colcon-cargo.git
# pip install git+https://github.com/colcon/colcon-ros-cargo.git

# cd $ROBOCUP_GUI_WS
# rm -rf src/ros2_rust
# mkdir -p src
# git clone https://github.com/ros2-rust/ros2_rust.git src/ros2_rust
# vcs import src < src/ros2_rust/ros2_rust_${ROS_DISTRO}.repos
# . /opt/ros/${ROS_DISTRO}/setup.sh
# colcon build


