#!/bin/bash

#set -e

echo "Starting Setup for RpackagesOBU"

source "rpi_detect.sh"

mkdir -p ~/work

sudo sed -i -E 's/^XKBLAYOUT="[^"]*"/XKBLAYOUT="de"/' /etc/default/keyboard

#######################################
# Services
#######################################

# SSH-Setup
sudo apt install -y openssh-server
sudo systemctl enable --now ssh
# systemctl status ssh --no-pager

# Samba-Setup
. samba_setup.sh

###########################################################
# apt
##########################################################

# Install additional utilities
sudo apt install -y git terminator screen htop 
sudo apt install -y rpi-imager
sudo apt install -y ncdu 
sudo apt install -y python3-pip
sudo apt install -y python3-opencv
sudo apt install -y v4l-utils
sudo apt-get install -y p7zip-full
#sudo apt install libcamera-apps

#compiler for python
/bin/bash -c "$(curl -fsSL https://exaloop.io/install.sh)"

############################################################
# pip
############################################################
#Gehe nur am Raspberry, wir können jedoch die Autovervollständigung für eine Entwicklung am PC nutzen
pip install rpi_ws281x --break-system-packages
#Erst nach dieser Installation geht die Autovervollständiguung in VSC
pip install opencv-python --break-system-packages
#Bekomme sonst Fehlermeldung beim ros2-jazzy-tf-transformations Paket -> wahrscheinlich geht dann opencv nicht :-(
pip install "numpy<2.0"  --break-system-packages
pip install pillow --break-system-packages
#pip install dynamixel-sdk --break-system-packages

if is_raspberry_pi; then
    pip install smbus2 --break-system-packages
    pip install RPi.GPIO --break-system-packages
    pip install ST7789 --break-system-packages
    pip install numpy --break-system-packages
    pip install git+https://github.com/sparkfun/Qwiic_VL53L5CX_Py.git --break-system-packages
fi

#############################################################
# PC-Only
#############################################################
if ! is_raspberry_pi; then
#    sudo apt install -y blender kde-plasma-desktop
    sudo add-apt-repository --yes ppa:kicad/kicad-9.0-releases
    sudo apt install --install-recommends kicad

    # Install snap packages
    #------------------------------
    sudo snap install gimp
    #VSCode with
    sudo snap install code --classic
    # Extensions for VS-Code
    code --install-extension platformio.platformio-ide
    code --install-extension ms-vscode-remote.remote-ssh
    code --install-extension ms-vscode-remote.remote-ssh-edit
    code --install-extension ms-vscode.remote-explorer
    code --install-extension ranch-hand-robotics.rde-pack
    code --install-extension ranch-hand-robotics.rde-ros-2
    code --install-extension ranch-hand-robotics.urdf-editor
    code --install-extension pdconsec.vscode-print

    #Symlinks für VSC erstellen (Code Snippets)
    . link_vsc_snippets

    #Desktop Icons kopieren
    cp -r ~/work/.robu/config/desktop/*.* $HOME/Desktop
fi

. git_setup.sh

. ros_setup.sh

. microros_setup.sh

. machine_learning_setup.sh

####################################################################
# gnome-extensions
####################################################################
# gnome-extensions list
# gnome-shell --version
# gnome-extensions info gnome-extension-all-ip-addresses@havekes.eu
# TODO: gnome-extensions install config/extensions/gnome-extension-all-ip-addresseshavekes.eu.v12.shell-extension.zip

####################################################################
# OS-Settings
####################################################################

# deactivate password for sudo
echo "$USER ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/$USER

# TODO
#sudo hostnamectl set-hostname robu-desktop
#sudo nano /etc/hosts

# Symlinks für colcon erstellen
mkdir -p ~/.colcon
ln -sf ~/work/.robu/config/.colcon/defaults.yaml ~/.colcon/defaults.yaml


#Automatischen Login nach dem Booten bei der Desktop-Version aktivieren
# TODO
# sudo nano /etc/gdm3/custom.conf
# [daemon]
# AutomaticLoginEnable=true
# AutomaticLogin=dein_benutzername

#Benutzer zur Gruppe dialout hinzufügen (z.B. serielle Schnittstelle)
sudo adduser $USER dialout
sudo adduser $USER video
sudo adduser $USER kmem

###################################################################
# raspberry config
###################################################################
if is_raspberry_pi; then

    #sudo adduser $USER gpio -> gibt es eventuell nicht, Gruppe erstellen plus udev rule hinzufügen
    sudo chown root:$USER /dev/gpiomem

    #Zugriffsrechte hinzufügen (gehen nach dem Neustart verloren)
    sudo chmod a+rw /dev/ttyACM0
    sudo chmod a+rw /dev/gpiomem
    sudo chmod a+rw /dev/i2c-1
    sudo chmod a+rw /dev/video0
    sudo chmod a+rw /dev/ttyUSB0
    sudo chmod a+rw /dev/mem

fi

#pishrink installieren
cd ~/work/
wget https://raw.githubusercontent.com/Drewsif/PiShrink/master/pishrink.sh
sudo chmod +x pishrink.sh
sudo mv pishrink.sh /usr/local/bin

echo "Setup completed successfully!"
