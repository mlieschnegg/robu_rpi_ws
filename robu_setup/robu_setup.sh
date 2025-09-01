#!/bin/bash

#set -e

echo "Starting Setup for ROBU"

mkdir -p ~/work

sudo sed -i -E 's/^XKBLAYOUT="[^"]*"/XKBLAYOUT="de"/' /etc/default/keyboard

. ros_setup.sh
. microros_setup.sh

# Install additional utilities
sudo apt install -y git terminator screen htop kde-plasma-desktop
sudo apt install -y ncdu blender
sudo snap install gimp

. samba_setup.sh

sudo apt install -y python3-pip
sudo apt install -y python3-opencv

#Gehe nur am Raspberry, wir können jedoch die Autovervollständigung für eine Entwicklung am PC nutzen
pip install rpi_ws281x --break-system-packages
#Erst nach dieser Installation geht die Autovervollständiguung in VSC
pip install opencv-python --break-system-packages
#Bekomme sonst Fehlermeldung beim ros2-jazzy-tf-transformations Paket -> wahrscheinlich geht dann opencv nicht :-(
pip install "numpy<2.0"  --break-system-packages

# Install snap packages
sudo snap install code --classic

# Setup for VS-Code
code --install-extension platformio.platformio-ide
code --install-extension ms-vscode-remote.remote-ssh
code --install-extension ms-vscode-remote.remote-ssh-edit
code --install-extension ms-vscode.remote-explorer
code --install-extension ranch-hand-robotics.rde-pack
code --install-extension ranch-hand-robotics.rde-ros-2
code --install-extension ranch-hand-robotics.urdf-editor
code --install-extension pdconsec.vscode-print


# alessandrosofia.ros2-topic-viewer
# fabianlauer.vs-code-xml-format
# github.copilot
# github.copilot-chat
# hansrobo.ros2-snippets
# ms-ceintl.vscode-language-pack-de
# ms-python.debugpy
# ms-python.python
# ms-python.vscode-pylance
# ms-python.vscode-python-envs
# ms-vscode-remote.remote-ssh
# ms-vscode-remote.remote-ssh-edit
# ms-vscode.cmake-tools
# ms-vscode.cpptools
# ms-vscode.cpptools-extension-pack
# ms-vscode.cpptools-themes
# ms-vscode.remote-explorer
# pdconsec.vscode-print
# platformio.platformio-ide
# ranch-hand-robotics.rde-pack
# ranch-hand-robotics.rde-ros-2
# ranch-hand-robotics.urdf-editor
# redhat.java
# redhat.vscode-xml
# redhat.vscode-yaml
# tomoki1207.pdf
# twxs.cmake
# visualstudioexptteam.intellicode-api-usage-examples
# visualstudioexptteam.vscodeintellicode
# vscjava.vscode-gradle
# vscjava.vscode-java-debug
# vscjava.vscode-java-dependency
# vscjava.vscode-java-pack
# vscjava.vscode-java-test
# vscjava.vscode-maven

# Get rid of password for sudo (use with caution)
echo "$USER ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/$USER

. git_setup.sh

. machine_learning_setup.sh

# TODO
#sudo hostnamectl set-hostname robu-desktop
#sudo nano /etc/hosts
#.colcon ordner kopieren
#websocke istnallieren
#code snippets einrichten...

# Symlinks für colcon erstellen
mkdir -p ~/.colcon
ln -sf ~/work/.robu/config/.colcon/defaults.yaml ~/.colcon/defaults.yaml

#Symlinks für VSC erstellen (Code Snippets)
. link_vsc_snippets

echo "Setup completed successfully!"
