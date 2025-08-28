
export ROBU_RPI_WS=~/work/.robu

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
