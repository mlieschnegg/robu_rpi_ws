#!/bin/bash

#set -e

echo "Installing git and cloning repositories"
echo "Please wait..."

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/rpi_detect.sh"

ROBU_RPI_WS="$HOME/work/.robu"
ROBOCUP_DIR="$HOME/work/robocup"
ROBOCUP_ROS_WS="$ROBOCUP_DIR/robocup-ros"
ROBOCUP_TEENSY_WS="$ROBOCUP_DIR/robocup-teensy"
ROBOCUP_GUI_WS="$ROBOCUP_DIR/robocup-gui"
PLAYGROUND_TEENSY_WS="$ROBOCUP_DIR/playground-teensy"

append_once() {
    local line="$1"
    if ! grep -Fqx "$line" "$HOME/.bashrc" 2>/dev/null; then
        echo "$line" >> "$HOME/.bashrc"
    fi
}

clone_if_missing() {
    local repo_url="$1"
    local target_dir="$2"

    if [ ! -d "$target_dir/.git" ]; then
        git clone "$repo_url" "$target_dir"
    fi
}

setup_autostart() {
    mkdir -p "$HOME/.config/autostart"

    cat > "$HOME/.config/autostart/robu.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=ROBU-RPI-Autostart
Comment=Startet mein Skript beim Anmelden
Exec=bash -c "$HOME/work/.robu/autostart/autostart.sh"
Terminal=false
X-KDE-Autostart-enabled=true
X-GNOME-Autostart-enabled=true
StartupNotify=false
EOF
}

ensure_github_auth() {
    if gh auth status &>/dev/null; then
        return
    fi

    if [ -f "$HOME/.gh_token" ]; then
        gh auth login --with-token < "$HOME/.gh_token"
    else
        echo "GitHub authentication required. Run: gh auth login"
        exit 1
    fi
}

install_packages() {
    sudo apt install -y git gh
}

clone_repositories() {
    mkdir -p "$HOME/work" "$ROBOCUP_DIR"

    clone_if_missing https://github.com/mlieschnegg/robu_rpi_ws "$ROBU_RPI_WS"
    clone_if_missing https://github.com/mlieschnegg/robocup24-ros "$ROBOCUP_ROS_WS"
    clone_if_missing https://github.com/mlieschnegg/robocup24-teensy "$ROBOCUP_TEENSY_WS"
    clone_if_missing https://github.com/mlieschnegg/robocup24-gui "$ROBOCUP_GUI_WS"
    clone_if_missing https://github.com/mlieschnegg/teensy "$PLAYGROUND_TEENSY_WS"
}

setup_shell_environment() {
    append_once '[ -f "$HOME/work/.robu/install/setup.bash" ] && source "$HOME/work/.robu/install/setup.bash"'
}

install_raspberry_pi_compiler_fix() {
    if is_raspberry_pi; then
        sudo apt install -y --reinstall libstdc++-13-dev g++-13
    fi
}

build_ros_workspace_if_possible() {
    if [ ! -d "$ROBOCUP_ROS_WS" ]; then
        return
    fi

    if ! command -v colcon >/dev/null 2>&1; then
        echo "Skipping ROS workspace build: colcon not found. Run ros_setup.sh first."
        return
    fi

    if [ -z "${ROS_DISTRO:-}" ] || [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        echo "Skipping ROS workspace build: ROS environment is not active."
        return
    fi

    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    cd "$ROBOCUP_ROS_WS" || exit 1
    colcon build
    source install/local_setup.bash
}

build_teensy_workspace_if_possible() {
    if [ ! -d "$ROBOCUP_TEENSY_WS" ]; then
        return
    fi

    if ! command -v pio >/dev/null 2>&1; then
        echo "Skipping Teensy build: pio not found. Install PlatformIO first."
        return
    fi

    cd "$ROBOCUP_TEENSY_WS" || exit 1
    pio run
}

main() {
    install_packages
    ensure_github_auth
    clone_repositories
    setup_autostart
    setup_shell_environment
    install_raspberry_pi_compiler_fix
    build_ros_workspace_if_possible
    build_teensy_workspace_if_possible

    echo "Git setup completed."
}

main "$@"
