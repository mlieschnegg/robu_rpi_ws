#!/bin/bash

#set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/rpi_detect.sh"

WORK_DIR="$HOME/work"
CAMERA_WS="$WORK_DIR/camera_ws"

require_raspberry_pi() {
    if ! is_raspberry_pi; then
        echo "This script is only for Raspberry Pi"
        exit 0
    fi
}

require_ros() {
    if [ -z "${ROS_DISTRO:-}" ]; then
        echo "ROS_DISTRO is not set. Run ros_setup.sh first, then open a new shell or source /opt/ros/<distro>/setup.bash."
        exit 1
    fi

    if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
        echo "Missing ROS installation: /opt/ros/$ROS_DISTRO/setup.bash"
        exit 1
    fi

    for cmd in git meson ninja colcon rosdep; do
        if ! command -v "$cmd" >/dev/null 2>&1; then
            echo "Missing required command: $cmd"
            exit 1
        fi
    done
}

append_once() {
    local line="$1"
    if ! grep -Fqx "$line" "$HOME/.bashrc" 2>/dev/null; then
        echo "$line" >> "$HOME/.bashrc"
    fi
}

install_build_dependencies() {
    sudo apt install -y \
        python3-pip git python3-jinja2 python3-yaml python3-ply \
        libboost-dev libgnutls28-dev openssl libtiff5-dev pybind11-dev \
        qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5 \
        meson cmake ninja-build \
        libglib2.0-dev libgstreamer-plugins-base1.0-dev \
        libboost-program-options-dev libdrm-dev libexif-dev \
        ffmpeg libsdl2-2.0-0 adb wget gcc pkg-config \
        libsdl2-dev libavcodec-dev libavdevice-dev libavformat-dev \
        libavutil-dev libswresample-dev libusb-1.0-0 libusb-1.0-0-dev \
        libepoxy-dev libegl1-mesa-dev python3-colcon-meson

    python3 -m pip install --user meson
}

clone_or_update_repo() {
    local repo_url="$1"
    local target_dir="$2"

    if [ ! -d "$target_dir/.git" ]; then
        git clone "$repo_url" "$target_dir"
    fi
}

build_libcamera() {
    cd "$WORK_DIR" || exit 1
    clone_or_update_repo https://github.com/raspberrypi/libcamera.git "$WORK_DIR/libcamera"
    cd "$WORK_DIR/libcamera" || exit 1

    meson setup build \
        --buildtype=release \
        -Dpipelines=rpi/vc4,rpi/pisp \
        -Dipas=rpi/vc4,rpi/pisp \
        -Dv4l2=enabled \
        -Dgstreamer=enabled \
        -Dtest=false \
        -Dlc-compliance=disabled \
        -Dcam=disabled \
        -Dqcam=disabled \
        -Ddocumentation=disabled \
        -Dpycamera=enabled || true

    ninja -C build -j 1
    sudo ninja -C build install
    sudo ldconfig
}

build_camera_ros() {
    mkdir -p "$CAMERA_WS/src"
    cd "$CAMERA_WS/src" || exit 1
    clone_or_update_repo https://github.com/christianrauch/camera_ros.git "$CAMERA_WS/src/camera_ros"

    source "/opt/ros/$ROS_DISTRO/setup.bash"
    cd "$CAMERA_WS" || exit 1
    rosdep install -y --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" --skip-keys=libcamera
    colcon build --event-handlers=console_direct+

    append_once 'source ~/work/camera_ws/install/setup.bash'
}

build_rpicam_apps() {
    cd "$WORK_DIR" || exit 1
    clone_or_update_repo https://github.com/raspberrypi/rpicam-apps.git "$WORK_DIR/rpicam-apps"
    cd "$WORK_DIR/rpicam-apps" || exit 1

    meson setup build \
        -Denable_libav=disabled \
        -Denable_drm=enabled \
        -Denable_egl=enabled \
        -Denable_qt=enabled \
        -Denable_opencv=enabled \
        -Denable_tflite=disabled || true

    meson compile -C build -j 1
    sudo meson install -C build
    sudo ldconfig
}

build_libepoxy() {
    cd "$WORK_DIR" || exit 1
    clone_or_update_repo https://github.com/anholt/libepoxy.git "$WORK_DIR/libepoxy"
    cd "$WORK_DIR/libepoxy" || exit 1

    meson setup build || true
    ninja -C build -j 1
    sudo ninja -C build install
}

setup_gstreamer() {
    sudo apt install -y gstreamer1.0-plugins-bad
    append_once 'export GST_PLUGIN_PATH=/usr/local/lib/aarch64-linux-gnu/'
}

main() {
    require_raspberry_pi
    require_ros
    mkdir -p "$WORK_DIR"

    install_build_dependencies
    build_libcamera
    build_camera_ros
    build_rpicam_apps
    build_libepoxy
    setup_gstreamer

    echo "Raspberry Pi camera setup completed."
}

main "$@"
