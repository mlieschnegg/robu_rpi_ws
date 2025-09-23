#!/bin/bash
source "/pfad/zu/rpi_detect.sh"

if is_raspberry_pi; then

    #################################
    # libcamera
    #################################

    #Librarys

    sudo apt install -y python3-pip git python3-jinja2

    sudo apt install -y libboost-dev
    sudo apt install -y libgnutls28-dev openssl libtiff5-dev pybind11-dev
    sudo apt install -y qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5
    sudo apt install -y meson cmake
    sudo apt install -y python3-yaml python3-ply
    sudo apt install -y libglib2.0-dev libgstreamer-plugins-base1.0-dev

    sudo pip3 install meson

    #Libcamera
    cd ~/work/
    git clone https://github.com/raspberrypi/libcamera.git
    cd libcamera

    meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
    ninja -C build -j 1
    sudo ninja -C build install
    sudo ldconfig

    ########################################################
    # camera_ros
    # see: https://github.com/christianrauch/camera_ros
    ########################################################

    # create workspace
    mkdir -p ~/work/camera_ws/src
    cd ~/work/camera_ws/src

    # # check out libcamera
    # sudo apt -y install python3-colcon-meson
    # git clone https://github.com/raspberrypi/libcamera.git

    # check out this camera_ros repository
    git clone https://github.com/christianrauch/camera_ros.git

    # resolve binary dependencies and build workspace
    source /opt/ros/$ROS_DISTRO/setup.bash
    cd ~/work/camera_ws/
    rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera
    colcon build --event-handlers=console_direct+

    # Test der Camera:
    # ros2 run camera_ros camera_node
    # oder mit GUI-Fenster
    # export DISPLAY=0
    # xhost +SI:localuser:$(whoami)
    # ros2 launch camera_ros camera.launch.py
    CMD_TEXT="source ~/work/camera_ws/install/setup.bash"
    if ! grep -q "$CMD_TEXT" ~/.bashrc; then
        echo "$CMD_TEXT" >> ~/.bashrc
    fi


    ################################################
    # rpicam-apps
    ################################################

    # librarys
    sudo apt install -y cmake libboost-program-options-dev libdrm-dev libexif-dev
    sudo apt install -y meson ninja-build
    sudo apt install -y ffmpeg libsdl2-2.0-0 adb wget gcc pkg-config meson libsdl2-dev libavcodec-dev libavdevice-dev libavformat-dev libavutil-dev libswresample-dev libusb-1.0-0 libusb-1.0-0-dev
    sudo apt install -y libepoxy-dev

    #rpicam-apps
    git clone https://github.com/raspberrypi/rpicam-apps.git
    cd rpicam-apps

    meson setup build -Denable_libav=enabled -Denable_drm=enabled -Denable_egl=enabled -Denable_qt=enabled -Denable_opencv=enabled -Denable_tflite=disabled
    meson compile -C build -j 1

    sudo meson install -C build
    sudo ldconfig
    rpicam-still --version

    #####################################################
    # libepoxy
    #####################################################

    sudo apt install -y libegl1-mesa-dev

    cd ~/work/
    git clone https://github.com/anholt/libepoxy.git
    cd libepoxy
    meson setup build
    ninja -j 1
    sudo ninja install

    # ####################################################
    # # libcamera-apps
    # ####################################################

    # cd ~/work/
    # git clone https://github.com/raspberrypi/libcamera-apps.git
    # cd libcamera-apps

    # meson setup build -Denable_opencv=enabled
    # ninja -j 1
    # sudo meson install -C build
    # sudo ldconfig

    ####################################################
    # gstreamer
    ####################################################

    sudo apt-get install gstreamer1.0-plugins-bad
        
    CMD_TEXT="echo export GST_PLUGIN_PATH=/usr/local/lib/aarch64-linux-gnu/"
    if ! grep -q "$CMD_TEXT" ~/.bashrc; then
        echo "$CMD_TEXT" >> ~/.bashrc
    fi

    # CMD_TEXT="dtoverlay=imx219"
    # if ! grep -q "$CMD_TEXT" /boot/firmware/config.txt; then
    #     echo "$CMD_TEXT" >> /boot/firmware/config.txt
    # fi

fi