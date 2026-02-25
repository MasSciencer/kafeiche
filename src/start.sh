#!/usr/bin/env bash

# Терминал 1 — ROS2 + rviz/модель
gnome-terminal --tab --title="KAFEICHE VR" -- bash -c "
    cd ~/ros_ws &&
    source /opt/ros/jazzy/setup.bash &&
    source install/setup.bash &&
    ros2 launch kafeiche_description vr.launch.py;
    exec bash"

# Терминал 2 — MediaMTX
gnome-terminal --tab --title="MediaMTX" -- bash -c "
    cd ~/mediamtx &&
    ./mediamtx;
    exec bash"