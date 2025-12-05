# launch/mediamtx.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="kafeiche_webrtc_cam",
            executable="cam_webrtc_node",
            name="cam_webrtc_node"
        ),
    ])