# launch/mediamtx.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Запускаем MediaMTX
        ExecuteProcess(
            cmd=['/usr/local/bin/mediamtx', '/etc/mediamtx/mediamtx.yml'],
            output='screen',
            respawn=True,
        ),
        # Просто держим ноду живой и печатаем подсказку
        Node(
            package="kafeiche_webrtc_cam",
            executable="cam_webrtc_node",
            name="cam_webrtc_node"
        ),
    ])