from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ===== Arguments =====
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="kafeiche_description",
            description="Package with robot description (URDF/XACRO).",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="kafeiche_base.xacro",
            description="Xacro file with the robot model.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix for joint names (optional).",
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")

    # ===== Generate URDF using xacro =====
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "prefix:=", prefix,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # ===== ROS2 Control node =====
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution([
                FindPackageShare("kafeiche_drivers"),
                "config",
                "controller.yaml"
            ]),
        ],
        output="both",
    )

    # ===== Spawners =====

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # ===== Hardware node (servo motors) =====
    servo_motor_node = Node(
        package='kafeiche_drivers',
        executable='servo_motor',
        output='screen',
    )

    # ===== Return launch description =====
    return LaunchDescription(
        declared_arguments
        + [
            ros2_control_node,
            joint_state_broadcaster_spawner,
            diff_drive_spawner,
            servo_motor_node,
        ]
    )
