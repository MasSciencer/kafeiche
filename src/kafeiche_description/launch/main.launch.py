from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ── Аргументы запуска ───────────────────────────────────────────────
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="kafeiche_description",
            description="Description package with robot URDF/xacro files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="kafeiche_base.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup.",
        )
    )

    # ── Подстановки ─────────────────────────────────────────────────────
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")

    # Получаем robot_description через xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # ── Пути к конфигам ─────────────────────────────────────────────────
    controller_config = PathJoinSubstitution(
        [FindPackageShare("kafeiche_drivers"), "config", "controller.yaml"]
    )

    twist_mux_config = PathJoinSubstitution(
        [FindPackageShare("kafeiche_drivers"), "config", "priority.yaml"]
    )

    # ── Основные ноды ───────────────────────────────────────────────────
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="screen",
        remappings=[
            ('/diff_controller/cmd_vel', '/cmd_vel'),
            ('/diff_controller/cmd_vel_unstamped', '/cmd_vel'),
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    diff_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller"],
        output="screen",
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[
            twist_mux_config,
        ],
        remappings=[
            ("cmd_vel_out", "/cmd_vel"),
        ],
    )
    servo_motor_node = Node(
        package='kafeiche_drivers',
        executable='servo_motor',
        output='screen',
    )
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
    )
    start_rosbridge_after_servo = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=servo_motor_node,
            on_start=[rosbridge_node]
        )
    )

    # ── Сборка запуска ──────────────────────────────────────────────────
    return LaunchDescription(
        declared_arguments + [
            # Основные ноды
            control_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            diff_controller_spawner,
            twist_mux_node,
            servo_motor_node,
            # rosbridge после servo_motor
            start_rosbridge_after_servo,
        ]
    )