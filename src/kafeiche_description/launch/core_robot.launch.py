from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = []

    # ── Аргумент управления запуском (вектор) ─────────────────────────
    declared_arguments.append(
        DeclareLaunchArgument(
            "modules",
            default_value="[1, 0, 1, 0, 0]",
            description="Вектор запуска: [Base, Lidar, Teleop, IMU, EKF]. 1 - запуск, 0 - пропуск. Пример: [1,0,0,1,1]",
        )
    )

    # ── Общие аргументы ─────────────────────────────────────────────
    declared_arguments.append(DeclareLaunchArgument("description_package", default_value="kafeiche_description"))
    declared_arguments.append(DeclareLaunchArgument("description_file", default_value="kafeiche_base.xacro"))
    declared_arguments.append(DeclareLaunchArgument("log_level", default_value="warn"))
    declared_arguments.append(DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"))

    # ── Аргументы из лаунчера IMU ───────────────────────────────────
    declared_arguments.append(DeclareLaunchArgument('device', default_value='/dev/i2c-1'))
    declared_arguments.append(DeclareLaunchArgument('address', default_value='41'))
    declared_arguments.append(DeclareLaunchArgument('frame_id', default_value='imu'))
    declared_arguments.append(DeclareLaunchArgument('rate', default_value='50.0'))
    declared_arguments.append(DeclareLaunchArgument('calibration_file', default_value='bno055.json'))

    # Загрузка конфигураций
    modules             = LaunchConfiguration("modules")
    description_package = LaunchConfiguration("description_package")
    description_file    = LaunchConfiguration("description_file")
    log_level           = LaunchConfiguration("log_level")
    use_sim_time        = LaunchConfiguration("use_sim_time")

    # Получение robot_description через xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file])
    ])
    robot_description = {"robot_description": robot_description_content}

    # Пути к конфигурационным файлам
    controller_config = PathJoinSubstitution([FindPackageShare("kafeiche_drivers"), "config", "controller.yaml"])
    twist_mux_config = PathJoinSubstitution([FindPackageShare("kafeiche_drivers"), "config", "priority.yaml"])
    calibration_config = PathJoinSubstitution([FindPackageShare("kafeiche_drivers"), "config", LaunchConfiguration("calibration_file")])
    ekf_config = PathJoinSubstitution([FindPackageShare("kafeiche_drivers"), "config", "ekf.yaml"])

    # ── Условия запуска (на основе парсинга вектора) ────────────────

    base_condition   = IfCondition(PythonExpression(["eval('", modules, "')[0] == 1"]))
    lidar_condition  = IfCondition(PythonExpression(["eval('", modules, "')[1] == 1"]))
    teleop_condition = IfCondition(PythonExpression(["eval('", modules, "')[2] == 1"]))
    imu_condition    = IfCondition(PythonExpression(["eval('", modules, "')[3] == 1"]))
    ekf_condition    = IfCondition(PythonExpression(["eval('", modules, "')[4] == 1"]))

    # ── 0. Базовые узлы (Контроллеры, TF, Rosbridge) ────────────────

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="screen",
        arguments=['--ros-args', '--log-level', log_level],
        condition=base_condition,
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        condition=base_condition,
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", '--ros-args', '--log-level', log_level],
        output="screen",
        condition=base_condition,
    )

    diff_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_controller",
            "--controller-manager", "/controller_manager",
            "--controller-ros-args",
            "--ros-args -r /diff_controller/cmd_vel:=/cmd_vel -r /diff_controller/cmd_vel_unstamped:=/cmd_vel",
        ],
        output="screen",
        condition=base_condition,
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[twist_mux_config, {'use_sim_time': use_sim_time}],
        remappings=[("cmd_vel_out", "/cmd_vel")],
        condition=base_condition,
    )

    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=base_condition,
    )

    # ── 1. RPLiDAR ──────────────────────────────────────────────────

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("rplidar_ros"), "launch", "rplidar_c1_launch.py"])
        ]),
        condition=lidar_condition,
    )

    # ── 2. Teleop Twist Joy ─────────────────────────────────────────

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("teleop_twist_joy"), "launch", "teleop-launch.py"])
        ]),
        launch_arguments={
            "joy_config": "xbox",
            "config_filepath": PathJoinSubstitution([FindPackageShare("kafeiche_drivers"), "config", "xbox_custom.yaml"]),
            "publish_stamped_twist": "true"
        }.items(),
        condition=teleop_condition,
    )

    # ── 3. IMU (BNO055) ─────────────────────────────────────────────

    imu_node = Node(
        package='kafeiche_imu',
        executable='bno055_i2c_node',
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('device'),
            'address': LaunchConfiguration('address'),
            'frame_id': LaunchConfiguration('frame_id'),
            'rate': LaunchConfiguration('rate'),
            'calibration_file': calibration_config,
        }],
        condition=imu_condition,
    )

    # ── 4. EKF (Robot Localization) ─────────────────────────────────

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        condition=ekf_condition,
    )

    # ── Сборка LaunchDescription ────────────────────────────────────

    return LaunchDescription(
        declared_arguments + [
            control_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            diff_controller_spawner,
            twist_mux_node,
            rosbridge_node,
            rplidar_launch,
            teleop_launch,
            imu_node,
            robot_localization_node,
        ]
    )