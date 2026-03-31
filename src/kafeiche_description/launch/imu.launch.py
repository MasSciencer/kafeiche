from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Аргументы командной строки
    declare_device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/i2c-1',
        description='I2C device path'
    )
    declare_address_arg = DeclareLaunchArgument(
        'address',
        default_value='41',
        description='I2C address (0x28=40, 0x29=41)'
    )
    declare_frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu',
        description='Frame ID for IMU messages'
    )
    declare_rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='50.0',
        description='Publishing rate in Hz'
    )
    declare_calibration_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value='bno055.json',
        description='Calibration JSON file name (inside config/ folder of kafeiche_drivers)'
    )

    calibration_config = PathJoinSubstitution([
        FindPackageShare("kafeiche_drivers"),
        "config",
        "bno055.json"
    ])

    # Узел IMU
    imu_node = Node(
        package='imu_bno055',
        executable='bno055_i2c_node',
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('device'),
            'address': LaunchConfiguration('address'),
            'frame_id': LaunchConfiguration('frame_id'),
            'rate': LaunchConfiguration('rate'),
            'calibration_file': calibration_config,   # передан полный путь
        }]
    )

    return LaunchDescription([
        declare_device_arg,
        declare_address_arg,
        declare_frame_id_arg,
        declare_rate_arg,
        declare_calibration_arg,
        imu_node,
    ])