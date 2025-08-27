from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Launch arguments
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyACM0')
    baudrate_arg = DeclareLaunchArgument('baudrate', default_value='115200')
    imu_rate_arg = DeclareLaunchArgument('imu_rate', default_value='500')
    imu_frame_id_arg = DeclareLaunchArgument('imu_frame_id', default_value='imu_link')

    # Microstrain node
    microstrain_node = Node(
        package='microstrain_mips',
        executable='microstrain_mips_node',
        name='microstrain_mips_node',
        output='screen',
        respawn=False,  # デバッグ時は False、運用時は True
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'device_setup': True,
            'readback_settings': True,
            'save_settings': True,
            'auto_init': True,
            'frame_based_enu': False,
            'publish_imu': True,
            'imu_rate': LaunchConfiguration('imu_rate'),
            'imu_frame_id': LaunchConfiguration('imu_frame_id'),
            'declination_source': 2,
            'declination': 0.23,  # 単位要確認
            'publish_filtered_imu': False,
            'remove_imu_gravity': True,
            'imu_orientation_cov': [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01],
            'imu_linear_cov': [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01],
            'imu_angular_cov': [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01],
            'gps_rate': 4,
            'gps_frame_id': 'navsat_link',
            'nav_rate': 10,
            'dynamics_mode': 1,
            'odom_frame_id': 'wgs84_odom_link',
            'odom_child_frame_id': 'base_link'
        }]
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        imu_rate_arg,
        imu_frame_id_arg,
        microstrain_node,
    ])
