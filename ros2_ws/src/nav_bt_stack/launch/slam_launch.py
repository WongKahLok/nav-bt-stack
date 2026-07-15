import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    nav_bt_stack_dir = get_package_share_directory('nav_bt_stack')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock if true')
    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(nav_bt_stack_dir, 'config', 'slam_toolbox.yaml'),
        description='Full path to the slam_toolbox params file')

    # Same sensor/localization chain as bringup_launch.py Parts 6-7, minus the Nav2
    # stack (AMCL/planners/bt_main) -- SLAM builds map -> odom itself and doesn't need
    # them. Drive the robot with teleop_twist_keyboard (or similar) separately; no
    # teleop node here since driving is manual, not launched.

    # PointCloud2 (/livox/lidar) -> LaserScan (/scan) for slam_toolbox to consume.
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[
            os.path.join(nav_bt_stack_dir, 'config', 'pointcloud_to_laserscan.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('cloud_in', '/livox/lidar'), ('scan', '/scan')],
    )

    # hdas_msg/Imu (/hdas/imu_chassis) -> sensor_msgs/Imu (/imu), for the EKF below.
    imu_hdas_translator = Node(
        package='nav_bt_stack',
        executable='imu_hdas_translator.py',
        name='imu_hdas_translator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Fuses /odom (wheel FK) + /imu into odom -> base_link. slam_toolbox publishes
    # map -> odom on top of this during mapping.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(nav_bt_stack_dir, 'config', 'ekf.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params_file,
        pointcloud_to_laserscan,
        imu_hdas_translator,
        ekf_node,
        slam_toolbox_node,
    ])
