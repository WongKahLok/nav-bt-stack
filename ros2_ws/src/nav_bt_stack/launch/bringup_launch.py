import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    nav_bt_stack_dir = get_package_share_directory('nav_bt_stack')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    tree_id = LaunchConfiguration('tree_id')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock if true')
    declare_map = DeclareLaunchArgument(
        'map',
        description='Full path to the map yaml file to load')
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_bt_stack_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 params file')
    declare_tree_id = DeclareLaunchArgument(
        'tree_id', default_value='GpsrTask',
        description='Which BT to run; see bt_xml/tasks for the available IDs')

    # Brings up AMCL, controller/planner/behavior servers, bt_navigator,
    # waypoint_follower, velocity_smoother and the lifecycle manager -- all
    # configured via params_file (see config/nav2_params.yaml)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    bt_main = Node(
        package='nav_bt_stack',
        executable='bt_main',
        name='nav_bt_stack',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'tree_id': tree_id}],
    )

    # Part 6: PointCloud2 (/livox/lidar, from robocup_nav -- sim or real Mid-360 driver,
    # neither emits a native LaserScan) -> LaserScan (/scan) for AMCL/costmaps.
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

    # Part 7: hdas_msg/Imu (/hdas/imu_chassis, from robocup_nav -- sim or real HDAS driver)
    # -> sensor_msgs/Imu (/imu), for the EKF below. Runs unmodified against sim or real
    # hardware since both publish the same hdas_msg/Imu contract.
    imu_hdas_translator = Node(
        package='nav_bt_stack',
        executable='imu_hdas_translator.py',
        name='imu_hdas_translator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Part 7: fuses /odom (wheel FK, from robocup_nav) + /imu (above) into odom -> base_link.
    # Sole broadcaster of that TF -- robocup_nav's swerve_odometry.py deliberately doesn't.
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

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params_file,
        declare_tree_id,
        nav2_bringup,
        bt_main,
        pointcloud_to_laserscan,
        imu_hdas_translator,
        ekf_node,
    ])
