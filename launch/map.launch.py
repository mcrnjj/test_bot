from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

PKG = get_package_share_directory('test_bot')

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_name = LaunchConfiguration('map_name')
    map_file = LaunchConfiguration('map_file')
    markers_db = LaunchConfiguration('markers_db')
    nav2_params = LaunchConfiguration('nav2_params')
    ekf_params = LaunchConfiguration('ekf_params')

    nav2_lifecycle_nodes = [
        'map_server', 'planner_server', 'controller_server',
        'recoveries_server', 'bt_navigator', 'waypoint_follower',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_name', default_value='large'),
        DeclareLaunchArgument('map_file', default_value=[PKG, '/config/test_map_', map_name, '.yaml']),
        DeclareLaunchArgument('markers_db', default_value=[PKG, '/aruco_assets_', map_name, '/config/markers_db.yaml']),
        DeclareLaunchArgument('nav2_params', default_value=f'{PKG}/config/nav2_params.yaml'),
        DeclareLaunchArgument('ekf_params', default_value=f'{PKG}/config/ekf.yaml'),

        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[{'yaml_filename': map_file, 'use_sim_time': use_sim_time}]),

        Node(package='test_bot', executable='aruco_localizer', name='aruco_localizer',
             output='screen',
             parameters=[{
                 'use_sim_time': use_sim_time,
                 'markers_db': markers_db,
                 'image_topic': '/camera/image_raw',
                 'camera_info_topic': '/camera/camera_info',
                 'camera_frame': 'camera_link_optical',
                 'base_frame': 'base_link',
                 'odom_frame': 'odom',
                 'map_frame': 'map',
                 'publish_tf': False,
                 'max_distance': 2.0,
                 'max_reproj_error_px': 3.0,
                 'min_marker_area_px': 200.0,
                 'filter_window': 1,
                 'ambiguity_ratio_threshold': 1.5,
                 'marker_frame_correction_rpy': [1.5708, 0.0, 0.0],
             }]),

        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node_odom', output='screen',
             parameters=[ekf_params],
             remappings=[('/odometry/filtered', '/odometry/filtered_odom')]),

        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node_map', output='screen',
             parameters=[ekf_params],
             remappings=[('/odometry/filtered', '/odometry/filtered_map')]),

        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[nav2_params]),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=[nav2_params]),
        Node(package='nav2_recoveries', executable='recoveries_server', name='recoveries_server',
             output='screen', parameters=[nav2_params]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[nav2_params]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=[nav2_params]),
     Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
     name='lifecycle_manager_map', output='screen',
     parameters=[{'autostart': True,
                  'node_names': ['map_server'],
                  'use_sim_time': use_sim_time}]),

Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
     name='lifecycle_manager_navigation', output='screen',
     parameters=[{'autostart': True,
                  'node_names': ['planner_server', 'controller_server',
                                 'recoveries_server', 'bt_navigator',
                                 'waypoint_follower'],
                  'use_sim_time': use_sim_time}]),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             output='screen', parameters=[{'use_sim_time': use_sim_time}]),
    ])