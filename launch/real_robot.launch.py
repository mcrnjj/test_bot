import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg = get_package_share_directory('test_bot')
    xacro_file = os.path.join(pkg, 'description', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }]
        ),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [640, 480],
                'camera_info_url':
                    'file://' + os.path.join(pkg, 'config', 'camera.yaml'),
            }]
        ),
        Node(
            package='test_bot',
            executable='aruco_localizer',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'markers_db': os.path.join(pkg, 'aruco_assets_small', 'config', 'markers_db.yaml'),
                'publish_tf': True,
                'camera_frame': 'camera_link_optical',
                'base_frame': 'base_link',
            }]
        ),
    ])