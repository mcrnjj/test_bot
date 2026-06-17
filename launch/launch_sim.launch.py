import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'test_bot'
    pkg_path = get_package_share_directory(package_name)

    map_name  = LaunchConfiguration('map_name')
    x_pose    = LaunchConfiguration('x_pose')
    y_pose    = LaunchConfiguration('y_pose')
    yaw_pose  = LaunchConfiguration('yaw')
    world_file = LaunchConfiguration('world')

    # GAZEBO_MODEL_PATH como lista de substitutions (no os.path.join)
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[os.environ.get('GAZEBO_MODEL_PATH', ''), ':',
               pkg_path, '/aruco_assets_', map_name, '/models']
    )

    # Modelo realista (export Fusion360). Para volver al modelo simple,
    # cambia 'robot_v2.urdf.xacro' por 'robot.urdf.xacro'.
    model_file = os.environ.get('ROBOT_MODEL', 'robot_v2.urdf.xacro')
    xacro_file = os.path.join(pkg_path, 'description', model_file)
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    # Resolve package:// URIs to absolute paths so Gazebo/OGRE can load the STL meshes
    robot_description_raw = robot_description_raw.replace(
        'package://test_bot/', pkg_path + '/'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items()
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_bot',
                   '-x', x_pose,
                   '-y', y_pose,
                   '-z', '0.05',
                   '-Y', yaw_pose],
        output='screen'
    )

    delayed_spawn = TimerAction(period=5.0, actions=[spawn_entity])

    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='large'),
        DeclareLaunchArgument(
            'world',
            default_value=[pkg_path, '/aruco_assets_', map_name, '/worlds/test_world_with_markers.world']
        ),
        DeclareLaunchArgument('x_pose', default_value='-0.10'),
        DeclareLaunchArgument('y_pose', default_value='-1.35'),
        DeclareLaunchArgument('yaw',    default_value='0.0'),
        set_gazebo_model_path,
        gazebo,
        node_robot_state_publisher,
        delayed_spawn,
    ])