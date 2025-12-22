import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('gp7_robot')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    # Path to URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'GP7 Robot.SLDASM.urdf')

    # Read URDF file
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Include Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        )
    )

    # Static transform publisher (replacing tf static_transform_publisher)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    # Spawn entity node (replacing spawn_model)
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', 'GP7_Robot_SLDASM',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ]
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gazebo_launch,
        static_tf_node,
        robot_state_publisher_node,
        spawn_entity_node
    ])
