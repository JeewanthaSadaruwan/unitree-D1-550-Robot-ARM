import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('gp7_robot')

    # Path to URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'GP7 Robot.SLDASM.urdf')

    # Read URDF file
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

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

    # Joint State Publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2 node
    rviz_config_file = os.path.join(pkg_share, 'config', 'urdf.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
    )

    # Box Publisher node for grab and place simulation
    boxes_node = Node(
        package='gp7_boxes',
        executable='boxes',
        name='box_publisher',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        boxes_node
    ])
