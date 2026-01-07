import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('d1_550_description').find('d1_550_description')
    
    # Path to URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'd1_550_description.urdf')
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Joint State Publisher GUI node - for interactive joint control
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'urdf.rviz')] if os.path.exists(os.path.join(pkg_share, 'urdf.rviz')) else []
    )
    
    # Environment visualization publisher - Shows pick/place boxes
    env_publisher = Node(
        package='d1_550_description',
        executable='environment_publisher.py',
        name='environment_publisher',
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        env_publisher  # Enabled - shows boxes for pick and place
    ])
