import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg_share = get_package_share_directory('gp7_robot_description')

    urdf_path = os.path.join(pkg_share, "urdf", "gp7_robot.urdf")

    with open(urdf_path, 'r') as inf:
        robot_desc = inf.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    joint_state_pub_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_pub_gui,
        rviz
    ])
