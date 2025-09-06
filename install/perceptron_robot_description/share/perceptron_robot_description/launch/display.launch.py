import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Get paths
    pkg_share = get_package_share_directory('perceptron_robot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'perceptron_robot.xacro')

    # Process URDF from Xacro
    print(f"xacro file Path: {xacro_file}")
    robot_description_config = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_config}],
            output='screen'
        ),

        # joint_state_publisher (no GUI)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # joint_state_publisher_gui (GUI)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),
    ])
