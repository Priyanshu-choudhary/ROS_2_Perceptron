from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package paths
    description_pkg = get_package_share_directory('perceptron_robot_description')
    control_pkg = get_package_share_directory('perceptron_robot_control')

    # Paths
    urdf_file = os.path.join(description_pkg, 'urdf', 'perceptron_robot.xacro')
    controllers_yaml = os.path.join(control_pkg, 'config', 'controllers.yaml')

    # Arguments
    gui = LaunchConfiguration('gui')
    declare_gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Set to "false" to run headless.'
    )

    # Convert xacro → URDF
    robot_description = Command(['xacro ', urdf_file])

    # Robot state publisher (publishes /robot_description + TF tree)
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Start Gazebo Classic (server + GUI)
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Spawn robot into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'perceptron_robot'],
        output='screen'
    )

    # Controller manager (ros2_control)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},
                    controllers_yaml],
        output='screen'
    )

    # Load controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )

    return LaunchDescription([
        declare_gui_arg,
        gazebo,
        robot_state_pub_node,
        spawn_entity,
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_spawner
    ])
