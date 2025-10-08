# gazebo.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from pathlib import Path

# optional launch arguments
ARGUMENTS = [
    DeclareLaunchArgument(
        'world_path',
        default_value='',
        description='Path to a world file to load (optional)'
    ),
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    ),
]

def generate_launch_description():

    # set up a GAZEBO_MODEL_PATH that includes common models and your package (optional)
    gz_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            '/usr/share/gazebo-11/models/:',
            # add your package models directory (if you have custom models)
            str(Path(get_package_share_directory('perceptron_robot_description')).joinpath('models'))
        ]
    )

    # Launch args
    world_path = LaunchConfiguration('world_path')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # controller config used by the xacro (and optionally by the controller_manager node)
    config_perceptron_controllers = PathJoinSubstitution(
        [FindPackageShare("perceptron_robot_control"), "config", "controllers.yaml"]
    )

    # get URDF via xacro; pass is_sim and gazebo_controllers to the xacro
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([FindPackageShare("perceptron_robot_description"), "urdf", "perceptron_robot.xacro"]),
            " ",
            "is_sim:=true",
            " ",
            "gazebo_controllers:=",
            config_perceptron_controllers,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # robot_state_publisher: publishes TF from the robot_description
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}, robot_description],
    )

    # spawner: joint_state_broadcaster
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    # spawner: diff_drive_controller (your controllers.yaml must define this controller name)
    spawn_diff_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '-c', '/controller_manager'],
        output='screen',
    )

    # ensure diff_drive is spawned after the joint_state_broadcaster
    diffdrive_controller_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_diff_drive],
        )
    )

    # gzserver (physics)
    gzserver = ExecuteProcess( 
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             LaunchConfiguration('world_path')],
        output='screen',
    )

    # gzclient (GUI)
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
    )

    # spawn the robot in Gazebo using the robot_description topic
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_perceptron',
        arguments=['-topic', 'robot_description', '-entity', 'perceptron_robot'],
        output='screen',
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_model_path)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_joint_state_broadcaster)
    ld.add_action(diffdrive_controller_spawn_callback)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(spawn_entity)

    return ld
