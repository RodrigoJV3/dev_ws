import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node



def generate_launch_description():

    package_name='robot_pkg'

    # RSP
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    # Define dummy_joint_state_publisher node
    joint_state_publisher = Node(
        package=package_name,
        executable='joint_state_publisher.py',  # Make sure this matches the actual executable name
        name='dummy_joint_state_publisher',
        output='screen'
    )

    # joystick = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','joystick.launch.py'
    #             )])
    # )


    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    

    # Setup for ros2_control

    # Get robot description from rsp mssgs
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    # Define controller.yaml file
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','controllers.yaml')

    # Controller manager node
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    # Delay for controller_manager
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    # Differential driver node
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    # Joints broadcaster node
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )


    # Launch nodes
    return LaunchDescription([
        rsp,
        joint_state_publisher,
        #joystick,
        twist_mux,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])

#ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
#ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/dev-pc/dev_ws/src/robot_pkg/config/mapper_params_online_async.yaml use_sim_time:=false