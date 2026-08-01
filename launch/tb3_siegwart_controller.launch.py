from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    
    pkg_share_dir = get_package_share_directory("kinematic_control_turtle_bot_3")
    
    ld = LaunchDescription()
    
    tb3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("turtlebot3_gazebo"),
                "launch",
                "empty_world.launch.py",
            )
        )
    )

    controller_config_params = os.path.join(
        pkg_share_dir,
        "config",
        "controller_params.yaml",
    )

    siegwart_controller = Node(
        package="kinematic_control_turtle_bot_3",
        executable="tb3_siegwart_controller",
        name="tb3_siegwart_controller",
        parameters=[controller_config_params],
        output="screen",
    )
    
    plotjuggler = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "plotjuggler",
            "plotjuggler",
            "--layout",
            os.path.join(
                pkg_share_dir,
                "plotjuggler_layout",
                "xy_pose_layout.xml",
            ),
        ],
        output="screen",
    )
    
    wait_for_gazebo = Node(
        package="kinematic_control_turtle_bot_3",
        executable="wait_for_odom",
        name="wait_for_odom",
        output="screen",
    )
    
    controller_start = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_gazebo,
            on_exit=[
                siegwart_controller,
                plotjuggler,
            ]
        )
    )

    ld.add_action(action=tb3_launch)
    # ld.add_action(action=plotjuggler)
    # ld.add_action(action=siegwart_controller)
    ld.add_action(action=wait_for_gazebo)
    ld.add_action(action=controller_start)
    
    return ld
