from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
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
        get_package_share_directory("kinematic_control_turtle_bot_3"),
        "config",
        "controller_params.yaml",
    )

    siegwart_controller = Node(
        package="kinematic_control_turtle_bot_3",
        executable="tb3_siegwart_controller",
        name="tb3_siegwart_controller",
        parameters=[controller_config_params]
    )
    
    plotjuggler = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "plotjuggler",
            "plotjuggler",
            "--layout",
            os.path.join(
                get_package_share_directory("kinematic_control_turtle_bot_3"),
                "plotjuggler_layout",
                "xy_pose_layout.xml",
            ),
            "--buffer_size",
            "600",
            "--autoshow",
        ],
        output="screen",
    )

    ld.add_action(action=tb3_launch)
    ld.add_action(action=siegwart_controller)
    ld.add_action(action=plotjuggler)
    
    return ld
