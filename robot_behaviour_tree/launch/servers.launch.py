import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    namespace = LaunchConfiguration("namespace", default="")
    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    go_to_target_action_server = Node(
        package="robot_behaviour_tree",
        namespace=namespace,
        arguments=[],
        executable="go_to_target_action_server",
        output="screen",
    )

    ld.add_action(declare_namespace)
    ld.add_action(go_to_target_action_server)

    return ld
