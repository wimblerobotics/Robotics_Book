import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    behavior_directory = get_package_share_directory("bt_stepper")

    ld.add_action(
        DeclareLaunchArgument(
            name="xml_path",
            default_value="foo",
            description="Flag to enable joint_state_publisher_gui",
        )
    )

    xml_path_substitution = PathJoinSubstitution(
        [behavior_directory, "config", LaunchConfiguration("xml_path")]
    )
    behavior_node = Node(
        package="bt_stepper",
        executable="bt_run",
        name="bt_stepper",
        parameters=[
            {
                # Use PathJoinSubstitution to build the path at runtime
                "xml_path": xml_path_substitution
            }
        ],
        # prefix=['xterm -e gdb --args'],
        respawn=False,
        output="screen",
    )
    ld.add_action(behavior_node)

    return ld
