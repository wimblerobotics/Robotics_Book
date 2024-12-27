# Copyright 2024 Michael Wimble
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, version 3.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import OpaqueFunction


def launch_robot_state_publisher(context, file_name_var):
    description_directory_path = get_package_share_directory('description_1')
    file_name = context.perform_substitution(file_name_var)
    print(F"file_name: {file_name}")
    xacro_file_path = os.path.join(description_directory_path, 'urdf', file_name)
    print(F"xacro_file_path: {xacro_file_path}")
    urdf_as_xml = xacro.process_file(xacro_file_path).toxml()
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'frame_prefix': '',
                'ignore_timestamp': False,
                'publish_frequency': 30.0,
                # processed_urdf,
                'robot_description':  urdf_as_xml
            }
        ]
        )
    return [robot_state_publisher_node]

def generate_launch_description():
    launch_args = [DeclareLaunchArgument(
        'urdf', default_value='xxx.urdf', description='URDF file name')]

    do_rviz = LaunchConfiguration('do_rviz')
    publish_joints = LaunchConfiguration('publish_joints')
    urdf_file_name = LaunchConfiguration('urdf_file_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    ld = LaunchDescription(launch_args)

    description_directory_path = get_package_share_directory('description_1')

    ld.add_action(DeclareLaunchArgument(
        name='do_rviz',
        default_value='true',
        description='Launch RViz if true'))
    ld.add_action(DeclareLaunchArgument(
        name='publish_joints',
        default_value='true',
        description='Launch joint_states_publisher if true'))

    ld.add_action(DeclareLaunchArgument(
        name='urdf_file_name',
        default_value='base.urdf.xacro',
        description='URDF file name'))

    ld.add_action(DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'))

    # Publish joints.
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(LaunchConfiguration("publish_joints")),
        parameters=[
            {
                'delta': 0.0,
                'publish_default_efforts': False,
                'publish_default_positions': True,
                'publish_default_velocities': False,
                'rate': 30.0,
                'use_mimic_tag': True,
                'use_smallest_joint_limits': True
            }
        ]
    )
    ld.add_action(joint_state_publisher_node)

    ld.add_action(OpaqueFunction(function=launch_robot_state_publisher, args=[
                             LaunchConfiguration('urdf_file_name')]))
    
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration("do_rviz")),
        arguments=['-d', os.path.join(description_directory_path, 'rviz', 'config.rviz')],
    )
    ld.add_action(rviz_node)

    return ld
