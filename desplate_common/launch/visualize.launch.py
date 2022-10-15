# Copyright 2022 Louise Poubel
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch necessary tools to visualize description on RViz."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = os.path.join(get_package_share_directory('desplate_common'))

    description_str_arg = DeclareLaunchArgument(
        'description_str',
        description="Description file as a string.")
    enable_jsp_arg = DeclareLaunchArgument(
        'enable_jsp',
        default_value="true",
        description="Enable the joint_state_publisher")

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': LaunchConfiguration('description_str')}]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_jsp'))
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'visualize.rviz')],
    )

    return LaunchDescription([
        description_str_arg,
        enable_jsp_arg,
        robot_state_publisher,
        joint_state_publisher,
        rviz
    ])
