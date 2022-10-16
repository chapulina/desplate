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

"""Generate vehicle templated with Xacro and visualize on RViz."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro


def generate_launch_description():

    pkg = os.path.join(get_package_share_directory('desplate_xacro'))
    template_path = os.path.join(pkg, 'sdf', 'vehicle.sdf.xacro')

    # This line generates a description file from a Xacro template
    description_str = xacro.process_file(template_path).toxml()

    print(description_str)

    # Visualize on RViz
    pkg_desplate_common = get_package_share_directory('desplate_common')
    visualize = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_desplate_common, 'launch', 'visualize.launch.py')]),
        launch_arguments=[
            ('description_str', description_str),
            # Disable joint_state_publisher to reduce noise - it doesn't support SDF
            ('enable_jsp', "false"),
        ],
    )

    return LaunchDescription([
        visualize
    ])
