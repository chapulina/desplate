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

"""Generate vehicle templated with ERB and visualize on RViz."""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg = os.path.join(get_package_share_directory('desplate_erb'))
    template_path = os.path.join(pkg, 'urdf', 'vehicle.urdf.erb')

    # This line generates a URDF file from an ERB template
    template_str = subprocess.run(['erb', template_path], capture_output=True).stdout.decode()

    # Visualize on RViz
    pkg_desplate_common = get_package_share_directory('desplate_common')
    visualize = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_desplate_common, 'launch', 'visualize.launch.py')]),
        launch_arguments=[('template_str', template_str)],
    )

    return LaunchDescription([
        visualize
    ])
