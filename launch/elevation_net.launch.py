# Copyright (c) 2022，Horizon Robotics.
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

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 启动elevation_net pkg
        Node(
            package='elevation_net',
            executable='elevation_net',
            output='screen',
            parameters=[
                {"config_file_path": "./config"},
                {"feed_image": "./config/images/charging_base.png"}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
