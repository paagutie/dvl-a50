# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Launch a talker and a listener in a component container."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    launch.actions.DeclareLaunchArgument('ip_address', default_value='192.168.194.95'),
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='dvl_a50',
                    plugin='composition::LifecycleDVL',
                    name='dvl_a50_node',
                    parameters=[{'dvl_ip_address': launch.substitutions.LaunchConfiguration('ip_address')}],
                    extra_arguments=[{'use_intra_process_comms': True}])#,
                #ComposableNode(
                #    package='composition',
                #   plugin='composition::Listener',
                #    name='listener')
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
