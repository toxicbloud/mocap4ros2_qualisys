# Copyright 2019 Intelligent Robotics Lab
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
#
# Author: David Vargas Frutos <david.vargas@urjc.es>
#         Antonin Rousseau    <antonin.rousseau@inria.fr>

import os

from ament_index_python.packages import get_package_share_directory

import launch

from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch.substitutions import LaunchConfiguration

import lifecycle_msgs.msg


def generate_launch_description():
    default_params_path = os.path.join(get_package_share_directory(
      'qualisys_driver'), 'config', 'qualisys_driver_params.yaml')

    # Declare a launch argument 'config' so the user can override the params file
    declare_config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_params_path,
        description='Path to the parameters YAML file for the qualisys driver')

    stdout_linebuf_envvar = SetEnvironmentVariable(
      'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    def launch_setup(context, *args, **kwargs):
      # Resolve the 'config' launch argument at runtime
      config_val = LaunchConfiguration('config').perform(context)

      if os.path.isabs(config_val) or os.path.sep in config_val:
        params_file_path = config_val
      else:
        params_file_path = os.path.join(
          get_package_share_directory('qualisys_driver'), 'config', config_val)

      driver_node = LifecycleNode(
        name='qualisys_driver_node',
        namespace='',
        package='qualisys_driver',
        executable='qualisys_driver_main',
        output='screen',
        parameters=[params_file_path],
      )

      driver_configure_trans_event = EmitEvent(
        event=ChangeState(
          lifecycle_node_matcher=launch.events.matchers.matches_action(driver_node),
          transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
      )

      driver_activate_trans_event = EmitEvent(
         event=ChangeState(
          lifecycle_node_matcher=launch.events.matchers.matches_action(driver_node),
          transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
      )

      return [driver_node, driver_configure_trans_event, driver_activate_trans_event]

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_config_arg)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
