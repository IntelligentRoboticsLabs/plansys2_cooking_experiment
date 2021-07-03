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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('plansys2_cooking_example')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_monolithic_launch.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/cooking_domain.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions
    move_1_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move_1',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'move',
            'bt_xml_file': example_dir + '/behavior_trees_xml/move.xml'
          }
        ])

    transport_1_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='transport_1',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'transport',
            'bt_xml_file': example_dir + '/behavior_trees_xml/transport.xml'
          }
        ])
    recharge_1_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='recharge_1',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'recharge',
            'bt_xml_file': example_dir + '/behavior_trees_xml/recharge.xml'
          }
        ])

    cook_omelette_1_cmd = Node(
        package='plansys2_cooking_example',
        executable='cook_omelette_action_node',
        name='cook_omelette_1',
        namespace=namespace,
        output='screen',
        parameters=[])   # Create the launch description and populate

    cook_cake_1_cmd = Node(
        package='plansys2_cooking_example',
        executable='cook_cake_action_node',
        name='cook_cake_1',
        namespace=namespace,
        output='screen',
        parameters=[])   # Create the launch description and populate

    cook_spaghetti_1_cmd = Node(
        package='plansys2_cooking_example',
        executable='cook_spaghetti_action_node',
        name='cook_spaghetti_1',
        namespace=namespace,
        output='screen',
        parameters=[])   # Create the launch description and populate

    marker_cmd = Node(
        package='plansys2_cooking_example',
        executable='marker_pub',
        name='marker_pub',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml'
        ])   # Create the launch description and populate


    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(move_1_cmd)
    ld.add_action(transport_1_cmd)
    ld.add_action(recharge_1_cmd)
    ld.add_action(cook_omelette_1_cmd)
    ld.add_action(cook_cake_1_cmd)
    ld.add_action(cook_spaghetti_1_cmd)

    ld.add_action(marker_cmd)

    return ld
