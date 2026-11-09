#!/usr/bin/env python3
# Copyright (c) 2026 00carmen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""
Run the original enclosed canyon world and five-waypoint demo.

Usage:
  ros2 launch canyon_control canyon_waypoints.launch.py algorithm:=lidar
  ros2 launch canyon_control canyon_waypoints.launch.py algorithm:=basic
  ros2 launch canyon_control canyon_waypoints.launch.py algorithm:=none
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    canyon_dir = get_package_share_directory('canyon_control')
    gazebo_dir = get_package_share_directory('gazebo_ros')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    tb3_description_dir = get_package_share_directory('turtlebot3_description')

    world_file = os.path.join(canyon_dir, 'worlds', 'canyon_world.world')
    rviz_file = os.path.join(canyon_dir, 'rviz', 'canyon_view.rviz')
    burger_sdf = os.path.join(
        tb3_gazebo_dir, 'models', 'turtlebot3_burger', 'model.sdf'
    )
    burger_urdf = os.path.join(
        tb3_description_dir, 'urdf', 'turtlebot3_burger.urdf'
    )

    algorithm = LaunchConfiguration('algorithm')
    algorithm_argument = DeclareLaunchArgument(
        'algorithm',
        default_value='lidar',
        description='none / basic / lidar (only one controller is started)',
    )

    tb3_environment = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL', value='burger'
    )
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_environment = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.pathsep.join(
            item for item in [os.path.join(tb3_gazebo_dir, 'models'), existing_model_path]
            if item
        ),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )

    description = ParameterValue(
        Command(['xacro ', burger_urdf, ' namespace:=']),
        value_type=str,
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': description, 'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_waypoints_burger',
        output='screen',
        arguments=[
            '-entity', 'waypoints_burger',
            '-file', burger_sdf,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-Y', '1.5708',
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
    )

    # This monitor publishes /canyon/robot_path from /odom.
    # Its finish-time summary listens to /canyon/maze_state, so it does NOT
    # produce a FINISHED summary for these waypoint controllers, which publish
    # /canyon/nav_state instead.
    monitor = Node(
        package='canyon_control',
        executable='maze_monitor',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    basic = Node(
        package='canyon_control',
        executable='canyon_nav',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", algorithm, "' == 'basic'"])
        ),
    )
    lidar = Node(
        package='canyon_control',
        executable='canyon_nav_lidar',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", algorithm, "' == 'lidar'"])
        ),
    )

    return LaunchDescription([
        algorithm_argument,
        tb3_environment,
        gazebo_model_environment,
        gazebo,
        robot_state_publisher,
        TimerAction(period=3.0, actions=[spawn_robot]),
        rviz,
        monitor,
        TimerAction(period=8.0, actions=[basic, lidar]),
    ])
