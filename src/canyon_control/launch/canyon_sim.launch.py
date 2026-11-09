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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # 1. 获取功能包路径
    canyon_share = get_package_share_directory(
        'canyon_control'
    )

    gazebo_share = get_package_share_directory(
        'gazebo_ros'
    )

    turtlebot_share = get_package_share_directory(
        'turtlebot3_gazebo'
    )

    # 2. 自定义峡谷地图
    world_file = os.path.join(
        canyon_share,
        'worlds',
        'canyon_maze.world'
    )

    # 3. TurtleBot3 Burger 的 SDF 模型
    robot_file = os.path.join(
        turtlebot_share,
        'models',
        'turtlebot3_burger',
        'model.sdf'
    )

    # 4. Gazebo 模型搜索路径
    models_dir = os.path.join(
        turtlebot_share,
        'models'
    )

    old_model_path = os.environ.get(
        'GAZEBO_MODEL_PATH', ''
    )

    model_path = os.pathsep.join(
        p for p in [models_dir, old_model_path] if p
    )

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=model_path
    )

    # 5. 加载自定义 Gazebo 世界
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_share,
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_file
        }.items()
    )

    # 6. 生成 TurtleBot3 Burger

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_canyon_burger',
        output='screen',
        arguments=[
            '-entity', 'canyon_burger',
            '-file', robot_file,

            # 绿色起点坐标
            '-x', '0.0',
            '-y', '-1.5',

            # 机器人初始高度
            '-z', '0.1',

            # 朝向Y轴正方向（迷宫内部）
            '-Y', '1.5708'
        ]
    )

    # 7. 稍后调用生成服务
    # spawn_entity.py 本身也会等待服务可用
    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_robot]
    )

    return LaunchDescription([
        set_model_path,
        gazebo,
        delayed_spawn
    ])
