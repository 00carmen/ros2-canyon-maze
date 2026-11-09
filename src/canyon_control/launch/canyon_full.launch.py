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

from ament_index_python.packages import (
    get_package_share_directory
)

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction
)

from launch.conditions import IfCondition

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    Command
)

from launch_ros.actions import Node

from launch_ros.parameter_descriptions import (
    ParameterValue
)


def generate_launch_description():

    # ========= 功能包路径 =========

    canyon_dir = get_package_share_directory(
        'canyon_control'
    )

    gazebo_dir = get_package_share_directory(
        'gazebo_ros'
    )

    tb3_gazebo_dir = get_package_share_directory(
        'turtlebot3_gazebo'
    )

    tb3_description_dir = get_package_share_directory(
        'turtlebot3_description'
    )

    # ========= 文件路径 =========

    world_file = os.path.join(
        canyon_dir,
        'worlds',
        'canyon_maze.world'
    )

    # RViz2 显示配置文件
    rviz_config = os.path.join(
        canyon_dir,
        'rviz',
        'canyon_view.rviz'
    )

    sdf_file = os.path.join(
        tb3_gazebo_dir,
        'models',
        'turtlebot3_burger',
        'model.sdf'
    )

    urdf_file = os.path.join(
        tb3_description_dir,
        'urdf',
        'turtlebot3_burger.urdf'
    )

    # ========= 控制模式参数 =========

    control = LaunchConfiguration('control')

    declare_control = DeclareLaunchArgument(
        'control',
        default_value='none',
        description='none / baseline / adaptive / v3'
    )

    # ========= Gazebo 环境 =========

    set_tb3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='burger'
    )

    model_dir = os.path.join(
        tb3_gazebo_dir,
        'models'
    )

    old_path = os.environ.get(
        'GAZEBO_MODEL_PATH', ''
    )

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.pathsep.join(
            p for p in [model_dir, old_path] if p
        )
    )

    # ========= 加载迷宫 =========

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_dir,
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_file
        }.items()
    )

    # ========= URDF 与 TF =========

    robot_description = ParameterValue(
        Command([
            'xacro ',
            urdf_file,
            ' namespace:='
        ]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # ========= 生成 Burger =========

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_canyon_burger',
        output='screen',
        arguments=[
            '-entity', 'canyon_burger',
            '-file', sdf_file,
            '-x', '0.0',
            '-y', '-1.5',
            '-z', '0.1',
            '-Y', '1.5708'
        ]
    )

    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_robot]
    )

    # ========= 启动 RViz2 =========

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            rviz_config
        ],
        parameters=[{
            'use_sim_time': True
        }]
    )

    # ========= 基础左墙跟随 =========

    baseline = Node(
        package='canyon_control',
        executable='maze_wall_nav',
        name='maze_wall_baseline',
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", control, "' == 'baseline'"
            ])
        )
    )

    # ========= V2 自适应左墙跟随 =========

    adaptive = Node(
        package='canyon_control',
        executable='maze_wall_adaptive',
        name='maze_wall_adaptive',
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", control, "' == 'adaptive'"
            ])
        )
    )

    # ========= V3 最终优化版 =========
    v3 = Node(
        package='canyon_control',
        executable='maze_wall_v3',
        name='maze_wall_v3',
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", control, "' == 'v3'"
            ])
        )
    )

    # 延迟启动算法
    # 如果模型尚未生成，算法会等待传感器数据
    delayed_control = TimerAction(
        period=8.0,
        actions=[
            baseline,
            adaptive,
            v3
        ]
    )

    # 轨迹与状态监测节点
    maze_monitor = Node(
        package='canyon_control',
        executable='maze_monitor',
        name='maze_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        declare_control,
        set_tb3_model,
        set_model_path,
        gazebo,
        robot_state_publisher,
        delayed_spawn,
        rviz,
        delayed_control,
        maze_monitor
    ])
