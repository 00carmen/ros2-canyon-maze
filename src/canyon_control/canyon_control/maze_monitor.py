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

import math
import time
from collections import defaultdict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class MazeMonitor(Node):

    def __init__(self):
        super().__init__('canyon_maze_monitor')

        # 订阅里程计和导航状态
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.state_sub = self.create_subscription(
            String, '/canyon/maze_state',
            self.state_callback, 10
        )

        # 发布实际轨迹和统计结果
        self.path_pub = self.create_publisher(
            Path, '/canyon/robot_path', 10
        )

        self.summary_pub = self.create_publisher(
            String, '/canyon/monitor_summary', 10
        )

        # 实际轨迹
        self.path = Path()
        self.path.header.frame_id = 'odom'

        # 运动统计
        self.total_distance = 0.0
        self.last_position = None
        self.last_record_time = 0.0
        self.current_position = None

        # 状态统计
        self.current_state = None
        self.state_start_time = None
        self.state_durations = defaultdict(float)

        # 时间统计
        self.run_start_time = None
        self.run_elapsed = None
        self.finished = False

        # 定时发布完整轨迹
        self.timer = self.create_timer(
            0.5, self.publish_results
        )

        self.get_logger().info(
            '迷宫轨迹与运行状态监测节点启动'
        )

    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if not math.isfinite(x) or not math.isfinite(y):
            return

        self.current_position = (x, y)

        # 计算累计行驶距离
        if self.last_position is not None:
            step = math.hypot(
                x - self.last_position[0],
                y - self.last_position[1]
            )

            # 忽略异常的位置跳变
            if step < 0.5:
                self.total_distance += step

        self.last_position = (x, y)

        # 每0.1秒记录一个轨迹点
        now = time.monotonic()

        if now - self.last_record_time < 0.1:
            return

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path.header.frame_id = (
            msg.header.frame_id or 'odom'
        )
        self.path.poses.append(pose)

        self.last_record_time = now

    def state_callback(self, msg):

        state = msg.data
        now = time.monotonic()

        # 统计每种状态的累计持续时间
        if state != self.current_state:

            if (
                self.current_state is not None
                and self.state_start_time is not None
            ):
                duration = now - self.state_start_time
                self.state_durations[
                    self.current_state
                ] += duration

            self.current_state = state
            self.state_start_time = now

        # 有效控制开始时记录起始时刻
        if (
            self.run_start_time is None
            and state not in (
                'WAIT_SENSOR',
                'FINISHED',
                'STOPPED'
            )
        ):
            self.run_start_time = now

        # 只记录一次完成事件
        if state == 'FINISHED' and not self.finished:

            self.finished = True

            if self.run_start_time is not None:
                self.run_elapsed = (
                    now - self.run_start_time
                )

            self.get_logger().info(
                '检测到迷宫通关完成'
            )

            self.print_summary()

    def print_summary(self):

        self.get_logger().info(
            '========== 迷宫运行统计 =========='
        )

        if self.run_elapsed is not None:
            self.get_logger().info(
                f'监测耗时: {self.run_elapsed:.1f}s'
            )

        self.get_logger().info(
            f'累计行驶距离: '
            f'{self.total_distance:.2f}m'
        )

        if self.current_position is not None:
            x, y = self.current_position
            self.get_logger().info(
                f'最终位置: ({x:.2f}, {y:.2f})'
            )

        for state, duration in sorted(
            self.state_durations.items()
        ):
            self.get_logger().info(
                f'{state}: {duration:.1f}s'
            )

        self.get_logger().info(
            '=================================='
        )

    def publish_results(self):

        if self.path.poses:

            self.path.header.stamp = (
                self.get_clock().now().to_msg()
            )

            self.path_pub.publish(self.path)

        # 持续发布最终统计，方便终端读取
        if self.finished:

            summary = String()

            elapsed = (
                f'{self.run_elapsed:.1f}s'
                if self.run_elapsed is not None
                else 'N/A'
            )

            summary.data = (
                f'FINISHED | 耗时:{elapsed} | '
                f'里程:{self.total_distance:.2f}m'
            )

            self.summary_pub.publish(summary)


def main(args=None):

    rclpy.init(args=args)
    node = MazeMonitor()

    try:
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        node.destroy_node()

        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
