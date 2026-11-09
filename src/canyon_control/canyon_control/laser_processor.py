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

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class LaserProcessor(Node):

    def __init__(self):
        super().__init__('canyon_laser_processor')

        # 订阅激光雷达数据
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # 发布处理后的雷达状态
        self.status_pub = self.create_publisher(
            String,
            '/canyon/laser_status',
            10
        )

        # 三个方向的最近有效距离
        self.front_distance = None
        self.left_distance = None
        self.right_distance = None

        self.received_scan = False
        self.last_scan_time = None

        # 每1秒输出一次结果
        self.timer = self.create_timer(
            1.0,
            self.print_status
        )

        self.get_logger().info(
            '峡谷挑战：激光雷达数据处理节点已启动'
        )

    def scan_callback(self, msg):

        front = []
        left = []
        right = []

        # 遍历激光雷达的所有测距数据
        for i, distance in enumerate(msg.ranges):

            # 排除无效数据
            if not math.isfinite(distance):
                continue

            if distance < msg.range_min:
                continue

            if distance > msg.range_max:
                continue

            # 计算当前测量点的角度
            angle = msg.angle_min + i * msg.angle_increment

            # 归一化到 [-pi, pi]
            angle = math.atan2(
                math.sin(angle),
                math.cos(angle)
            )

            angle_deg = math.degrees(angle)

            # 前方：-25° ~ +25°
            if -25.0 <= angle_deg <= 25.0:
                front.append(distance)

            # 左侧：+65° ~ +115°
            if 65.0 <= angle_deg <= 115.0:
                left.append(distance)

            # 右侧：-115° ~ -65°
            if -115.0 <= angle_deg <= -65.0:
                right.append(distance)

        # 取各方向最近的有效障碍物
        self.front_distance = min(front) if front else None
        self.left_distance = min(left) if left else None
        self.right_distance = min(right) if right else None

        self.received_scan = True
        self.last_scan_time = time.monotonic()

    def format_distance(self, distance):

        if distance is None:
            return '无有效回波'

        return f'{distance:.2f} m'

    def print_status(self):

        if not self.received_scan:
            self.get_logger().warn('等待 /scan 雷达数据...')
            return

        # 检测扫描数据是否中断
        if time.monotonic() - self.last_scan_time > 2.0:
            self.get_logger().warn('雷达数据超过2秒未更新')
            return

        front = self.format_distance(self.front_distance)
        left = self.format_distance(self.left_distance)
        right = self.format_distance(self.right_distance)

        status = (
            f'前方: {front} | '
            f'左侧: {left} | '
            f'右侧: {right}'
        )

        # 终端显示
        self.get_logger().info(status)

        # 发布处理结果
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)
    node = LaserProcessor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
