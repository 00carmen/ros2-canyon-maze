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

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class ObstacleAvoid(Node):

    def __init__(self):
        super().__init__('canyon_obstacle_avoid')

        # 订阅激光雷达
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # 发布机器人运动速度
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # 发布避障状态，供后续监测系统使用
        self.state_pub = self.create_publisher(
            String,
            '/canyon/avoid_state',
            10
        )

        # 前方安全距离阈值
        self.stop_distance = 0.50
        self.clear_distance = 0.85

        # 速度参数
        self.forward_speed = 0.12
        self.slow_speed = 0.05
        self.turn_speed = 0.55

        # 雷达数据
        self.front = None
        self.left = None
        self.right = None

        self.last_scan_time = None
        self.last_log_time = 0.0

        # 控制状态
        self.state = 'WAIT_SCAN'

        # 每0.1秒运行一次
        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            '峡谷挑战：自主避障节点启动'
        )

    def sector_distance(self, msg, min_deg, max_deg):
        """Return the nearest obstacle distance in the requested sector."""
        values = []

        for i, distance in enumerate(msg.ranges):

            angle = msg.angle_min + i * msg.angle_increment

            # 将角度转换为[-180, 180]度
            angle = math.atan2(
                math.sin(angle),
                math.cos(angle)
            )
            angle_deg = math.degrees(angle)

            if not min_deg <= angle_deg <= max_deg:
                continue

            # 过滤异常测量
            if math.isnan(distance):
                continue

            if distance < msg.range_min:
                continue

            if math.isinf(distance):
                # 正无穷通常表示量程内没有检测到障碍物
                if distance > 0 and math.isfinite(msg.range_max):
                    distance = msg.range_max
                else:
                    continue

            if distance > msg.range_max:
                continue

            values.append(distance)

        if not values:
            return None

        return min(values)

    def scan_callback(self, msg):

        # 前方：-25到25度
        self.front = self.sector_distance(
            msg, -25.0, 25.0
        )

        # 左侧：50到120度
        self.left = self.sector_distance(
            msg, 50.0, 120.0
        )

        # 右侧：-120到-50度
        self.right = self.sector_distance(
            msg, -120.0, -50.0
        )

        self.last_scan_time = time.monotonic()

    def choose_turn(self):
        """Choose a turn direction using left and right clearance."""
        if self.left is None and self.right is None:
            return None

        if self.left is None:
            return 'TURN_RIGHT'

        if self.right is None:
            return 'TURN_LEFT'

        if self.left >= self.right:
            return 'TURN_LEFT'

        return 'TURN_RIGHT'

    def publish_state(self):

        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def stop_robot(self):

        self.cmd_pub.publish(Twist())

    def log_status(self):

        now = time.monotonic()

        if now - self.last_log_time < 1.0:
            return

        self.last_log_time = now

        def fmt(value):
            return 'N/A' if value is None else f'{value:.2f}m'

        self.get_logger().info(
            f'状态: {self.state} | '
            f'前: {fmt(self.front)} | '
            f'左: {fmt(self.left)} | '
            f'右: {fmt(self.right)}'
        )

    def control_loop(self):

        cmd = Twist()

        # 没有雷达数据或数据中断时，安全停车
        if (
            self.last_scan_time is None
            or time.monotonic() - self.last_scan_time > 1.0
        ):
            self.state = 'WAIT_SCAN'
            self.stop_robot()
            self.publish_state()
            self.log_status()
            return

        # 前方没有可用测距数据时，不盲目前进
        if self.front is None:
            self.state = 'NO_FRONT_DATA'
            self.stop_robot()
            self.publish_state()
            self.log_status()
            return

        # 初始状态或数据恢复后，从前进状态重新判断
        if self.state in ('WAIT_SCAN', 'NO_FRONT_DATA',
                          'NO_SIDE_DATA'):
            self.state = 'FORWARD'

        # 当前未转向，且检测到前方障碍
        if self.state == 'FORWARD':
            if self.front <= self.stop_distance:
                direction = self.choose_turn()

                if direction is None:
                    self.state = 'NO_SIDE_DATA'
                else:
                    self.state = direction

        # 正在转向时，前方足够畅通才结束转向
        if self.state in ('TURN_LEFT', 'TURN_RIGHT'):
            if self.front >= self.clear_distance:
                self.state = 'FORWARD'

        # 根据状态产生速度指令
        if self.state == 'TURN_LEFT':
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed

        elif self.state == 'TURN_RIGHT':
            cmd.linear.x = 0.0
            cmd.angular.z = -self.turn_speed

        elif self.state == 'FORWARD':
            if self.front < self.clear_distance:
                cmd.linear.x = self.slow_speed
            else:
                cmd.linear.x = self.forward_speed

            cmd.angular.z = 0.0

        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)
        self.publish_state()
        self.log_status()


def main(args=None):

    rclpy.init(args=args)
    node = ObstacleAvoid()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # 退出时发送停止指令
        for _ in range(3):
            node.stop_robot()
            time.sleep(0.05)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
