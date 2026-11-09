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
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class CanyonNavigator(Node):

    def __init__(self):
        super().__init__('canyon_navigator')

        # 订阅机器人里程计
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # 订阅激光雷达
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # 发布速度指令
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        # 发布导航状态
        self.state_pub = self.create_publisher(
            String, '/canyon/nav_state', 10
        )

        # 五个目标航点
        self.waypoints = [
            (0.0, 2.8),
            (2.45, 2.8),
            (2.45, 0.2),
            (4.5, 0.2),
            (4.5, 3.0)
        ]

        self.index = 0

        # 机器人位置和方向
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # 数据更新时刻
        self.last_odom_time = None
        self.last_scan_time = None

        # 前方障碍物距离
        self.front_distance = None

        # P控制参数
        self.k_linear = 0.7
        self.k_angular = 1.2

        self.max_linear = 0.12
        self.max_angular = 0.5

        # 航点到达判定
        self.goal_tolerance = 0.12

        # 先对准方向，再前进
        self.angle_tolerance = 0.15

        # 雷达安全停车距离
        self.stop_distance = 0.40

        self.finished = False
        self.last_log_time = 0.0

        # 每0.1秒执行一次控制
        self.timer = self.create_timer(
            0.1, self.control_loop
        )

        self.get_logger().info(
            '峡谷自主穿越导航节点启动'
        )

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        # 四元数转偏航角
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        )

        self.last_odom_time = time.monotonic()

    def scan_callback(self, msg):

        distances = []

        for i, distance in enumerate(msg.ranges):

            angle = msg.angle_min + i * msg.angle_increment

            # 角度归一化
            angle = math.atan2(
                math.sin(angle),
                math.cos(angle)
            )

            angle_deg = math.degrees(angle)

            # 只检查正前方±20度
            if abs(angle_deg) > 20.0:
                continue

            if math.isnan(distance):
                continue

            # 正无穷表示量程内未探测到障碍物
            if math.isinf(distance):
                if distance > 0 and math.isfinite(msg.range_max):
                    distance = msg.range_max
                else:
                    continue

            if msg.range_min <= distance <= msg.range_max:
                distances.append(distance)

        self.front_distance = (
            min(distances) if distances else None
        )

        self.last_scan_time = time.monotonic()

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def publish_state(self, text):

        msg = String()
        msg.data = text
        self.state_pub.publish(msg)

    def log_status(self, text):

        now = time.monotonic()

        if now - self.last_log_time >= 1.0:
            self.get_logger().info(text)
            self.last_log_time = now

    def control_loop(self):

        # 已到终点
        if self.finished:
            self.stop_robot()
            return

        now = time.monotonic()

        # 里程计或雷达丢失时停车
        if (
            self.last_odom_time is None
            or self.last_scan_time is None
            or now - self.last_odom_time > 1.0
            or now - self.last_scan_time > 1.0
            or self.front_distance is None
        ):
            self.stop_robot()
            self.publish_state('WAIT_SENSOR')
            self.log_status('等待有效的里程计和雷达数据')
            return

        # 当前目标
        goal_x, goal_y = self.waypoints[self.index]

        dx = goal_x - self.x
        dy = goal_y - self.y

        distance = math.hypot(dx, dy)

        # 到达当前航点
        if distance < self.goal_tolerance:

            self.stop_robot()

            self.get_logger().info(
                f'成功到达航点 {self.index + 1}: '
                f'({goal_x:.2f}, {goal_y:.2f})'
            )

            self.index += 1

            if self.index >= len(self.waypoints):
                self.finished = True
                self.publish_state('FINISHED')
                self.get_logger().info(
                    '峡谷穿越成功！已到达终点。'
                )
            else:
                self.publish_state(
                    f'WAYPOINT_{self.index + 1}'
                )

            return

        # 计算目标方向
        target_yaw = math.atan2(dy, dx)

        angle_error = math.atan2(
            math.sin(target_yaw - self.yaw),
            math.cos(target_yaw - self.yaw)
        )

        cmd = Twist()

        # 计算并限制角速度
        angular = self.k_angular * angle_error
        angular = max(
            -self.max_angular,
            min(self.max_angular, angular)
        )

        # 方向误差较大：先原地转向
        if abs(angle_error) > self.angle_tolerance:

            cmd.linear.x = 0.0
            cmd.angular.z = angular
            state = 'ALIGNING'

        # 前方过近：停车
        elif self.front_distance < self.stop_distance:

            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            state = 'BLOCKED'

        # 方向正确且前方安全：前进
        else:

            cmd.linear.x = min(
                self.max_linear,
                self.k_linear * distance
            )

            # 靠近障碍时降低速度
            if self.front_distance < 0.75:
                cmd.linear.x = min(cmd.linear.x, 0.05)

            cmd.angular.z = angular
            state = 'MOVING'

        self.cmd_pub.publish(cmd)

        self.publish_state(state)

        self.log_status(
            f'目标 {self.index + 1}/5 | '
            f'位置 ({self.x:.2f}, {self.y:.2f}) | '
            f'距离误差 {distance:.2f}m | '
            f'前方 {self.front_distance:.2f}m | '
            f'状态 {state}'
        )


def main(args=None):

    rclpy.init(args=args)
    node = CanyonNavigator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        for _ in range(3):
            node.stop_robot()
            time.sleep(0.05)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
