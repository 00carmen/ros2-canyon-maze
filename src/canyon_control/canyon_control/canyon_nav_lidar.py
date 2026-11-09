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


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def limit(value, maximum):
    return max(-maximum, min(maximum, value))


class CanyonNavLidar(Node):

    def __init__(self):
        super().__init__('canyon_nav_lidar')

        # ROS2通信
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.create_subscription(
            LaserScan, '/scan',
            self.scan_callback, qos_profile_sensor_data
        )

        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.state_pub = self.create_publisher(
            String, '/canyon/nav_state', 10
        )

        # 峡谷S型路线
        self.waypoints = [
            (0.0, 2.8),
            (2.45, 2.8),
            (2.45, 0.2),
            (4.5, 0.2),
            (4.5, 3.0)
        ]

        self.index = 0

        # 机器人状态
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_odom = None
        self.last_scan = None

        # 雷达三个方向距离
        self.front = None
        self.left = None
        self.right = None

        # 航点控制参数
        self.goal_tolerance = 0.12
        self.angle_tolerance = 0.15

        self.k_linear = 0.7
        self.k_angular = 1.2

        self.max_linear = 0.12
        self.max_angular = 0.50

        # 雷达安全距离
        self.obstacle_distance = 0.55
        self.slow_distance = 0.85
        self.side_safe_distance = 0.60

        # 局部绕行参数
        self.avoid_phase = None
        self.avoid_direction = 0
        self.avoid_yaw = 0.0
        self.avoid_start = (0.0, 0.0)
        self.avoid_distance = 0.40
        self.avoid_attempts = 0
        self.max_avoid_attempts = 3

        self.finished = False
        self.blocked = False

        self.last_log = 0.0
        self.last_state = ''

        # 10Hz控制循环
        self.timer = self.create_timer(
            0.1, self.control_loop
        )

        self.get_logger().info(
            '峡谷雷达融合导航节点启动'
        )

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        self.yaw = math.atan2(
            2.0 * (q.w*q.z + q.x*q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        )

        self.last_odom = time.monotonic()

    def sector_distance(self, msg, min_deg, max_deg):

        values = []

        for i, distance in enumerate(msg.ranges):

            angle = (
                msg.angle_min + i * msg.angle_increment
            )

            angle = normalize_angle(angle)
            degree = math.degrees(angle)

            if not min_deg <= degree <= max_deg:
                continue

            if math.isnan(distance):
                continue

            if math.isinf(distance):
                if distance > 0 and math.isfinite(
                    msg.range_max
                ):
                    distance = msg.range_max
                else:
                    continue

            if msg.range_min <= distance <= msg.range_max:
                values.append(distance)

        return min(values) if values else None

    def scan_callback(self, msg):

        self.front = self.sector_distance(
            msg, -20.0, 20.0
        )

        self.left = self.sector_distance(
            msg, 30.0, 95.0
        )

        self.right = self.sector_distance(
            msg, -95.0, -30.0
        )

        self.last_scan = time.monotonic()

    def send_command(self, linear, angular, state):

        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

        status = String()
        status.data = state
        self.state_pub.publish(status)

        now = time.monotonic()

        # 状态变化时立即记录，否则每秒记录一次
        if (
            state != self.last_state
            or now - self.last_log >= 1.0
        ):

            def fmt(value):
                return (
                    'N/A' if value is None
                    else f'{value:.2f}'
                )

            self.get_logger().info(
                f'目标 {min(self.index + 1, 5)}/5 | '
                f'位置 ({self.x:.2f},{self.y:.2f}) | '
                f'前:{fmt(self.front)}m '
                f'左:{fmt(self.left)}m '
                f'右:{fmt(self.right)}m | '
                f'状态:{state}'
            )

            self.last_log = now
            self.last_state = state

    def start_avoidance(self):

        # 限制单个航点的连续绕行次数
        if self.avoid_attempts >= self.max_avoid_attempts:
            self.blocked = True
            return

        # 至少需要一侧有足够空间
        candidates = []

        if (
            self.left is not None
            and self.left > self.side_safe_distance
        ):
            candidates.append((self.left, 1))

        if (
            self.right is not None
            and self.right > self.side_safe_distance
        ):
            candidates.append((self.right, -1))

        if not candidates:
            self.blocked = True
            return

        # 选择空间较大的一侧
        _, self.avoid_direction = max(candidates)

        # 先转向约60度
        self.avoid_yaw = normalize_angle(
            self.yaw
            + self.avoid_direction * math.radians(60)
        )

        self.avoid_phase = 'TURN'
        self.avoid_attempts += 1

        direction = (
            '左' if self.avoid_direction > 0 else '右'
        )

        self.get_logger().warn(
            f'前方检测到障碍物，尝试向{direction}绕行，'
            f'第{self.avoid_attempts}次'
        )

    def execute_avoidance(self):

        # 第一步：原地调整绕行方向
        if self.avoid_phase == 'TURN':

            error = normalize_angle(
                self.avoid_yaw - self.yaw
            )

            if abs(error) > 0.12:

                angular = limit(
                    1.2 * error, self.max_angular
                )

                self.send_command(
                    0.0, angular, 'AVOID_TURN'
                )
                return

            self.avoid_start = (self.x, self.y)
            self.avoid_phase = 'BYPASS'

        # 第二步：沿绕行方向短距离前进
        if self.avoid_phase == 'BYPASS':

            traveled = math.hypot(
                self.x - self.avoid_start[0],
                self.y - self.avoid_start[1]
            )

            # 完成局部绕行，重新寻找原航点
            if traveled >= self.avoid_distance:

                self.avoid_phase = None
                self.send_command(
                    0.0, 0.0, 'REJOIN_ROUTE'
                )
                return

            # 前方或侧方过近时停车
            if (
                self.front is None
                or self.front < 0.35
                or (
                    self.left is not None
                    and self.left < 0.28
                )
                or (
                    self.right is not None
                    and self.right < 0.28
                )
            ):
                self.blocked = True
                self.send_command(
                    0.0, 0.0, 'BLOCKED'
                )
                return

            error = normalize_angle(
                self.avoid_yaw - self.yaw
            )

            self.send_command(
                0.07,
                limit(0.8 * error, 0.25),
                'AVOID_BYPASS'
            )

    def control_loop(self):

        if self.finished:
            self.send_command(0.0, 0.0, 'FINISHED')
            return

        if self.blocked:
            self.send_command(0.0, 0.0, 'BLOCKED')
            return

        now = time.monotonic()

        # 数据丢失时停车
        if (
            self.last_odom is None
            or self.last_scan is None
            or now - self.last_odom > 1.0
            or now - self.last_scan > 1.0
            or self.front is None
        ):
            self.send_command(
                0.0, 0.0, 'WAIT_SENSOR'
            )
            return

        goal_x, goal_y = self.waypoints[self.index]

        dx = goal_x - self.x
        dy = goal_y - self.y

        distance = math.hypot(dx, dy)

        # 到达当前航点
        if distance < self.goal_tolerance:

            self.get_logger().info(
                f'成功到达航点 {self.index + 1}: '
                f'({goal_x:.2f},{goal_y:.2f})'
            )

            self.index += 1
            self.avoid_phase = None
            self.avoid_attempts = 0

            if self.index >= len(self.waypoints):
                self.finished = True
                self.send_command(
                    0.0, 0.0, 'FINISHED'
                )
                self.get_logger().info(
                    '峡谷穿越完成！已到达终点。'
                )
            else:
                self.send_command(
                    0.0, 0.0, 'NEXT_WAYPOINT'
                )

            return

        # 正在避障时，优先执行绕行动作
        if self.avoid_phase is not None:
            self.execute_avoidance()
            return

        # 计算目标方向
        target_yaw = math.atan2(dy, dx)

        angle_error = normalize_angle(
            target_yaw - self.yaw
        )

        angular = limit(
            self.k_angular * angle_error,
            self.max_angular
        )

        # 先对准目标方向
        if abs(angle_error) > self.angle_tolerance:

            self.send_command(
                0.0, angular, 'ALIGNING'
            )
            return

        # 前方障碍物过近，启动局部避障
        if self.front < self.obstacle_distance:

            self.start_avoidance()

            if self.blocked:
                self.send_command(
                    0.0, 0.0, 'BLOCKED'
                )
            else:
                self.send_command(
                    0.0, 0.0, 'AVOID_START'
                )

            return

        # 正常朝目标前进
        linear = min(
            self.max_linear,
            self.k_linear * distance
        )

        # 前方距离较小时减速
        if self.front < self.slow_distance:
            linear = min(linear, 0.05)

        self.send_command(
            linear, angular, 'MOVING'
        )


def main(args=None):

    rclpy.init(args=args)
    node = CanyonNavLidar()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        for _ in range(3):
            node.send_command(
                0.0, 0.0, 'STOPPED'
            )
            time.sleep(0.05)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
