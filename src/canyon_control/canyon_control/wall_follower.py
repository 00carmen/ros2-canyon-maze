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
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class WallFollower(Node):

    def __init__(self):
        super().__init__('canyon_wall_follower')

        # 订阅激光雷达
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # 发布速度指令
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # 发布当前控制状态
        self.state_pub = self.create_publisher(
            String,
            '/canyon/wall_state',
            10
        )

        # 左墙跟随的目标距离
        self.target_distance = 0.50

        # P控制比例系数
        self.kp = 0.7

        # 机器人运动参数
        self.forward_speed = 0.10
        self.slow_speed = 0.05
        self.turn_speed = 0.45
        self.max_angular = 0.20

        # 前方避障阈值
        self.front_stop = 0.50
        self.front_clear = 0.80

        # 左前方障碍物阈值
        self.left_front_stop = 0.40
        self.left_front_clear = 0.65

        # 左侧距离过远，认为需要寻找墙壁
        self.wall_lost_distance = 1.50

        # 雷达数据
        self.front = None
        self.left = None
        self.left_front = None

        self.last_scan_time = None
        self.last_log_time = 0.0

        # 控制状态
        self.state = 'WAIT_SCAN'

        # 每0.1秒执行一次控制
        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            '峡谷挑战：左墙跟随节点启动'
        )

    def get_sector_distance(
        self, msg, min_deg, max_deg
    ):
        """Return the nearest valid range within the angular limits."""
        values = []

        for i, distance in enumerate(msg.ranges):

            angle = (
                msg.angle_min
                + i * msg.angle_increment
            )

            angle = math.atan2(
                math.sin(angle),
                math.cos(angle)
            )

            angle_deg = math.degrees(angle)

            if not min_deg <= angle_deg <= max_deg:
                continue

            if math.isnan(distance):
                continue

            # +inf 表示量程内未检测到物体
            if math.isinf(distance):
                if distance > 0 and math.isfinite(
                    msg.range_max
                ):
                    distance = msg.range_max
                else:
                    continue

            if not (
                msg.range_min <= distance <= msg.range_max
            ):
                continue

            values.append(distance)

        return min(values) if values else None

    def scan_callback(self, msg):

        # 正前方：-20度到+20度
        self.front = self.get_sector_distance(
            msg, -20.0, 20.0
        )

        # 左前方：30度到60度
        self.left_front = self.get_sector_distance(
            msg, 30.0, 60.0
        )

        # 正左侧：70度到110度
        self.left = self.get_sector_distance(
            msg, 70.0, 110.0
        )

        self.last_scan_time = time.monotonic()

    def publish_state(self):

        msg = String()
        msg.data = self.state

        self.state_pub.publish(msg)

    def control_loop(self):

        cmd = Twist()

        # 雷达超时或未接收到数据
        if (
            self.last_scan_time is None
            or time.monotonic() - self.last_scan_time > 1.0
        ):
            self.state = 'WAIT_SCAN'

        # 关键方向无有效测量时安全停车
        elif (
            self.front is None
            or self.left is None
            or self.left_front is None
        ):
            self.state = 'NO_SCAN_DATA'

        else:

            # 正在原地右转时，使用较大的恢复阈值
            if self.state == 'TURN_RIGHT':

                if (
                    self.front >= self.front_clear
                    and self.left_front >=
                    self.left_front_clear
                ):
                    self.state = 'FOLLOW_WALL'

            # 检测到前方或左前方障碍
            elif (
                self.front <= self.front_stop
                or self.left_front <= self.left_front_stop
            ):
                self.state = 'TURN_RIGHT'

            # 左侧墙壁太远，尝试寻找
            elif self.left >= self.wall_lost_distance:
                self.state = 'SEARCH_WALL'

            # 正常跟随左墙
            else:
                self.state = 'FOLLOW_WALL'

            if self.state == 'TURN_RIGHT':

                # 前方有障碍，原地右转
                cmd.linear.x = 0.0
                cmd.angular.z = -self.turn_speed

            elif self.state == 'SEARCH_WALL':

                # 缓慢前进，同时向左寻找墙壁
                cmd.linear.x = 0.08
                cmd.angular.z = 0.0

            elif self.state == 'FOLLOW_WALL':

                # 距离误差
                error = (
                    self.left - self.target_distance
                )

                # P控制计算角速度
                angular = self.kp * error

                angular = max(
                    -self.max_angular,
                    min(self.max_angular, angular)
                )

                # 接近前方障碍物时降低速度
                if self.front < self.front_clear:
                    cmd.linear.x = self.slow_speed
                else:
                    cmd.linear.x = self.forward_speed

                cmd.angular.z = angular

        # 其他状态保持零速度
        self.cmd_pub.publish(cmd)
        self.publish_state()
        self.log_status()

    def log_status(self):

        now = time.monotonic()

        if now - self.last_log_time < 1.0:
            return

        self.last_log_time = now

        def fmt(value):
            if value is None:
                return 'N/A'
            return f'{value:.2f}m'

        self.get_logger().info(
            f'状态: {self.state} | '
            f'前方: {fmt(self.front)} | '
            f'左侧: {fmt(self.left)} | '
            f'左前: {fmt(self.left_front)}'
        )

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):

    rclpy.init(args=args)
    node = WallFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        # 退出前发送停车指令
        for _ in range(3):
            node.stop_robot()
            time.sleep(0.05)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
