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

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class PointToPoint(Node):

    def __init__(self):
        super().__init__('canyon_point_to_point')

        # ===== 目标点参数 =====
        # 当前迷宫入口为 (0, -1.5)
        # 目标位于入口内侧、第一段通道中
        self.declare_parameter('goal_x', 0.25)
        self.declare_parameter('goal_y', -0.25)

        self.goal_x = float(
            self.get_parameter('goal_x').value
        )
        self.goal_y = float(
            self.get_parameter('goal_y').value
        )

        # ===== P控制参数 =====
        self.kp_dist = 0.6
        self.kp_ang = 1.2

        self.max_v = 0.12
        self.max_w = 0.50

        self.dist_tolerance = 0.05
        self.angle_tolerance = 0.25

        # ===== ROS2通信 =====
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.state_pub = self.create_publisher(
            String,
            '/canyon/p2p_state',
            10
        )

        # ===== 位姿状态 =====
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_odom_time = None
        self.last_log_time = 0.0
        self.finished = False

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            f'闭环P控制启动，目标点：'
            f'({self.goal_x:.2f}, {self.goal_y:.2f})'
        )

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        )

        self.last_odom_time = time.monotonic()

    def publish_control(self, v, w, state):

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

        state_msg = String()
        state_msg.data = state
        self.state_pub.publish(state_msg)

    def control_loop(self):

        if self.finished:
            self.publish_control(
                0.0, 0.0, 'FINISHED'
            )
            return

        # 里程计丢失时停车
        if (
            self.last_odom_time is None
            or time.monotonic() - self.last_odom_time > 1.0
        ):
            self.publish_control(
                0.0, 0.0, 'WAIT_ODOM'
            )
            return

        # ===== 位置误差 =====
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        distance = math.hypot(dx, dy)

        # ===== 方向误差 =====
        desired_yaw = math.atan2(dy, dx)

        angle_error = normalize_angle(
            desired_yaw - self.yaw
        )

        # ===== 判断是否到达目标 =====
        if distance <= self.dist_tolerance:

            self.finished = True

            self.publish_control(
                0.0, 0.0, 'FINISHED'
            )

            self.get_logger().info(
                f'目标到达！'
                f'最终位置({self.x:.3f}, {self.y:.3f})，'
                f'位置误差={distance:.3f}m'
            )
            return

        # ===== 角度P控制 =====
        w = clamp(
            self.kp_ang * angle_error,
            -self.max_w,
            self.max_w
        )

        # ===== 距离P控制 =====
        if abs(angle_error) > self.angle_tolerance:

            # 方向偏差大，先原地转向
            v = 0.0
            state = 'ALIGNING'

        else:

            # 方向基本正确，边前进边修正
            v = clamp(
                self.kp_dist * distance,
                0.0,
                self.max_v
            )
            state = 'MOVING'

        self.publish_control(v, w, state)

        # 每秒输出一次误差
        now = time.monotonic()

        if now - self.last_log_time >= 1.0:

            self.get_logger().info(
                f'状态:{state} | '
                f'位置:({self.x:.2f},{self.y:.2f}) | '
                f'距离误差:{distance:.3f}m | '
                f'角度误差:{angle_error:.3f}rad | '
                f'v={v:.3f}m/s w={w:.3f}rad/s'
            )

            self.last_log_time = now

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):

    rclpy.init(args=args)
    node = PointToPoint()

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
