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

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class OpenLoopDemo(Node):

    def __init__(self):
        super().__init__('canyon_open_loop_demo')

        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.start_time = time.monotonic()
        self.last_state = None

        # 10Hz发布速度指令
        self.timer = self.create_timer(
            0.1, self.control_loop
        )

        self.get_logger().info(
            '开环运动演示启动'
        )

    def control_loop(self):

        elapsed = time.monotonic() - self.start_time

        cmd = Twist()

        # 第1阶段：直线前进2秒
        if elapsed < 2.0:

            state = 'STRAIGHT'
            cmd.linear.x = 0.10
            cmd.angular.z = 0.0

        # 第2阶段：停车1秒
        elif elapsed < 3.0:

            state = 'STOP_1'

        # 第3阶段：原地左转2秒
        elif elapsed < 5.0:

            state = 'ROTATE'
            cmd.linear.x = 0.0
            cmd.angular.z = 0.50

        # 第4阶段：停车并保持停止
        else:

            state = 'FINISHED'

        self.cmd_pub.publish(cmd)

        if state != self.last_state:

            self.get_logger().info(
                f'运动状态: {state} | '
                f'线速度: {cmd.linear.x:.2f}m/s | '
                f'角速度: {cmd.angular.z:.2f}rad/s'
            )

            self.last_state = state

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):

    rclpy.init(args=args)
    node = OpenLoopDemo()

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
