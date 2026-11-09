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
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class MazeWallNavigator(Node):

    def __init__(self):
        super().__init__('maze_wall_navigator_adaptive')

        # ===== ROS2通信 =====
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

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
            '/canyon/maze_state',
            10
        )

        # ===== 迷宫出口（世界坐标）=====
        self.goal_x = 4.50
        self.goal_y = 4.55
        self.goal_tolerance = 0.15

        # ===== 出口引导参数 =====
        self.exit_mode = False
        self.exit_stage = 0

        # 出口内部的安全对准点
        self.exit_pre_x = 4.50
        self.exit_pre_y = 2.80

        # ===== 基础左墙跟随参数 =====
        self.target_left = 0.50
        self.kp = 0.85

        self.forward_speed = 0.10

        # ===== 自适应速度参数 =====
        self.adaptive_max_speed = 0.16
        self.adaptive_mid_speed = 0.11
        self.adaptive_min_speed = 0.06

        self.turn_speed_max = 0.50
        self.wall_angular_max = 0.22

        # 障碍物判断阈值
        self.front_stop = 0.48
        self.left_front_stop = 0.33
        self.wall_lost = 1.25
        self.wall_detected = 1.10

        # ===== 机器人位姿 =====
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # ===== 雷达测距 =====
        self.front = None
        self.left = None
        self.left_front = None
        self.right = None

        self.last_scan_time = None
        self.last_odom_time = None

        # ===== 状态机 =====
        self.state = 'WAIT_SENSOR'

        self.turn_target = 0.0
        self.advance_start = (0.0, 0.0)
        self.advance_distance = 0.30

        # 检测左墙是否曾靠近
        self.has_left_contact = False

        # 左侧开口需要连续检测
        self.left_open_count = 0

        self.last_log_time = 0.0
        self.previous_log_state = ''

        self.start_time = None
        self.finished = False
        self.blocked = False

        # 10Hz控制
        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            '迷宫左墙跟随节点启动，等待雷达和里程计'
        )

    def sector_distance(self, msg, deg_min, deg_max):
        """Return the closest valid range in the requested angular sector."""
        values = []

        for i, distance in enumerate(msg.ranges):

            angle = (
                msg.angle_min
                + i * msg.angle_increment
            )

            degree = math.degrees(
                normalize_angle(angle)
            )

            if not deg_min <= degree <= deg_max:
                continue

            if math.isnan(distance):
                continue

            # 没测到量程内障碍时，按最大量程处理
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

        self.left_front = self.sector_distance(
            msg, 30.0, 60.0
        )

        self.left = self.sector_distance(
            msg, 75.0, 105.0
        )

        self.right = self.sector_distance(
            msg, -105.0, -75.0
        )

        self.last_scan_time = time.monotonic()

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        self.yaw = math.atan2(
            2.0 * (q.w*q.z + q.x*q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        )

        self.last_odom_time = time.monotonic()

    def publish_cmd(self, linear, angular, state):

        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)

        self.cmd_pub.publish(cmd)

        self.state = state

        status = String()
        status.data = state
        self.state_pub.publish(status)

        now = time.monotonic()

        if (
            state != self.previous_log_state
            or now - self.last_log_time >= 1.0
        ):

            def fmt(value):
                if value is None:
                    return 'N/A'
                return f'{value:.2f}'

            self.get_logger().info(
                f'状态:{state} | '
                f'线速度:{linear:.3f}m/s | '
                f'位置:({self.x:.2f},{self.y:.2f}) | '
                f'前:{fmt(self.front)}m '
                f'左:{fmt(self.left)}m '
                f'左前:{fmt(self.left_front)}m'
            )

            self.previous_log_state = state
            self.last_log_time = now

    def begin_turn(self, direction):
        """Start a quarter turn: +1 for left, -1 for right."""
        self.turn_target = normalize_angle(
            self.yaw + direction * math.pi / 2.0
        )

        if direction > 0:
            self.state = 'TURN_LEFT'
            self.has_left_contact = False
        else:
            self.state = 'TURN_RIGHT'

        self.left_open_count = 0

    def execute_turn(self):

        error = normalize_angle(
            self.turn_target - self.yaw
        )

        # 角度误差小于0.10rad，视为转向完成
        if abs(error) <= 0.10:

            if self.state == 'TURN_LEFT':
                self.advance_distance = 0.40
            else:
                self.advance_distance = 0.30

            self.advance_start = (self.x, self.y)
            self.state = 'ADVANCE'

            self.publish_cmd(
                0.0, 0.0, 'ADVANCE'
            )
            return

        angular = clamp(
            1.2 * error,
            -self.turn_speed_max,
            self.turn_speed_max
        )

        self.publish_cmd(
            0.0, angular, self.state
        )

    def execute_advance(self):

        distance = math.hypot(
            self.x - self.advance_start[0],
            self.y - self.advance_start[1]
        )

        # 转弯后前进一定距离，避免反复触发墙角
        if distance >= self.advance_distance:
            self.state = 'FOLLOW_WALL'
            self.publish_cmd(
                0.0, 0.0, 'FOLLOW_WALL'
            )
            return

        # 新方向仍被阻挡则安全停车
        if (
            self.front is None
            or self.front < 0.32
        ):
            self.blocked = True
            self.publish_cmd(
                0.0, 0.0, 'BLOCKED'
            )
            return

        error = normalize_angle(
            self.turn_target - self.yaw
        )

        self.publish_cmd(
            0.08,
            clamp(0.8 * error, -0.16, 0.16),
            'ADVANCE'
        )

    def execute_exit_approach(self):
        """Align with the exit waypoint and drive through the final corridor."""
        # 阶段0：先到出口内侧的对准点
        if self.exit_stage == 0:
            gx = self.exit_pre_x
            gy = self.exit_pre_y

            distance = math.hypot(
                gx - self.x,
                gy - self.y
            )

            if distance < 0.20:
                self.exit_stage = 1
                self.publish_cmd(
                    0.0, 0.0, 'EXIT_PREPOINT_OK'
                )
                self.get_logger().info(
                    '已到达出口对准点，开始驶向出口'
                )
                return

        # 阶段1：前往黄色终点
        else:
            gx = self.goal_x
            gy = self.goal_y

            distance = math.hypot(
                gx - self.x,
                gy - self.y
            )

        # 计算目标方向
        target_yaw = math.atan2(
            gy - self.y,
            gx - self.x
        )

        angle_error = normalize_angle(
            target_yaw - self.yaw
        )

        # 方向偏差较大时，先原地调整朝向
        if abs(angle_error) > 0.16:
            angular = clamp(
                1.1 * angle_error,
                -0.45,
                0.45
            )

            self.publish_cmd(
                0.0, angular, 'EXIT_ALIGN'
            )
            return

        # 前方危险时停车，防止撞墙
        if self.front < 0.40:
            self.publish_cmd(
                0.0, 0.0, 'EXIT_BLOCKED'
            )
            return

        # 出口引导阶段采用安全自适应速度
        speed = min(
            0.14,
            0.55 * distance
        )

        # 前方较近时提前减速
        if self.front < 0.80:
            speed = min(speed, 0.04)

        elif self.front < 1.20:
            speed = min(speed, 0.09)

        # 方向偏差较大时限制前进速度
        if abs(angle_error) > 0.10:
            speed = min(speed, 0.08)

        angular = clamp(
            1.0 * angle_error,
            -0.18,
            0.18
        )

        self.publish_cmd(
            speed,
            angular,
            'EXIT_MOVING'
        )

    def calculate_adaptive_speed(self, wall_error, angular):
        """Choose a forward speed from clearance and wall-following error."""
        # 1. 距离较危险时，低速行驶
        if (
            self.front < 0.70
            or self.left_front < 0.42
        ):
            speed = self.adaptive_min_speed

        # 2. 中等距离，正常速度
        elif (
            self.front < 1.00
            or self.left_front < 0.70
        ):
            speed = self.adaptive_mid_speed

        # 3. 前方较空旷，适度加速
        else:
            speed = self.adaptive_max_speed

        # 4. 墙距误差较大时限制速度
        if abs(wall_error) > 0.25:
            speed = min(
                speed,
                self.adaptive_mid_speed
            )

        # 5. 转向幅度较大时降低速度
        if abs(angular) > 0.18:
            speed = min(
                speed,
                self.adaptive_mid_speed
            )

        # 6. 距离左墙太近时进一步减速
        if self.left < 0.35:
            speed = self.adaptive_min_speed

        return speed

    def control_loop(self):

        now = time.monotonic()

        if self.finished:
            self.publish_cmd(
                0.0, 0.0, 'FINISHED'
            )
            return

        if self.blocked:
            self.publish_cmd(
                0.0, 0.0, 'BLOCKED'
            )
            return

        # ===== 传感器安全检查 =====
        if (
            self.last_scan_time is None
            or self.last_odom_time is None
            or now - self.last_scan_time > 1.2
            or now - self.last_odom_time > 1.2
            or self.front is None
            or self.left is None
            or self.left_front is None
        ):
            self.publish_cmd(
                0.0, 0.0, 'WAIT_SENSOR'
            )
            return

        if self.start_time is None:
            self.start_time = now

        # 防止无限运行
        if now - self.start_time > 450.0:
            self.blocked = True
            self.publish_cmd(
                0.0, 0.0, 'TIMEOUT'
            )
            self.get_logger().warn(
                '运行超过450秒，停止控制'
            )
            return

        # ===== 终点判定 =====
        goal_error = math.hypot(
            self.goal_x - self.x,
            self.goal_y - self.y
        )

        if goal_error < self.goal_tolerance:

            self.finished = True
            self.publish_cmd(
                0.0, 0.0, 'FINISHED'
            )

            self.get_logger().info(
                f'迷宫通关成功！'
                f'最终位置({self.x:.2f},{self.y:.2f}) '
                f'误差:{goal_error:.3f}m '
                f'耗时:{now - self.start_time:.1f}s'
            )
            return

        # ===== 进入最后通道后启用出口引导 =====
        if (
            not self.exit_mode
            and self.x >= 3.85
            and self.y >= 0.15
        ):

            self.exit_mode = True

            # 如果已经靠近出口对准点，
            # 不必返回更低的位置
            if (
                self.y >= 2.65
                and abs(self.x - 4.50) <= 0.35
            ):
                self.exit_stage = 1
            else:
                self.exit_stage = 0

            self.get_logger().info(
                '进入最后通道，切换到出口引导模式'
            )

        # 出口模式优先于普通沿墙转向逻辑
        if self.exit_mode:
            self.execute_exit_approach()
            return

        # ===== 转向状态 =====
        if self.state in ('TURN_LEFT', 'TURN_RIGHT'):
            self.execute_turn()
            return

        # ===== 转向后的前进状态 =====
        if self.state == 'ADVANCE':
            self.execute_advance()
            return

        # ===== 前方有障碍，优先右转 =====

        # ===== 更新左墙接触状态 =====
        if self.left < self.wall_detected:
            self.has_left_contact = True

        # ===== 检测左侧是否出现可通行开口 =====
        left_open = (
            self.left > self.wall_lost
            and self.left_front > 0.90
            and self.front > 0.55
        )

        # 连续检测，避免单次雷达误判
        if self.has_left_contact and left_open:
            self.left_open_count += 1
        else:
            self.left_open_count = 0

        # ===== 优先处理左侧开口 =====
        # 左手法则：发现可通行左侧通道时左转
        if self.left_open_count >= 4:

            self.get_logger().info(
                '检测到左侧开口，准备左转'
            )

            self.begin_turn(1)

            self.publish_cmd(
                0.0, 0.0, 'TURN_LEFT'
            )
            return

        # ===== 前方被堵住时右转 =====
        # 仅在没有确认左侧开口时执行
        if (
            self.front < self.front_stop
            or self.left_front < self.left_front_stop
        ):

            self.get_logger().info(
                '前方存在障碍，准备右转'
            )

            self.begin_turn(-1)

            self.publish_cmd(
                0.0, 0.0, 'TURN_RIGHT'
            )
            return

        # ===== 左侧尚未找到墙壁 =====
        if self.left > self.wall_lost:

            # 先直行寻找墙壁，避免持续绕小圈
            self.publish_cmd(
                0.08, 0.0, 'SEARCH_WALL'
            )
            return

        # ===== 正常左墙P控制 =====
        error = self.left - self.target_left

        angular = clamp(
            self.kp * error,
            -self.wall_angular_max,
            self.wall_angular_max
        )

        # 左侧太近时，先修正方向
        if self.left < 0.25:

            self.publish_cmd(
                0.02, -0.30, 'TOO_CLOSE'
            )
            return

        # 根据障碍物距离和墙距误差计算自适应速度
        adaptive_speed = self.calculate_adaptive_speed(
            error,
            angular
        )

        self.publish_cmd(
            adaptive_speed,
            angular,
            'FOLLOW_WALL'
        )

    def stop_robot(self):

        self.cmd_pub.publish(Twist())


def main(args=None):

    rclpy.init(args=args)
    node = MazeWallNavigator()

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
