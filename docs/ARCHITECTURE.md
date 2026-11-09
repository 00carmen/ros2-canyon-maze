# 系统结构

`canyon_control` 是一个 `ament_python` 功能包。`setup.py` 注册节点并安装启动文件、场景与 RViz2 配置；启动文件通过 ament 索引定位资源，不依赖某一台电脑上的工作空间路径。

## V3 数据流

```text
Gazebo / TurtleBot3
    ├── /scan ──────────┐
    └── /odom ──────────┼──> maze_wall_v3 ── /cmd_vel ──> Gazebo
                       │          └── /canyon/maze_state
                       └──────────────────┬─────────────┘
                                         v
                                   maze_monitor
                                    ├── /canyon/robot_path ──> RViz2
                                    └── /canyon/monitor_summary
```

`robot_state_publisher` 发布机器人 TF；Gazebo 插件提供雷达、里程计和底盘运动接口。

| 模块 | 主要文件 | 职责 |
| --- | --- | --- |
| 主算法 | `maze_wall_v3.py` | 传感器有效性检查、沿墙控制、转向、开口确认与出口引导 |
| 对照算法 | `maze_wall_nav.py`、`maze_wall_adaptive.py` | 基础版与 V2 |
| 监测 | `maze_monitor.py` | 轨迹、累计里程、状态耗时与完成统计 |
| 旧地图导航 | `canyon_nav.py`、`canyon_nav_lidar.py` | 普通和雷达辅助五航点 |
| 基础节点 | `open_loop_demo.py`、`point_to_point.py`、`laser_processor.py`、`obstacle_avoid.py`、`wall_follower.py` | 分项功能演示 |

## V3 控制过程

1. 等待有效雷达和里程计。缺失或超过 1.2 秒未更新时发布零速度并进入 `WAIT_SENSOR`。
2. 沿左墙行驶，根据前方距离、墙距误差和转向幅度调节速度。
3. 用三次新雷达扫描的连续检测确认左侧开口；确认开口后左转，未确认且前方受阻时右转。
4. 转向后前进一段距离，再恢复沿墙控制。
5. 进入地图末段后，先对准出口内侧点，再向终点移动；距离目标小于 0.15 米时发布 `FINISHED` 并停车。

目标点为 `(4.50, 4.55)`，出口内侧点为 `(4.50, 2.80)`。两者与随附地图及里程计坐标设置配套，换地图时需要重新配置和验证。

## 统计口径与限制

- 控制器和监测器使用 `time.monotonic()` 记录运行耗时；这是实际经过的时间，虚拟机负载会影响结果。
- 累计里程由里程计相邻位置差累加，忽略不小于 0.5 米的单次跳变；它不是规划路径长度。
- RViz2 展示已走过的轨迹；轨迹显示本身不构成建图或全局规划。
- `FINISHED` 基于里程计到预设目标点的距离，尚无独立的全程碰撞检测与成功率评测。
- 运行时间超过 450 秒会触发超时停止；持续缺少传感器数据时，程序可能一直停在等待状态。
- 当前退出障碍状态、异常恢复和多地图适配仍需进一步完善。
