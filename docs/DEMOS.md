# 分项演示

所有命令都在 Ubuntu 的仓库根目录执行。先完成 README 中的构建步骤；每次切换实验先关闭上一轮仿真，使机器人从固定起点重新开始。

## 迷宫版本

```bash
bash scripts/run_demo.sh v3          # 主版本
bash scripts/run_demo.sh baseline    # 基础版
bash scripts/run_demo.sh adaptive    # V2
```

以上命令一次选择一个运行。对应启动文件为 `canyon_full.launch.py`，`control` 参数分别为 `v3`、`baseline`、`adaptive`。该启动文件默认 `control:=none`，省略参数不会自动运行导航。

## 五航点

```bash
bash scripts/run_demo.sh waypoints        # 雷达辅助五航点
bash scripts/run_demo.sh waypoints-basic  # 普通五航点
```

五航点使用 `canyon_world.world` 旧地图，启动文件为 `canyon_waypoints.launch.py`，`algorithm` 参数分别为 `lidar` 和 `basic`。完成情况以航点节点日志为准；不能用迷宫专属的 `FINISHED` 统计代替判断。

## 开环、点到点、雷达与避障

终端一启动不带运动控制器的场景：

```bash
bash scripts/run_demo.sh none
```

待机器人生成后，在仓库根目录打开终端二：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

然后选择下列一项执行：

| 功能 | 命令 |
| --- | --- |
| 开环直行与转向 | `ros2 run canyon_control open_loop_demo` |
| 点到点 P 控制 | `ros2 run canyon_control point_to_point` |
| 激光雷达分区距离输出 | `ros2 run canyon_control laser_processor` |
| 局部反应式避障 | `ros2 run canyon_control obstacle_avoid` |
| 独立沿墙控制 | `ros2 run canyon_control wall_follower` |

运动节点共用 `/cmd_vel`，不能同时运行。雷达处理节点只输出距离信息。

## 查看状态

在已加载同一工作空间环境的终端中执行：

```bash
ros2 node list
ros2 topic echo /canyon/maze_state
```

V3 到达终点后，可用以下命令读取监测摘要：

```bash
ros2 topic echo /canyon/monitor_summary --once
```

在 RViz2 中选择 `Fixed Frame=odom`，Path 话题选择 `/canyon/robot_path`。缺少轨迹时先检查监测节点与 `/odom`，再检查显示配置。
