# ROS 2 Canyon Maze

基于 ROS 2 Humble、Gazebo Classic 和 TurtleBot3 Burger 的峡谷迷宫导航演示项目。

主版本 **V3** 使用激光雷达进行左墙跟随，通过自适应速度和连续新扫描确认识别开口，最后结合里程计与预设出口坐标到达终点。RViz2 显示机器人、激光和行驶轨迹，监测节点输出耗时、里程及状态统计。

当前版本为 **0.1.0**，维护者为 [00carmen](https://github.com/00carmen)，源码仓库为 [ros2-canyon-maze](https://github.com/00carmen/ros2-canyon-maze)。项目原创代码采用 MIT 许可证。

## 功能与范围

| 功能 | 说明 |
| --- | --- |
| V3 迷宫导航 | 主要演示入口，包含开口确认和末段出口引导 |
| 基础版、V2 | 保留固定速度与自适应速度版本，便于对照 |
| 基础控制 | 开环运动、点到点控制、五航点跟踪、雷达处理与局部避障 |
| 可视化与统计 | RViz2 轨迹、控制状态、运行耗时和累计里程 |

这是针对随附地图调试的仿真项目。出口坐标、航点和控制阈值目前写在源码中；项目尚未实现 SLAM、全局路径规划、动态重规划或实机验证，也不保证适用于任意迷宫。

## 环境

目标环境：Ubuntu 22.04、ROS 2 Humble、Python 3.10、Gazebo Classic、RViz2，以及 TurtleBot3 的 `turtlebot3_gazebo` 和 `turtlebot3_description` 功能包。

新版 Gazebo 和其他 ROS 2 发行版尚未适配。先安装 ROS 2 Humble、colcon 和 rosdep；机器人模型通过官方依赖获取，本仓库不复制其网格文件。

## 构建

本仓库本身就是 colcon 工作空间。在 Ubuntu 终端下载源码并构建：

```bash
git clone https://github.com/00carmen/ros2-canyon-maze.git
cd ros2-canyon-maze

source /opt/ros/humble/setup.bash

# 仅在机器尚未初始化 rosdep 时执行：sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y

colcon build --symlink-install --packages-select canyon_control
source install/setup.bash
```

依赖安装方法见 [ROS 2 rosdep 文档](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Intermediate/Rosdep.rst)。修改源码或配置后重新编译。

## 运行 V3

构建完成后，在仓库根目录执行：

```bash
bash scripts/run_demo.sh v3
```

脚本加载环境并启动 Gazebo、机器人、RViz2、V3 控制器和轨迹监测器，等价于：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch canyon_control canyon_full.launch.py control:=v3
```

机器人生成后会自动开始导航。成功时终端出现“迷宫通关成功”和运行统计；RViz2 的固定坐标系为 `odom`，轨迹话题为 `/canyon/robot_path`。观察完成后按 `Ctrl+C` 关闭本轮仿真。

每轮只运行一个运动控制节点。切换模式前结束上一轮，再启动新场景。

## 其他演示

在仓库根目录选择一项运行：

| 模式 | 命令 |
| --- | --- |
| 基础版迷宫 | `bash scripts/run_demo.sh baseline` |
| V2 自适应迷宫 | `bash scripts/run_demo.sh adaptive` |
| 仅迷宫、机器人及监测 | `bash scripts/run_demo.sh none` |
| 雷达辅助五航点，使用旧地图 | `bash scripts/run_demo.sh waypoints` |
| 普通五航点，使用旧地图 | `bash scripts/run_demo.sh waypoints-basic` |

开环、点到点、雷达处理和局部避障的独立命令见 [基础功能演示](docs/DEMOS.md)。

## 目录

```text
ros2-canyon-maze/
├── src/canyon_control/
│   ├── canyon_control/       # 控制、感知、导航和监测节点
│   ├── launch/               # 三个 ROS 2 启动文件
│   ├── worlds/               # 迷宫、旧地图与测试障碍物
│   ├── rviz/                 # RViz2 配置
│   ├── test/                 # ament 风格检查与节点退出回归测试
│   └── package.xml, setup.py, setup.cfg, resource/
├── scripts/                  # 演示启动与静态检查
├── docs/                     # 结构、演示、检查说明和改进计划
├── LICENSES/                 # 随附第三方文件的许可证
├── THIRD_PARTY_NOTICES.md
└── CHANGELOG.md
```

## 检查与后续开发

```bash
python3 scripts/check_project.py
```

该检查覆盖 Python 语法、元数据一致性、节点入口、XML 和资源文件；不会运行机器人。ROS 2 构建、风格检查与 V3 仿真需分别验证，详见 [检查说明](docs/TESTING.md)。

当前导航代码已在新工作空间中通过依赖检查与构建，并完成 V3 导航：耗时 189.9 秒，到点里程 17.37 米。ROS 2 环境中的风格与退出测试共 6 项通过；许可确定后补齐版权头、启用版权检查，并在本地单独验证。另一次短检查确认 Gazebo、RViz2 和项目节点均正常结束。导航算法、地图和控制参数保持原样，完整结果及其边界见 [验证记录](docs/VERIFICATION.md)。

后续优先做参数配置化、传感器异常与状态机测试，再评估多地图和新仿真平台支持，见 [改进计划](docs/ROADMAP.md)。

## 来源与许可证

项目由个人完成设计、集成调试和实验验证。在提供思路与程序框架后，使用 AI 辅助编写部分基础代码及整理文档。

项目原创代码与文档采用 [MIT 许可证](LICENSE)，版权署名为 `00carmen`。三个原有测试模板保留 Apache-2.0 许可，外部模型与依赖遵循各自许可证，详见 [第三方说明](THIRD_PARTY_NOTICES.md)。

功能包元信息和 Git 提交使用维护者的 GitHub noreply 邮箱。该地址不用于接收咨询；问题反馈和交流请使用 [仓库 Issues](https://github.com/00carmen/ros2-canyon-maze/issues)。
