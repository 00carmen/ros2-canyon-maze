# 检查与复现

静态检查、安装检查和仿真验证分别记录。静态通过不代表 ROS 2 运行通过，原有课程截图也不代表本准备版已完成从零构建。

## 1. 静态检查

在仓库根目录执行：

```bash
python3 scripts/check_project.py
```

需要 Python 3.10 或更高版本，无需安装 ROS 2。检查 Python 3.10 语法兼容性、XML、功能包版本与描述、节点注册、启动文件引用及资源安装清单。公开维护信息和项目许可证尚未完成时会显示警告，不会伪装成已具备发布条件。

发布前可使用严格模式：

```bash
python3 scripts/check_project.py --release
```

严格模式将未完成的维护信息或项目许可证视为失败。

## 2. 新工作空间构建

将完整项目复制到一个新的目录，避免沿用课程工作空间的 `build/`、`install/` 和 `log/`。打开没有加载旧项目环境的新终端，只加载 `/opt/ros/humble/setup.bash`，按 README 的步骤安装依赖并构建。

构建后确认实际加载的是新目录：

```bash
source install/setup.bash
ros2 pkg prefix canyon_control
ros2 pkg executables canyon_control
ros2 launch canyon_control canyon_full.launch.py --show-args
```

功能包前缀必须指向本次新建工作空间的 `install`。应能找到 11 个节点入口，且安装目录包含 3 个启动文件、2 个 world、1 个障碍物 SDF、1 个 RViz2 配置及 `LICENSE`。

## 3. 风格检查与退出回归测试

```bash
colcon test --packages-select canyon_control --event-handlers console_direct+
colcon test-result --verbose
```

`test/` 包含原有版权、flake8、PEP257 检查，以及新增的节点退出回归测试。后者在真实 ROS 2 子进程中分别向 V3 和监测节点发送 SIGINT、SIGTERM，检查退出码和异常输出，共四个用例；无需启动 Gazebo。

MIT 许可证及版权头已补齐，版权检查已启用。没有 ROS 2 的环境不能执行退出回归测试。当前仍缺少开口判断、传感器异常和导航成功率等行为测试；跳过或未执行不计作通过。

## 4. V3 完整运行

```bash
bash scripts/run_demo.sh v3
```

记录启动日志，确认 Gazebo 中只有本轮机器人与控制器。至少检查：

- 机器人从随附迷宫的起点开始，雷达、里程计和 RViz2 轨迹正常。
- 控制器输出“迷宫通关成功”和终点误差，状态到达 `FINISHED`。
- `/canyon/monitor_summary` 给出完成摘要，记录耗时与累计里程。
- 观察全程是否发生碰撞或卡住；到达终点的日志不能单独证明全程没有碰撞。
- 完成后机器人停车，`Ctrl+C` 能结束该轮仿真。

控制器使用实际经过时间判定 450 秒超时。虚拟机暂停、资源不足或过低的实时因子可能影响结果。先保留日志定位原因，再决定是否调整算法。

## 本地准备版的验证边界

当前代码保持导航算法、地图和参数不变，已通过依赖检查、新工作空间构建、风格与退出回归测试，并完成一次修订后的 V3 到点运行。另一次短运行验证了整套仿真的正常退出。许可与署名整理后单独执行本地版权和静态检查，没有重新运行完整仿真。实机、任意地图和批量成功率尚未验证。各轮结果和局限见 [验证记录](VERIFICATION.md)。
