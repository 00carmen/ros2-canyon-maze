# canyon_control

ROS 2 Humble 下的峡谷迷宫导航功能包。V3 为主要演示版本，同时保留基础版、V2、五航点与基础控制节点。

完整构建、演示和检查说明见仓库根目录 `README.md` 及 `docs/`。

在工作空间根目录完成构建后运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch canyon_control canyon_full.launch.py control:=v3
```

该命令启动迷宫、机器人、RViz2、V3 控制器和轨迹监测器；按 `Ctrl+C` 结束。算法针对随附地图调试，末段依赖预设出口坐标。

维护者为 `00carmen`，原创代码采用随包安装的 MIT 许可证；原有测试模板保留其 Apache-2.0 版权头。维护者 noreply 邮箱用于提交和包元信息，不用于接收咨询。
