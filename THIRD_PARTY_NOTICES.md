# 第三方资源说明

## 仓库内的测试模板

以下文件保留其原始 Apache License 2.0 版权头：

| 文件 | 原版权声明 |
| --- | --- |
| `src/canyon_control/test/test_copyright.py` | Copyright 2015 Open Source Robotics Foundation, Inc. |
| `src/canyon_control/test/test_flake8.py` | Copyright 2017 Open Source Robotics Foundation, Inc. |
| `src/canyon_control/test/test_pep257.py` | Copyright 2015 Open Source Robotics Foundation, Inc. |

许可证全文见 [Apache-2.0](LICENSES/Apache-2.0.txt)，来源为 [Apache Software Foundation](https://www.apache.org/licenses/LICENSE-2.0.txt)。这份许可针对相应第三方文件，不代表本项目整体已经选择 Apache-2.0。

`test_copyright.py` 由 `00carmen` 移除了生成模板中的跳过标记，以启用版权检查；该文件增加修改者版权声明与修改说明，继续遵循 Apache-2.0。另两个模板内容保持原样。

## 外部依赖

- TurtleBot3 Burger 的 URDF、SDF 和网格由安装环境中的 `turtlebot3_description`、`turtlebot3_gazebo` 提供；本仓库不附带这些模型文件。参见 [ROBOTIS 仿真文档](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)、[描述包](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble/turtlebot3_description) 和 [仿真包](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/humble/turtlebot3_gazebo)。
- 场景通过 `model://ground_plane` 和 `model://sun` 引用 Gazebo 的模型资源，没有将其实现复制到仓库。
- ROS 2、Gazebo Classic、RViz2 及其他依赖遵循各自的许可证。

本项目原创代码与文档采用 MIT，署名 `00carmen`，见根目录 [LICENSE](LICENSE)。以上第三方文件和依赖的许可证保持各自适用范围。
