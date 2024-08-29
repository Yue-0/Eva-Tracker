# Eva-Tracker

[English](README.md) | __简体中文__

## 介绍

__Eva-Tracker__ 是一种用于空中跟踪的高效轨迹规划框架，它不依赖环境 ESDF 和安全走廊。

<!-- __论文__: Eva-Tracker: An ESDF-update-free Approach for Aerial Tracking with Visibility-aware Planning （ICRA 2025 在投）。 -->

<!-- __视频__: 即将发布。 -->

## 快速开始

本项目需在 Ubuntu20.04 & ROS-noetic 下运行。

```shell
git clone https://github.com/Yue-0/Eva-Tracker.git
cd Eva-Tracker
catkin_make
```

开启仿真环境：

```shell
souce devel/setup.bash
roslaunch simulator simulation.launch
```

在另一个终端开启跟踪器，跟踪器将自主跟踪目标：

```shell
source devel/setup.bash
roslaunch tracker tracking.launch
```

以下命令用于重现我们的模拟实验：

```shell
rosbag play bag/replay.bag
```

或者，使用以下命令控制目标运动：

```shell
rosbag play --loop bag/goal.bag
```

或者，你可以直接使用 `2D Nav Goal` 控制目标运动。

## 致谢

我们使用 [LBFGS-Lite](https://github.com/ZJU-FAST-Lab/LBFGS-Lite) 求解数值优化问题。
