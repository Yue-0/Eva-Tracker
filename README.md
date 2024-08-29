# Eva-Tracker

__English__| [简体中文](README_cn.md)

## Introduction

__Eva-Tracker__ is an efficient trajectory planning framework for aerial tracking that does not rely on environmental ESDF and safe corridors.

<!-- __Paper__: Eva-Tracker: An ESDF-update-free Approach for Aerial Tracking with Visibility-aware Planning (ICRA 2025 submission). -->

<!-- __Video__: Coming soon. -->

## Quick Start

This project needs to run under Ubuntu20.04 & ROS-noetic.

```shell
git clone https://github.com/Yue-0/Eva-Tracker.git
cd Eva-Tracker
catkin_make
```

Start the simulation environment:

```shell
source devel/setup.bash
roslaunch simulator simulation.launch
```

Start the tracker in another terminal, and the tracker will track the target autonomously:

```shell
source devel/setup.bash
roslaunch tracker tracking.launch
```

The following command is used to reproduce our simulation experiment:

```shell
rosbag play bag/replay.bag
```

Or, use the following command to control the target movement:

```shell
rosbag play --loop bag/goal.bag
```

Or, you can directly use `2D Nav Goal` to control the target movement.

## Acknowledgements

We use [LBFGS-Lite](https://github.com/ZJU-FAST-Lab/LBFGS-Lite) to solve numerical optimization problems.
