# Eva-Tracker

## Introduction

__Eva-Tracker__ is an ESDF-update-free, Visibility-Aware trajectory planning framework for aerial tracking.

## Paper

__Eva-Tracker: ESDF-update-free, Visibility-aware Planning with Target Reacquisition for Robust Aerial Tracking__. Accepted by __ICRA 2026__. The full paper will be published soon.

Author: [Yue Lin](https://github.com/Yue-0), Yang Liu, Dong Wang, Huchuan Lu.

## Quick Start

In Ubuntu20.04 & ROS-noetic:

```shell
git clone https://github.com/Yue-0/Eva-Tracker.git && cd Eva-Tracker
```

### Run in simulation

If `quadrotor_msgs` is not available in your environment, please unzip the simple version:

```shell
unzip quadrotor_msgs.zip
```

Compile this project:

```shell
catkin_make
```

Start the simulation environment:

```shell
source devel/setup.zsh
roslaunch simulator simulation.launch
```

Start the tracker in another terminal, and the tracker will track the target autonomously:

```shell
source devel/setup.zsh
roslaunch tracker tracking.launch
```

Use `2D Nav Goal` to control the target movement.

### Run in real world

Our drone platform is shown in the figure, equipped with a Livox Mid-360 LiDAR and an Intel RealSense depth camera. The computing platform is Jetson Orin NX.

![Our Drone Platform](uav.png)

For the first run, make sure your device supports `TensorRT` and then export the object detection model, which would take a while:

```shell
python onnx2trt.py
```

In line 4 of [src/tracker/CMakeLists.txt](src/tracker/CMakeLists.txt), set the value of `REAL_WORLD` to `TRUE`:

```CMake
set(REAL_WORLD TRUE)
```

Make sure your device supports CUDA, then compile the project.

```shell
catkin_make
```

Before launch the tracker, please enable [PX4](https://github.com/mavlink/mavros) and [FasterLIO](https://github.com/gaoxiang12/faster-lio):

```shell
sudo -S chmod 777 /dev/tty*
roslaunch mavros px4.launch
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0
roslaunch faster_lio mapping_mid360.launch
```

Launch the tracker:

```shell
roslaunch tracker realworld.launch
```

If you turn the 8th channel of the remote controller from down to the middle, the drone will automatically take off to a height of 1.2m. If you turn it from the middle to up, the drone will automatically follow the target. Conversely, if you turn it from up to the middle, the drone will automatically hover. If you turn it down, the drone will automatically land.

## Acknowledgements

We use the pre-trained model of [YOLOv11](https://github.com/ultralytics/ultralytics)-Pose for human keypoint detection.

We use [LBFGS-Lite](https://github.com/ZJU-FAST-Lab/LBFGS-Lite) to solve numerical optimization problems.

Our real-world drone is provided by __Differential Robotics__.
