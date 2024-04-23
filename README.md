<div align="center">

Depth Anything TensorRT Based On Jetson Orin ROS1
===========================
[![python](https://img.shields.io/badge/python-3.10.12-green)](https://www.python.org/downloads/release/python-31012/)
[![cuda](https://img.shields.io/badge/cuda-11.6-green)](https://developer.nvidia.com/cuda-downloads)
[![trt](https://img.shields.io/badge/TRT-8.6-green)](https://developer.nvidia.com/tensorrt)
[![mit](https://img.shields.io/badge/license-MIT-blue)](https://github.com/spacewalk01/depth-anything-tensorrt/blob/main/LICENSE)
</div>


<p align="center">
  <img src="assets/davis_dolphins_result.gif" height="225px" width="720px" />
</p>


## 🚀 Quick Start

#### C++


Example:
``` shell
mkdir catkin_ws & cd catkin_ws
mkidr src
catkin_make
source devel/setup.bash
roslaunch depth_online run_depth.launch
```


## 🛠️ Build

#### C++

Refer to [depth-anything-tensorrt](https://github.com/spacewalk01/depth-anything-tensorrt.git) and [csdn blog](https://blog.csdn.net/CCChester/article/details/138010704)for C++ environment installation.

## 🤖 Model Preparation

Refer to [depth-anything-tensorrt](https://github.com/spacewalk01/depth-anything-tensorrt.git) for onnx models.

## 👏 Acknowledgement

This project is based on the following projects:
- [depth-anything-tensorrt](https://github.com/spacewalk01/depth-anything-tensorrt.git)
- [Depth-Anything](https://github.com/LiheYoung/Depth-Anything) - Unleashing the Power of Large-Scale Unlabeled Data.
- [TensorRT](https://github.com/NVIDIA/TensorRT/tree/release/8.6/samples) - TensorRT samples and api documentation.
