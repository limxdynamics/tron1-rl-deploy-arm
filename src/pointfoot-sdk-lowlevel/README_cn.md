# pointfoot-sdk-lowlevel（内置副本）

[English](README.md) | **中文**

这是 `tron1-rl-deploy-arm` 工作空间内置的 LimX 底层 SDK，随 `src/` 下的其他包一起编译，不需要单独克隆或编译。请按根目录 [README](../../README_cn.md) 操作：安装依赖、克隆本仓库，然后在工作空间根目录执行 `catkin_make install`。

> 不要再把 `pointfoot-sdk-lowlevel`、`pointfoot-gazebo-ros` 或 `robot-description` 克隆进这个工作空间。它们已经在 `src/` 下，重复的副本会导致 catkin 包重名、编译失败。

## 机型

本工作空间只支持两款带机械臂的 TRON1：`SF_TRON1A`（双足）和 `WF_TRON1A`（轮足）。`ROBOT_TYPE` 的设置方法见根目录 README。

## 示例程序

- C++：[`examples/pf_groupJoints_move.cpp`](examples/pf_groupJoints_move.cpp)、[`examples/pf_joint_move.cpp`](examples/pf_joint_move.cpp)
- Python：[`python3/amd64/example.py`](python3/amd64/example.py)

使用 Python 接口时，在本目录下安装对应平台的内置 wheel：

```bash
pip install python3/amd64/limxsdk-*-py3-none-any.whl    # Linux x86_64
pip install python3/aarch64/limxsdk-*-py3-none-any.whl  # Linux aarch64
pip install python3/win/limxsdk-*-py3-none-any.whl      # Windows
```

## 单独使用 SDK

在本工作空间之外，请使用独立仓库 [limxsdk-lowlevel](https://github.com/limxdynamics/limxsdk-lowlevel)（原 `pointfoot-sdk-lowlevel`）。仿真器和机器人模型分别在 [tron1-gazebo-ros](https://github.com/limxdynamics/tron1-gazebo-ros) 和 [tron1-robot-description](https://github.com/limxdynamics/tron1-robot-description)。
