# LimX SDK guide

**English** | [中文](README_cn.md)

## 1. Set up the development environment

On the algorithm developer's own computer, we recommend setting up an algorithm development environment based on ROS Noetic on Ubuntu 20.04. ROS provides a set of tools and libraries, such as core libraries, communication libraries and simulation tools (such as Gazebo), which make it much easier to develop, test and deploy robot algorithms. Together they give you a rich and complete environment for algorithm development.

You can also develop your motion control algorithms in other environments without ROS. Our motion control development interface is a dependency-free SDK based on standard C++11 and Python. It can be called across operating systems and platforms, which gives developers more flexibility.

To install ROS Noetic, follow https://wiki.ros.org/noetic/Installation/Ubuntu and choose "ros-noetic-desktop-full". After ROS Noetic is installed, run the following in a Bash terminal to install the libraries the development environment depends on:

```
sudo apt-get update
sudo apt install ros-noetic-urdf \
                 ros-noetic-kdl-parser \
                 ros-noetic-urdf-parser-plugin \
                 ros-noetic-hardware-interface \
                 ros-noetic-controller-manager \
                 ros-noetic-controller-interface \
                 ros-noetic-controller-manager-msgs \
                 ros-noetic-control-msgs \
                 ros-noetic-ros-control \
                 ros-noetic-gazebo-* \
                 ros-noetic-rqt-gui \
                 ros-noetic-rqt-controller-manager \
                 ros-noetic-plotjuggler* \
                 cmake build-essential libpcl-dev libeigen3-dev libopencv-dev libmatio-dev \
                 python3-pip libboost-all-dev libtbb-dev liburdfdom-dev liborocos-kdl-dev -y
```

## 2. Create a workspace

Follow these steps to create an algorithm development workspace:

- Open a Bash terminal.

- Create a new directory for the workspace. For example, create a directory named "limx_ws" in your home directory:

  ```
  mkdir -p ~/limx_ws/src
  ```

- Download the motion control development interface:

  ```
  cd ~/limx_ws/src
  git clone https://github.com/limxdynamics/pointfoot-sdk-lowlevel.git
  ```

- Download the Gazebo simulator:

  ```
  cd ~/limx_ws/src
  git clone https://github.com/limxdynamics/pointfoot-gazebo-ros.git
  ```

- Download the robot description files:

  ```
  cd ~/limx_ws/src
  git clone https://github.com/limxdynamics/robot-description.git
  ```

- Download the visualization and debugging tool:

  ```
  cd ~/limx_ws/src
  git clone https://github.com/limxdynamics/robot-visualization.git
  ```

- Build the workspace:

  ```
  cd ~/limx_ws
  catkin_make install
  ```


## 3. Python motion control development interface

### 3.1 Overview

The [Python motion algorithm development interface](https://github.com/limxdynamics/pointfoot-sdk-lowlevel/tree/master/python3) offers the same functions as the C++ one, so developers who are not familiar with C++ can develop motion control algorithms in Python. Python is easy to learn, with a clean, readable syntax and a rich set of third-party libraries, so developers can get started and implement algorithms quickly. Through the Python interface, developers can use Python's dynamic features for rapid prototyping and experimental validation, which speeds up iterating on and optimizing algorithms. Python's cross-platform support and strong ecosystem also let motion algorithms run on a wider range of platforms and environments. In addition, Python's flexibility makes it quick to deploy RL (reinforcement learning) models to simulation and to the real robot: developers can easily integrate RL models into various simulation platforms and real hardware, and iterate on and validate algorithm performance quickly.

### 3.2 Install the motion control development library

Choose the command for your operating system:

- Linux x86_64

  ```Bash
  pip install python3/amd64/limxsdk-*-py3-none-any.whl
  ```

- Linux aarch64

  ```Bash
  pip install python3/aarch64/limxsdk-*-py3-none-any.whl
  ```

- Windows

  ```Bash
  pip install python3/win/limxsdk-*-py3-none-any.whl
  ```

### 3.3 Example

Python interface example: https://github.com/limxdynamics/pointfoot-sdk-lowlevel/blob/master/python3/amd64/example.py
