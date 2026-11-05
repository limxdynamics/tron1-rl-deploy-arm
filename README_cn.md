<p align="center">
  <a href="README.md">English</a> | <b>中文</b>
</p>

<!-- limx-community:start -->
<!-- Managed by limxdynamics/limxdynamics/.github/community/sync_community.py; QR code: doc/wechat_qr.png in that repo -->
<p align="center"><a href="https://github.com/limxdynamics/limxdynamics/blob/main/doc/wechat_qr.png"><img src="https://img.shields.io/badge/%E5%BE%AE%E4%BF%A1%E5%BC%80%E5%8F%91%E8%80%85%E7%BE%A4-%E6%89%AB%E7%A0%81%E5%8A%A0%E5%85%A5%20%C2%B7%20%E8%A6%86%E7%9B%96%E5%85%A8%E9%83%A8%E5%BC%80%E6%BA%90%E9%A1%B9%E7%9B%AE-07C160?style=for-the-badge&logo=wechat&logoColor=white" alt="微信开发者社区"></a></p>
<!-- limx-community:end -->

# 训练结果部署

> **TRON1 开源生态的一部分。** 上手路线和其他 TRON1 仓库：[TRON1 快速上手](https://github.com/limxdynamics/limxdynamics/blob/main/doc/quickstart_cn.md#tron1)。



## 1. 部署环境配置

- 安装ROS Noetic：我们推荐在Ubuntu 20.04操作系统上建立基于ROS Noetic的算法开发环境。ROS提供了一系列工具和库，如核心库、通信库和仿真工具（如Gazebo），极大地便利了机器人算法的开发、测试和部署。这些资源为用户提供了一个丰富而完整的算法开发环境。ROS Noetic 安装请参考文档：https://wiki.ros.org/noetic/Installation/Ubuntu ，选择“ros-noetic-desktop-full”进行安装。ROS Noetic 安装完成后，Bash终端输入以下Shell命令，安装开发环境所依赖的库：

    ```bash
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
                 ros-noetic-robot-state-* \
                 ros-noetic-joint-state-* \
                 ros-noetic-rqt-gui \
                 ros-noetic-rqt-controller-manager \
                 ros-noetic-plotjuggler* \
                 cmake build-essential libpcl-dev libeigen3-dev libopencv-dev libmatio-dev \
                 python3-pip libboost-all-dev libtbb-dev liburdfdom-dev liborocos-kdl-dev -y
    ```

    

## 2. 编译运行

项目基于ros_control (https://wiki.ros.org/ros_control) 框架实现。可以按照以下步骤，编译运行：

- 打开一个Bash终端。

- 下载源代码：
    ```Bash
    git clone https://github.com/limxdynamics/tron1-rl-deploy-arm.git
    ```
    
- 编译工程：
    ```Bash
    cd tron1-rl-deploy-arm
    catkin_make install
    ```
    
- 选择机器人类型

  - 通过 Shell 命令 `tree -L 1 src/robot-description/pointfoot ` 列出可用的机器人类型：
  
    ```
    src/robot-description/pointfoot
    ├── SF_TRON1A
    └── WF_TRON1A
    ```
    
  - 以`SF_TRON1A`（请根据实际机器人类型进行替换）为例，设置机器人型号类型：
  
    ```
    echo 'export ROBOT_TYPE=SF_TRON1A' >> ~/.bashrc && source ~/.bashrc
    ```
  
- 打开新的命令窗口，下载虚拟遥控器：
    ```Bash
    git clone https://github.com/limxdynamics/robot-joystick.git
    ```
- 运行仿真

  通过运行Shell命令启动Gazebo仿真器，然后在仿真器窗口中按 `Ctrl + Shift + R`，机器人将开始移动。
  
  
  ```
  source install/setup.bash
  roslaunch robot_hw pointfoot_hw_sim.launch
  ```

  开启虚拟遥控器
    ```Bash
    ./robot-joystick/robot-joystick
    ```
  开启虚拟遥控器后，您可以通过键盘↑↓←→控制机器人前后左右移动，小键盘8/5控制机器人升高/降低，WASD控制机械臂四方向平移，IJKL控制机械臂四方向旋转，以双足为例效果如下

  ![](doc/simulator.gif)

