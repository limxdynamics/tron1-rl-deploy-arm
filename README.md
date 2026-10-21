# English | [中文](README_cn.md)
# Deployment of Training Results

> 🧭 **Part of the TRON1 open-source ecosystem.** All TRON1 repositories and where to start: [TRON1 open-source ecosystem](https://github.com/limxdynamics/tron1-rl-isaacgym#tron1-open-source-ecosystem).

## 1. Deployment Environment Setup

**Install ROS Noetic:**  
We recommend building an algorithm development environment based on ROS Noetic on Ubuntu 20.04. ROS provides a suite of tools and libraries—such as core libraries, communication frameworks, and simulation tools (e.g., Gazebo)—which greatly facilitate the development, testing, and deployment of robotic algorithms. These resources offer users a rich and complete development environment.

To install ROS Noetic, please refer to the official documentation:  
👉 [ROS Noetic Installation on Ubuntu](https://wiki.ros.org/noetic/Installation/Ubuntu)  
Make sure to choose the **`ros-noetic-desktop-full`** version.

After installing ROS Noetic, run the following shell commands in a Bash terminal to install the required dependencies:

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

---

## 2. Compilation and Execution

This project is implemented based on the [`ros_control`](https://wiki.ros.org/ros_control) framework. Follow the steps below to compile and run it:

### Step 1: Open a Bash terminal.

### Step 2: Clone the source code repository:

```bash
git clone https://github.com/limxdynamics/tron1-rl-deploy-arm.git
```

### Step 3: Compile the project:

```bash
cd tron1-rl-deploy-arm
catkin_make install
```

---

## 3. Set Robot Type

Use the following command to list available robot models:

```bash
tree -L 1 src/robot-description/pointfoot
```

Example output:
```
src/robot-description/pointfoot
├── SF_TRON1A
└── WF_TRON1A
```

Choose the appropriate model according to your actual robot. For example, to use `SF_TRON1A`, run:

```bash
echo 'export ROBOT_TYPE=SF_TRON1A' >> ~/.bashrc && source ~/.bashrc
```

---

## 4. Launch Virtual Joystick

Open a new terminal window and clone the virtual joystick repository:

```bash
git clone https://github.com/limxdynamics/robot-joystick.git
```

---

## 5. Run Simulation

Start the Gazebo simulator with the following commands. Once Gazebo is launched, press `Ctrl + Shift + R` in the simulator window to make the robot start moving:

```bash
source install/setup.bash
roslaunch robot_hw pointfoot_hw_sim.launch
```

---

## 6. Start the Virtual Joystick

Run the virtual joystick:

```bash
./robot-joystick/robot-joystick
```

Once started, you can control the robot using the following keys:

- Arrow keys (↑↓←→): Move the robot forward, backward, and turn left/right
- Numpad `8` / `5`: Raise / Lower the robot
- Keys `W`, `A`, `S`, `D`: Translate the robot arm in 4 directions
- Keys `I`, `J`, `K`, `L`: Rotate the robot arm in 4 directions

For example, the effect when using a bipedal robot is shown below.

---