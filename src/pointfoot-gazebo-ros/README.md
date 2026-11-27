# pointfoot-gazebo-ros (bundled copy)

**English** | [中文](README_cn.md)

This is the Gazebo simulator for TRON1 (`pointfoot_gazebo` and `pointfoot_sdk_sim`) as bundled in the `tron1-rl-deploy-arm` workspace. It is built together with the rest of `src/`, so there is nothing to clone or build here on its own. Follow the root [README](../../README.md): install the dependencies, clone this repository, run `catkin_make install` at the workspace root, then start the simulation with `roslaunch robot_hw pointfoot_hw_sim.launch`.

> Do not clone `pointfoot-gazebo-ros`, `pointfoot-sdk-lowlevel` or `robot-description` into this workspace again. They are already under `src/`, and a second copy gives duplicate catkin packages and a failed build.

## Robot types

This workspace supports the two arm-equipped TRON1 models only: `SF_TRON1A` (sole foot) and `WF_TRON1A` (wheel foot). Their models are in [`src/robot-description/pointfoot/`](../robot-description/pointfoot). Set `ROBOT_TYPE` as described in the root README.

![The robot moving in the Gazebo simulator](doc/simulator.gif)

## Using the simulator on its own

Outside this workspace, use the standalone repository [tron1-gazebo-ros](https://github.com/limxdynamics/tron1-gazebo-ros) (formerly `pointfoot-gazebo-ros`), together with [limxsdk-lowlevel](https://github.com/limxdynamics/limxsdk-lowlevel) and [tron1-robot-description](https://github.com/limxdynamics/tron1-robot-description).
