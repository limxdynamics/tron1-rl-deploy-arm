# pointfoot-sdk-lowlevel (bundled copy)

**English** | [中文](README_cn.md)

This is the LimX low-level SDK as bundled in the `tron1-rl-deploy-arm` workspace. It is built together with the rest of `src/`, so there is nothing to clone or build here on its own. Follow the root [README](../../README.md): install the dependencies, clone this repository, and run `catkin_make install` at the workspace root.

> Do not clone `pointfoot-sdk-lowlevel`, `pointfoot-gazebo-ros` or `robot-description` into this workspace again. They are already under `src/`, and a second copy gives duplicate catkin packages and a failed build.

## Robot types

This workspace supports the two arm-equipped TRON1 models only: `SF_TRON1A` (sole foot) and `WF_TRON1A` (wheel foot). Set `ROBOT_TYPE` as described in the root README.

## Examples

- C++: [`examples/pf_groupJoints_move.cpp`](examples/pf_groupJoints_move.cpp) and [`examples/pf_joint_move.cpp`](examples/pf_joint_move.cpp)
- Python: [`python3/amd64/example.py`](python3/amd64/example.py)

To use the Python interface, install the bundled wheel for your platform from this directory:

```bash
pip install python3/amd64/limxsdk-*-py3-none-any.whl    # Linux x86_64
pip install python3/aarch64/limxsdk-*-py3-none-any.whl  # Linux aarch64
pip install python3/win/limxsdk-*-py3-none-any.whl      # Windows
```

## Using the SDK on its own

Outside this workspace, use the standalone repository [limxsdk-lowlevel](https://github.com/limxdynamics/limxsdk-lowlevel) (formerly `pointfoot-sdk-lowlevel`). The simulator and robot models are in [tron1-gazebo-ros](https://github.com/limxdynamics/tron1-gazebo-ros) and [tron1-robot-description](https://github.com/limxdynamics/tron1-robot-description).
