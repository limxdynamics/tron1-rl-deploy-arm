"""
@brief This file contains the declarations of classes related to the control of robots.

@file Robot.py

© [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
"""

import sys
from typing import Callable, Any
import limxsdk.datatypes as datatypes
import limxsdk.robot as robot

class Robot(object):
    """
    @class Robot
    @brief Represents a robot with various functionalities.

    This class provides an interface to interact with different types of robots.
    """

    def __init__(self, robot_type: robot.RobotType, is_sim: bool = False):
        """
        @brief Initializes a Robot object with a specified type and simulation mode.

        This constructor initializes a robot instance based on the provided robot type and 
        whether it's for simulation or real-world operation. It sets up the necessary components
        for the robot's operation, such as the robot's IP address and its native instance.

        @param robot_type: Specifies the type of the robot. Possible values are:
                          - robot.RobotType.PointFoot: Represents a PointFoot robot.
                          - robot.RobotType.Wheellegged: Represents a Wheellegged robot.
                          
        @param is_sim: Boolean flag indicating whether the robot is running in simulation mode (True)
                      or real-world mode (False). The default value is False (real-world mode).

        The following attributes are initialized:
        @attribute robot_ip: Default IP address for the robot, set to '127.0.0.1'.
        @attribute robot: A native instance of the robot, either a PointFoot or Wheellegged type,
                          based on the provided `robot_type` argument.

        @return None
        """

        __slots__ = ['robot', 'robot_ip']

        # Default robot IP address
        self.robot_ip = '127.0.0.1'

        # Create a native robot instance based on the specified type
        if robot_type == robot.RobotType.PointFoot:
            self.robot = robot.RobotNative("PointFoot", is_sim)
        elif robot_type == robot.RobotType.Wheellegged:
            self.robot = robot.RobotNative("Wheellegged", is_sim)

    def init(self, robot_ip: str = "127.0.0.1"):
        """
        @brief Initializes the robot with a specified IP address.

        @param robot_ip: IP address of the robot.
        @return: True if initialization is successful, False otherwise.
        """
        self.robot_ip = robot_ip
        return self.robot.init(self.robot_ip)

    def getMotorNumber(self):
        """
        @brief Gets the number of motors of the robot.

        @return: Number of motors.
        """
        return self.robot.getMotorNumber()

    def subscribeImuData(self, callback: Callable[[datatypes.ImuData], Any]):
        """
        @brief Subscribes to receive updates on the robot's imu.

        @param callback: Callable[[datatypes.ImuData], Any]: 
                  Callback function to handle imu updates.
        @return: Subscription status.
        """
        return self.robot.subscribeImuData(callback)

    def subscribeRobotState(self, callback: Callable[[datatypes.RobotState], Any]):
        """
        @brief Subscribes to receive updates on the robot's state.

        @param callback: Callable[[datatypes.RobotState], Any]: 
                  Callback function to handle robot state updates.
        @return: Subscription status.
        """
        return self.robot.subscribeRobotState(callback)

    def publishRobotCmd(self, cmd: datatypes.RobotCmd):
        """
        @brief Publishes a robot command.

        @param cmd: Robot command to be published.
        @return: Status of the command publication.
        """
        return self.robot.publishRobotCmd(cmd)

    def subscribeSensorJoy(self, callback: Callable[[datatypes.SensorJoy], Any]):
        """
        @brief Subscribes to receive sensor joy updates.

        @param callback: Callable[[datatypes.SensorJoy], Any]: 
                  Callback function to handle sensor joy updates.
        @return: Subscription status.
        """
        return self.robot.subscribeSensorJoy(callback)

    def subscribeDiagnosticValue(self, callback: Callable[[datatypes.DiagnosticValue], Any]):
        """
        @brief Subscribes to receive diagnostic value updates.

        @param callback (Callable[[datatypes.DiagnosticValue], Any]): 
                  Callback function to handle diagnostic value updates.
        @return: Subscription status.
        """
        return self.robot.subscribeDiagnosticValue(callback)

    def setRobotLightEffect(self, effect: datatypes.LightEffect):
        """
        @brief Sets the robot's light effect.

        This method configures the robot's light effect based on the provided effect parameter.
        The `effect` parameter should be an instance of the `LightEffect` enum, which provides 
        predefined values for different light effects. The enum value is converted to its integer 
        representation before being sent to the robot's API.

        @param effect: An instance of the `LightEffect` enum representing the desired robot light effect. 
                      The method converts this enum value to an integer to be compatible with the underlying 
                      robot API. Refer to the `LightEffect` enum for valid values. For example:
                      - `datatypes.LightEffect.STATIC_RED` corresponds to a static red light.
                      - `datatypes.LightEffect.FAST_FLASH_BLUE` corresponds to a fast-flashing blue light.

        @return: `True` if the light effect was successfully set, `False` otherwise. The return value indicates
                whether the operation was successful.
        """
        # Convert the enum to its integer value and call the robot's method
        return self.robot.setRobotLightEffect(effect.value)

    def subscribeRobotCmdForSim(self, callback: Callable[[datatypes.RobotCmd], Any]):
        """
        @brief Subscribes to receive robot command updates for simulation.

        @param callback: Callable[[datatypes.RobotCmd], Any]: 
                      Callback function to handle robot command updates.
        @return: Subscription status.
        """
        return self.robot.subscribeRobotCmdForSim(callback)
    
    def publishRobotStateForSim(self, state: datatypes.RobotState):
        """
        @brief Publishes the robot's state for simulation.

        @param state: Robot state to be published.
        @return: Status of the state publication.
        """
        return self.robot.publishRobotStateForSim(state)

    def publishImuDataForSim(self, imudata: datatypes.ImuData):
        """
        @brief Publishes IMU data for simulation.

        @param imudata: IMU data to be published.
        @return: Status of the IMU data publication.
        """
        return self.robot.publishImuDataForSim(imudata)
