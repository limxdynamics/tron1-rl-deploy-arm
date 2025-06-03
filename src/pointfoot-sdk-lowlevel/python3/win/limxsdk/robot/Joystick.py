"""
@file Joystick.py

© [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
"""

import limxsdk.robot as robot
import limxsdk.datatypes as datatypes

class Joystick(object):
    def __init__(self, ip: str = "127.0.0.1"):
        """
        Initializes the Joystick class.

        Args:
            ip (str): The IP address of the joystick communication network. Default is "127.0.0.1".
        """
        self.native = robot.JoystickNative()
        self.native.init(ip)

    def publishSensorJoy(self, sensor_joy: datatypes.SensorJoy):
        """
        Publishes joystick sensor data.

        Args:
            sensor_joy (datatypes.SensorJoy): The sensor joy data to be published.

        Returns:
            bool: True if the desired publishing rate was achieved for the cycle; False otherwise.

        This method handles the publishing of joystick sensor data and manages timing
        to ensure the desired publishing rate is maintained.
        """
        return self.native.publishSensorJoy(sensor_joy)
