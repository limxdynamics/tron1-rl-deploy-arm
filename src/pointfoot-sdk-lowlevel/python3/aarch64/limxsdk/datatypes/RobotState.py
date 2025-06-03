import sys

class RobotState(object):
    __slots__ = ['stamp','tau','q','dq','motor_names']
    def __init__(self):
        self.stamp = 0
        self.tau = []
        self.q = []
        self.dq = []
        self.motor_names = []