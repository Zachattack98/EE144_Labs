import numpy as np
from math import pi, cos, sin
import modern_robotics as mr

def forward_kinematics(joints):
    # input: joint angles [joint1, joint2, joint3]
    # output: the position of end effector [x, y, z]
    # add your code here to complete the computation

    link1z = 0.065
    link2z = 0.039
    link3x = 0.050
    link3z = 0.150
    link4x = 0.150
    joint1 = joints[0]	#joint angles
    joint2 = joints[1]
    joint3 = joints[2]

    x = (link1z * cos(joint1)) + (link2z * cos(joint1 + joint2)) + (link3z * cos(joint1 + joint2 + joint3))

    y = (link1z * sin(joint1)) + (link2z * sin(joint1 + joint2)) + (link3z * sin(joint1 + joint2 + joint3))

    z = joint1 + joint2 + joint3

    return [x, y, z]
