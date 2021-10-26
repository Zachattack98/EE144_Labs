import numpy as np
from math import pi, cos, sin, atan, atan2, sqrt, acos

def inverse_kinematics(position):
    # input: the position of end effector [x, y, z]
    # output: joint angles [joint1, joint2, joint3]
    # add your code here to complete the computation

    link1z = 0.065
    link2z = 0.039
    link3x = 0.050
    link3z = 0.150
    link4x = 0.150
    x = position[0]
    y = position[1]
    z = position[2]

    #Finding joint 1 using theta 1 only
    theta1 = atan2(y,x)
    joint1 = theta1

    #Finding joint 2 using theta 2 and other terms
    sqX = sqrt(x**2 + y**2)
    alpha = atan2(0.05, 0.15)	#convert the given values from mm to cm
    beta1 = acos(((0.1501**2) + (sqX**2) + (z**2) - (0.15**2)) / (2 * 0.1581 * sqrt(sqX**2 + z**2)))
    gamma = atan2(z,x)

    theta2 = (pi/2) - alpha  - beta1 - gamma
    joint2 = theta2

    #Finding joint 3 using theta 3
    beta2 = acos(((0.1501**2) + (0.15**2) - (sqX**2) + (z**2)) / (2 * 0.1581 + 0.15))
    betaTemp = pi - beta2
    theta3 = pi - alpha - pi/2 - betaTemp
    joint3 = theta3

    return [joint1, joint2, joint3]
