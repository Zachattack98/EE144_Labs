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
    x = position[0]	#x direction points east
    y = position[1]     #y direction points north
    z = position[2]     #z direction points straight up, like a pole in the middle

    #Finding joint 1 using theta 1 only
    theta1 = atan2(y,x)
    joint1 = theta1

    #Finding joint 2 using theta 2 and other terms
    #y_1 = (x/cos(theta1))
    coord_XY = sqrt(x**2 + y**2)       #result of the hypotenuse for square rooting the coordinates x and y
    hypG = sqrt(coord_XY**2 + (z - 0.065 - 0.0389)**2)    #hypotenuse of gamma, or red and blue triangles; 65 + 38.9 is the offset
    alpha = atan2(0.05, 0.15)	#convert the given values from mm to cm
    beta1 = acos(((0.1581**2) + (hypG**2) - (0.15**2)) / (2 * 0.1581 * hypG))
    gamma = atan2(z,x)
    
    theta2 = (pi/2) - alpha  - beta1 - gamma
    joint2 = theta2

    #Finding joint 3 using theta 3
    beta2 = acos(((0.1581**2) + (0.15**2) - (hypG**2)) / (2 * 0.1581 * 0.15))
    betaTemp = pi - beta2        #imagine cutting the yellow hypotenuse angle into two smaller angles, resulting in two new triangles. Gives us larger yellow triangle's hypotenuse angle.
    
    theta3 = pi - alpha - (pi/2) - betaTemp   #(all angles of yellow triangle) - (alpha) - (right angle form at top of yellow triangle) - (betaTemp) = smallest angle of snaller yellow triangle which is mirror of theta3.
    joint3 = theta3

    return [joint1, joint2, joint3]
