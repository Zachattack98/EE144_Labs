import math
import numpy as np
from math import pi, cos, sin

def transform_inverse(mat):
    rotation=mat[0:3,0:3].T
    mat[0:3,0:3]=rotation
    mat[0:3,3]=-rotation.dot(mat[0:3,3])
    return mat

# transform the input vector to skew_symmetric matrix
def skew_symmetric(v):
    return np.matrix([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])

# calculate SO3 from given axis and angular velocity using matrix exponential
def matrix_exponential_so3(w,theta):
    return np.identity(3)+sin(theta)*skew_symmetric(w)+(1-cos(theta))*(np.dot(skew_symmetric(w),skew_symmetric(w)))
    pass

# calculate SE3 from given axis, linear velocity and angular velocity using matrix exponential
def matrix_exponential_se3(w,v,theta):
    mat=np.identity(4)
    mat[0:3,0:3]=matrix_exponential_so3(w,theta)
    mat[0:3,3]=np.dot(np.identity(3)*theta+ \
                     (1-cos(theta))*skew_symmetric(w)+ \
                     (theta-sin(theta))*(np.dot(skew_symmetric(w),skew_symmetric(w))),v.T).ravel()
    return mat
    

def forward_kinematics(joints):

    L=100
    joint1 = joints[0]
    joint2 = joints[1]
    joint3 = joints[2]
    joint4 = joints[3]

    M=np.identity(4)
    M[0:3,3]=[0,L,0]

    # calculate the transformation matrix using matrix exponential according to the fixed frame.
    T=matrix_exponential_se3(np.array([0,0,0]),np.array([1,0,0]),joint1).dot( \
        matrix_exponential_se3(np.array([0,0,0]),np.array([0,1,0]),joint2).dot(\
        matrix_exponential_se3(np.array([0,1,0]),np.array([0,0,0]),joint3).dot(\
        matrix_exponential_se3(np.array([1,0,0]),np.array([0,0,-L]),joint4).dot(\
            M))))
    
    return T

# debug script
if __name__ == '__main__':
    print(forward_kinematics([5,5,math.pi/2,math.pi/2]))
