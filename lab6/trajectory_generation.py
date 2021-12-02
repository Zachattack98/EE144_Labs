

#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D


class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        # reset odometry to zero
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        for i in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()
        
        # subscribe to odometry
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.previous_waypoint = [0,0]
        self.previous_velocity = [0,0]
        self.vel_ref = 0.3
        self.vel = Twist()

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')


    def run(self):
        waypoints = [[0.5, 0], [0.5, -0.5], [1, -0.5], [1, 0], [1, 0.5], [1.5, 0.5], [1.5, 0], [1.5, -0.5], [1, -0.5], [1, 0], [1, 0.5], [0.5, 0.5], [0.5, 0], [0, 0], [0, 0]]
        for i in range(len(waypoints)-1):
            self.move_to_point(waypoints[i], waypoints[i+1])

    def move_to_point(self, current_waypoint, next_waypoint):
        # generate polynomial trajectory and move to current_waypoint      
        # next_waypoint is to help determine the velocity to pass current_waypoint
        #determine boundary conditions for the position for x and y
        px_start = self.previous_waypoint[0]
        px_end = current_waypoint[0]
        #print(px_end)
        py_start = self.previous_waypoint[1]
        py_end = current_waypoint[1]

        #determine boundary  conditions for velocity
        #need to compute the angle of the robot in order to change the x,y velocities
        vx_start = self.previous_velocity[0]    #velocity*cos()
        vy_start = self.previous_velocity[1]    #velocity*sin()

        #decompose the velocity
        dx = next_waypoint[0] - current_waypoint[0]
        dy = next_waypoint[1] - current_waypoint[1]
        angle = atan2(dy,dx)

        if(current_waypoint[0] == 0 and current_waypoint[1] == 0 and next_waypoint[0] == 0 and next_waypoint[1] == 0):  #here the robot should stop moving forward since vx and vy take part in determining linear velocity
            vx_end = 0  
            vy_end = 0
        else:       
            vx_end = self.vel_ref*cos(angle)  
            vy_end = self.vel_ref*sin(angle)

        print(vx_end)
        print(vy_end)
        #solve the polynomial coefficient
        #going to get coefficients for x and coefficients for y
        T = 2
        Kp = 5
        coeff_x = self.polynomial_time_scaling_3rd_order(px_start, vx_start, px_end, vx_end, T)
        coeff_y = self.polynomial_time_scaling_3rd_order(py_start, vy_start, py_end, vy_end, T)
        #print(coeff_x)
        #print(coeff_y)
        #now we want the robot to move, we have the coefficients now which can be used to find the velocity.
        #The velocity can now be sent to twist for a specified range of time values hint: use for loop
        
        for i in range(0, 10*T):
            #send the velocity
            #we can play with t and extend the time to see what happens
            t = i * 0.1 
            #compute Vx_end, Vy_end; get vx vy, then we go from velocity to angle
            vx_end = np.dot([3*t**2, 2*t, 1, 0], coeff_x) # ([row] * [col]); t = i * 0,1 (choose 0 or 1)
            vy_end = np.dot([3*t**2, 2*t, 1, 0], coeff_y)  # np.dot is done since its a matrix multiplication
            theta = atan2(vy_end, vx_end)    #re-calculate angle theta
            dtheta = theta - self.pose.theta # delta theta = starting theta - current theta 
            #print(theta)

            #here we change theta to be within the range of Gazebo (-pi, pi)
            if (dtheta < -pi): 
                self.vel.angular.z = Kp*(dtheta + 2*pi)
            elif (dtheta > pi):
                self.vel.angular.z = Kp*(dtheta - 2*pi)
                
            else:
                self.vel.angular.z = Kp*dtheta
            
            self.vel.linear.x = sqrt(vx_end**2 + vy_end**2)

            self.vel_pub.publish(self.vel)
            	                            
            self.rate.sleep()
            
        #Update the previous values
        self.previous_waypoint = current_waypoint
        self.previous_velocity = [vx_end, vy_end]
        pass
 
        # implement a PID controller similar to lab 3 
        # 
        #now we go to the next waypoint working with the velocity

    def polynomial_time_scaling_3rd_order(self, p_start, v_start, p_end, v_end, T):
        # input: p,v: position and velocity of start/end point
        # T: the desired time to complete this segment of trajectory (in second)
        # output: the coefficients of this polynomial
        A = np.array([[p_start], [p_end], [v_start], [v_end]])
        B = np.array([[0, 0, 0, 1],[T**3, T**2, T, 1],[0, 0, 1, 0],[3*T**2, 2*T, 1, 0]])
        binv = np.linalg.inv(B)
        return((np.dot(binv, A)))
        
        #print((np.linalg.inv(B)*A))


    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            rospy.loginfo("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))


if __name__ == '__main__':
    whatever = Turtlebot()
