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

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')


    def run(self):
        waypoints = [[0.5, 0], [0.5, -0.5], [1, -0.5], [1, 0], [1, 0.5],\
                      [1.5, 0.5], [1.5, 0], [1.5, -0.5], [1, -0.5], [1, 0],\
                      [1, 0.5], [0.5, 0.5], [0.5, 0], [0, 0], [0, 0]]
        for i in range(len(waypoints)-1):
            self.move_to_point(waypoints[i], waypoints[i+1])

    def move_to_point(self, current_waypoint, next_waypoint):
        self.vel = Twist()
        self.previous_waypoint = [0,0]
        self.previous_velocity = [0,0]
        
        # generate polynomial trajectory and move to current_waypoint      
        # next_waypoint is to help determine the velocity to pass current_waypoint
        #determine boundary conditions for the position for x and y
        px_start = self.previous_waypoint[0]
        px_end = self.current_waypoint[0]

        py_start = self.previous_waypoint[1]
        py_end = self.current_waypoint[1]

        #determine boundary  conditions for velocity
        #need to compute the angle of the robot in order to change the x,y velocities
        vx_start = self.previous_velocity[0]#velocity*cos()
        vy_start = self.previous_velocity[1]#velocity*sin()

        #decompose the velocity
        dx = next_waypoint[0] - current_waypoint[0]
        dy = next_waypoint[1] - current_waypoint[1]
        angle = atan2(dy,dx)

        vx_end = self.vel_ref*cos(angle)
        vy_end = self.vel_ref*sin(angle)

        #solve the polynomial coefficient
        #going to get coefficients for x and coefficients for y
        T = 2

        coeff_x = self.polynomial_time_scaling_3rd_order(px_start, vx_start, px_end, vx_end, T)
        coeff_y = self.polynomial_time_scaling_3rd_order(py_start, vy_start, py_end, vy_end, T)

        #now we want the robot to move, we have the coefficients now which can be used to find the velocity.
        #The velocity can now be sent to twist for a specified range of time values hint: use for loop
        
        for i in range(0, 5*T)
            #send the velocity
            #we can play with t and extend the time to see what happens
            t = i * 1
            #compute Vx_end, Vy_end; get vx vy, then we go from velocity to angle
            Vx_end = [3*t^2, 2*t, 1, 0]*[coeffx]  # ([row] * [col]); t = i * 0,1 (choose 0 or 1)
            Vy_end = [3*t^2, 2*t, 1, 0]*[coeffy]  # same t
            
            #Code from lab 3 closed loop
            cnt = 0	                            
	        theta_error = 0	                    #Variable for e(t); compute theta
	        #while not rospy.is_shutdown():
		    turn_theta = cnt*(pi/2)		#Variable for turning at each point; 2D means we can stick with pi/2 to rotate 90 degrees.
		    delta_theta_error = theta_error	    #Temporary variable to store the lastest theta error; compute delta theta (self.pose.theta)
		    K_p = 3		                
	        K_d = 0.5

            if cnt < 2:                       #Condition check to reduce redundant computations
                theta_error = (turn_theta - self.pose.theta)
                theta_derivative = (theta_error - delta_theta_error) #Derivative of theta_error
		    if(abs(theta_error) < 0.05):	
			    self.vel.linear.x = 0.5		
		    else:					        
			    self.vel.linear.x = 0
		    if (cnt == 0 and abs(self.pose.x - 2) < 0.07):	#Using odom readings we can tell when we reach a corner; side 1
			    cnt += 1					                    
			    self.vel.linear.x = 0		        #Reset linear velocity before turning.
		    elif (cnt == 1 and abs(self.pose.y - 2) < 0.07):    #side 2
			    cnt += 1
			    self.vel.linear.x = 0
		    elif (cnt == 2):  #side 3
			    #Since Gazebo's range is (-pi, pi) we cannot use 3*pi/2, or else the bot spins out of control. Redo calculations     
		        if (self.pose.theta < 0):
				    theta_error = (-(pi) - self.pose.theta)
			    else:
				    theta_error = (turn_theta - self.pose.theta)
           		    theta_derivative = (theta_error - delta_theta_error)
			    if (abs(self.pose.x - 0) < 0.07):
				    cnt += 1		
				    self.vel.linear.x = 0
		    elif (cnt == 3):    #side 4
			    if (self.pose.theta < 0):
				    theta_error = (-(pi/2) - self.pose.theta)
			    else:
				    theta_error = (turn_theta - self.pose.theta)
           		    theta_derivative = (theta_error - delta_theta_error)
			    if ((abs(self.pose.y - 0)) < 0.07):
				    cnt += 1
				    self.vel.linear.x = 0			  
		
		    self.vel.angular.z = (K_p * theta_error) + (K_d * theta_derivative)	#Angular velocity using given u(t) equation; 
											                                    #k_p*e(t) + k_d*(e(t) - e(t - delta(t)))
                                                                                #update velocity = Kp * delta theta
		
		    self.vel_pub.publish(self.vel)  #Publish velocities
		    if (cnt == 4):
			    self.vel.angular.z = pi/2 + pi/4 + pi/8		#Reset to initial direction; pi was too much
			    self.vel_pub.publish(self.vel)
			    break
		    self.rate.sleep()
            
            #Update the previous values
            px_start = self.current_waypoint[i+1]
            px_end = self.next_waypoint[i+2]

            py_start = self.current_waypoint[i+2]
            py_end = self.next_waypoint[i+3]

            vx_start = self.current_velocity[i+1]  #velocity*cos()
            vy_start = self.current_velocity[i+2]  #velocity*sin()
            
            dx = next_waypoint[0] - current_waypoint[0]
            dy = next_waypoint[1] - current_waypoint[1]
            angle = atan2(dy,dx)

            vx_end = self.vel_ref*cos(angle)
            vy_end = self.vel_ref*sin(angle)

            coeff_x = self.polynomial_time_scaling_3rd_order(px_start, vx_start, px_end, vx_end, T)
            coeff_y = self.polynomial_time_scaling_3rd_order(py_start, vy_start, py_end, vy_end, T)
 
        # implement a PID controller similar to lab 3 
        # 
        #now we go to the next waypoint working with the velocity

    def polynomial_time_scaling_3rd_order(self, p_start, v_start, p_end, v_end, T):
        # input: p,v: position and velocity of start/end point
        # T: the desired time to complete this segment of trajectory (in second)
        # output: the coefficients of this polynomial
        A = np.array([[p_start], [p_end], [v_start], [v_end]])
        B = np.array([[0, 0, 0, 1],[T*T*T, T*T, T, 1],[0, 0, 1, 0],[3*T*T, 2*T, 1, 0]])
        return (np.linalg.inv(A))*B


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
