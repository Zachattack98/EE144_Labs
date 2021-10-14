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
	# add your code here to adjust your movement based on 2D pose feedback
	self.vel = Twist()
	cnt = 0	                            
	theta_error = 0	                    #Initialized to set dependent variable "last_theta_error" below.
	while not rospy.is_shutdown():
		desired_theta = cnt*(pi/2)	#Variable for turning at each point
		last_theta_error = theta_error	#Used in derivative calculation.
        	if cnt < 2:                       #Condition check to reduce redundant computations
            		theta_error = (desired_theta - self.pose.theta)
            		theta_derivative = (theta_error - last_theta_error) #Derivative of theta_error
	    		K_p = 3		                 #Values initially randomly selected but then
	    		K_d = 0.4 	                 #assigned based on experimental results.
		if(abs(theta_error) < 0.05):		#Want to ensure we don't move forward before we completed turn.
			self.vel.linear.x = 0.5		    #Conditional check waits until we are in acceptable
		else:					            #range before starting linear motion again.
			self.vel.linear.x = 0
		if (cnt == 0 and abs(self.pose.x - 4) < 0.07):	#Using odom readings we can tell when we reach a vertex of square.
			cnt += 1					                    
			self.vel.linear.x = 0		                #Linear vel needs to be set to 0 so it'll stop before turning.
            		print 'Side 1 Finished!'
		elif (cnt == 1 and abs(self.pose.y - 4) < 0.07):
			cnt += 1
			self.vel.linear.x = 0
            		print 'Side 2 Finished!'
		elif (cnt == 2):		              #When our desired angle is pi we need a special case, since Gazebo has range of (-pi, pi).
			if (self.pose.theta < 0):	                #If angle is CCW from pi it'll be read as a value close to -pi.
				theta_error = (-(pi) - self.pose.theta)	#Desired theta is adjusted accordingly.
			else:
				theta_error = (desired_theta - self.pose.theta)
           		theta_derivative = (theta_error - last_theta_error)
			if (abs(self.pose.x - 0) < 0.07):
				cnt += 1			           #For last length, desired angle is 3pi/2 but range is (-pi,pi)
				self.vel.linear.x = 0	   #below desired_theta is adjusted to handle this, but it would cause a fast CW
				self.vel.angular.z = 5	   #rotation. Angular vel is set positive for 1 cycle to ensure angle is negative.
				self.vel_pub.publish(self.vel)
                		print 'Side 3 Finished!'
				self.rate.sleep()
		elif (cnt == 3):
			desired_theta = -(pi/2)	                    #Adjustment from 3pi/2 to deal with Gazebo's range.
			theta_error = (desired_theta - self.pose.theta)		#Calculations need to be redone after adjustment.
			theta_derivative = (theta_error - last_theta_error)
			if(abs(theta_error) < 0.05):	                    #Error recalculated so check needs to be repreformed.
				self.vel.linear.x = 0.5
			else:
				self.vel.linear.x = 0
			if ((abs(self.pose.y - 0)) < 0.07): 	            #Assuming x pos ~ 0, when y pos gets close to 0 we have reached origin.
				self.vel.linear.x = 0
				self.vel_pub.publish(self.vel)
				self.rate.sleep()
				print 'Side 4 Finished and Origin Reached!'
				cnt += 1				                            #k iterated one more time as a stop flag.
		#print ('Error in theta = ', theta_error)
		self.vel.angular.z = (K_p * theta_error) + (K_d * theta_derivative)
		#print 'Angular Vel = ', self.vel.angular.z
		self.vel_pub.publish(self.vel)
		if (cnt == 4):
			break
		self.rate.sleep()


    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
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
