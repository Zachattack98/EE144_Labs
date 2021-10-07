#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
from math import pi

class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.run()
	  #print(pi)


    def run(self):
        vel = Twist()
	cnt = 0
	cnt2 = 0
	# start by readjusting the initial position to keep the bot on the x-axis
	vel.linear.x = 0
        vel.angular.z = pi/16
	for i in range(4):
        	self.vel_pub.publish(vel)
        	self.rate.sleep()
        while not rospy.is_shutdown():  # uncomment to use while loop
		if cnt < 4:
			vel.linear.x = 0.5
        		vel.angular.z = 0
        		for i in range(80):
            			self.vel_pub.publish(vel)
            			self.rate.sleep()

			vel.linear.x = 0	#robot remains in stationary position
            		vel.angular.z = pi/4	#rotate counter-clockwise (right-hand rule)
						#decreasing value slows the rotation speed
			while cnt2 < 20:   #start rotating for a set numbers of 0.1 secs	
            			self.vel_pub.publish(vel)
            			self.rate.sleep()
	    			cnt2 += 1

			#another rotation adjustment; still stationary but smaller
        		vel.angular.z = pi/16
			for i in range(7):
        			self.vel_pub.publish(vel)
        			self.rate.sleep()

			cnt += 1
			cnt2 = 0
		else:
			break


if __name__ == '__main__':
    try:
        whatever = Turtlebot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
