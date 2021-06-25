#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

import numpy as np
import cv2
import time
import threading as th

from pure_pursuit import *

def show():
	while not rospy.is_shutdown():
		cv2.imshow("d_img", position_img[::-1])
		if cv2.waitKey(10) == ord('q'):
			break
	cv2.destroyWindow("d_img")

class robot:
	def __init__(self, wheel_topic, odom_topic, base_speed=0.5, rate=50):
		self.rate    = rospy.Rate(rate)
		self.wheels  = rospy.Publisher(wheel_topic, Twist, queue_size=10)
		self.encoder = rospy.Subscriber(odom_topic, Odometry, self.clbk_odom)

		self.wheel_msg = Twist()

		self.pos = [0, 0, 0]

	def move(self, x, w):
		self.wheel_msg.linear.x = x
		self.wheel_msg.angular.z = w
		self.wheels.publish(self.wheel_msg)

	def read_odom(self):
		return self.pos

	def clbk_odom(self, msg):
		pose = msg.pose.pose.position
		self.pos[0] = pose.x
		self.pos[1] = pose.y

		ang   = msg.pose.pose.orientation
		quat  = (ang.x, ang.y, ang.z, ang.w)
		euler = transformations.euler_from_quaternion(quat)

		self.pos[2] = euler[2]



if __name__ == '__main__':	
	global position_img
	position_img = np.zeros((100, 100))

	# Thread for showing the position of robot
	thr = th.Thread(target = show, args=())
	thr.start()

	# node initialization and rate
	rospy.init_node('controls', anonymous=True)
	rate = rospy.Rate(50)

	# topics for robot 
	wheel_topics = ('/cmd_vel', "")
	odom_topics  = ('/odom', "")

	# list of robot objects (here only 1 robot)
	bots = [robot(wheel_topics[0], odom_topics[0])] 
	
	# array of points' co ordinates sequentially on path to follow (start point, ......, goal point)
	x = (0, 0.50, 0.68, 1.99, 2.57, 7.17, 8.50)
	y = (0, 0.77, 0.85, 2.55, 3.54, 4.26, 7.27)

	points = np.array((x, y)).T 		# converted to  numpy array

	# Variables for locomotion
	lhd = 0.35 				# look ahead distance
	v = 5					# linear speed
	w = 0					# angular speed
	theta = 0				# angle for curvature to follow
	min_lin_speed = 1		# minimum linear speed
	lim = 60*np.pi/180.0	# limit for theta
	reached = False			# flag for checking whether robot has reached to goal point

	# Initializing the object of pure pursuit controller
	cont = purePursuit(bots[0].pos, points, lhd)

	if not rospy.is_shutdown():
		time.sleep(2)
		# aligning the robot
		theta, r, d_img = cont.process()
		bots[0].move(0, 3*theta)
		time.sleep(abs(4*theta/np.pi))	
		bots[0].move(0, 0)
		time.sleep(1)

	while not rospy.is_shutdown():
		theta, r, position_img = cont.process()		# process function returns the
													# central angle inscribed by arc joining the robot and look ahead point, 
													# radius of the same arc and 
													# an image that shows path, look ahead point and position of robot 
		
		if reached:
			bots[0].move(0, 0)

		else:
			d = (bots[0].pos[0] - points[-1, 0])**2 + (bots[0].pos[1] - points[-1, 1])**2
			d = d**0.5
			reached = d < cont.lhd

			if theta is None:
				cont.lhd += 0.1			# in case robot has gone far from path, look ahead distance is increased to get the look ahead point on path
			else:
				# constraining the theta
				if theta > lim:
					theta = lim
				elif theta < -lim:
					theta = -lim
				
				# calculating linear and angular velocities
				lin_v = max(min_lin_speed, v - 0.5*(theta**2))
				w = (15.0*(theta**2) * (-1)**(theta<0) + (5 + (5/16.0))*theta)
				
				if d < 1.2:			# if robot has reached near goal point
					lin_v = 2		# speed is decreased
					w /= 1.5
				
				# command to robot for locomotion
				bots[0].move(lin_v, w)

		rate.sleep()
	
	bots[0].move(0, 0)
	thr.join()
	cv2.destroyAllWindows()
