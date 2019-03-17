#!/usr/bin/env python

import numpy

import cv2
import cv_bridge
import rospy
import time
import random

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

f = 1
print(f)

class Follower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image,
                                          self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist,
                                           queue_size=1)
        self.twist = Twist()
	self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
#lower_red = numpy.array([0, 50, 50])
#upper_red = numpy.array([1, 200, 200])             #find way to make it see colour vals
#lower_blue = numpy.array([115, 200, 100])
#upper_blue = numpy.array([125, 255, 105])             #find way to make it see colour 
#lower_yellow = numpy.array([25, 200, 100])
#upper_yellow = numpy.array([35, 255, 105])             #find way to make it see colour #lower_green = numpy.array([55, 200, 100])
#upper_green = numpy.array([65, 255, 105])             #find way to make it see colour vals
    
    def currentcolour(self, colnum):
	if colnum == 1:
		x = numpy.array([115, 200, 100])
		y = numpy.array([125, 255, 105])
		return x, y
	if colnum == 2:
		x = numpy.array([0, 50, 50])
		y = numpy.array([1, 200, 200])
		return x, y
	if colnum == 3:
		x = numpy.array([25, 200, 100])
		y = numpy.array([35, 255, 105])
		return x, y
	if colnum == 4:
		x = numpy.array([55, 200, 100])
		y = numpy.array([65, 255, 105])
		return x, y

    def image_callback(self, msg):
	global f
	col = 1
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	colours = self.currentcolour(f)
	#print colours
	lower_blue = colours[0]
	upper_blue = colours[1]
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        h, w, d = image.shape
        search_top = 0
        search_bot = h
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)	
	
	if M['m00'] == 0:
		print(f)
		f = f + 1
		if f == 5:
			f = 1
	#make an array of the 1 vals that loops through and once found gets removed 

        while M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 2, (255, 255, 255), -1)
            err = cx - w/2
            self.twist.linear.x = 1
            self.twist.angular.z = -float(err) / 100
            print("colour sighted")
            self.cmd_vel_pub.publish(self.twist)
	    if M['m00'] > 2000000:
                self.twist.linear.x = 0
                self.cmd_vel_pub.publish(self.twist)
		f = f + 1 
		print('m00')
		print("found a colour")
	    break
	    col = col + 1
		

	#if M['m00'] == 0:
         #   self.twist.linear.x = -0.5
          #  self.twist.angular.z = 0.2
           # self.cmd_vel_pub.publish(self.twist)
        cv2.imshow("window", image)
        cv2.waitKey(3)

    def laser_callback(self, msg):
	scanner = msg
	scanner.angle_min = 70
	scanner.angle_max = 110
	##time.sleep(1)

	q = 0


	for x in range(0,639):
		if scanner.ranges[x] > 0.0001:
			q = q + 1
	
	mindistance = [None] * q
	
	m = 0

	for x in range(0,639):
		if scanner.ranges[x] > 0.0001:
			mindistance[m] = scanner.ranges[x]
			m = m + 1
	dis = min(mindistance)	
	#print(min(mindistance))

	a = scanner.ranges.index(dis)
	#print a

	rand = random.uniform(0.1, 1)

	if dis < 1.2:
		if a < 213:
			self.twist.linear.x = 0
			self.twist.angular.z = 0.8
			self.cmd_vel_pub.publish(self.twist)
		if a > 426:
			self.twist.linear.x = 0
			self.twist.angular.z = -0.8
			self.cmd_vel_pub.publish(self.twist)
		else:
			self.twist.linear.x = 0
			self.twist.angular.z = -0.8
			self.cmd_vel_pub.publish(self.twist)
	else:
		self.twist.linear.x = 1
		self.twist.angular.z = 0
		self.cmd_vel_pub.publish(self.twist)

	

cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()
