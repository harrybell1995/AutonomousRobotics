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
dist = 5
print(f)
#print(dist)

class Follower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image,
                                          self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist,
                                           queue_size=1)
        self.twist = Twist()
	self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)

    def findred(self):
	x = numpy.array([0, 200, 100])
	y = numpy.array([1, 255, 105])
	print ("looking for red") 
	return x, y

    def findyellow(self):
	x = numpy.array([25, 200, 100])
	y = numpy.array([35, 255, 105])
	print ("looking for yellow") 
	return x, y    

    def findblue(self):
	x = numpy.array([115, 200, 100])
	y = numpy.array([125, 255, 105])
	print ("looking for blue") 
	return x, y    

    def findgreen(self):
	x = numpy.array([55, 200, 100])
	y = numpy.array([65, 255, 105])
	print ("looking for green") 
	return x, y

    def nextcol(self):
	global f
	print ("found colour ") 
	print(f)
	f = f + 1


    def image_callback(self, msg):
	global f
	global dist
	global mindist
	col = 1
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

	if f == 3:	
		colours = self.findyellow()	
	if f == 2:	
		colours = self.findred()	
        if f == 1:	
		colours = self.findblue()	
	if f == 4:	
		colours = self.findgreen()
	if f == 5:
	        self.twist.linear.x = 0
            	self.twist.angular.z = 99
                self.cmd_vel_pub.publish(self.twist)
		

	#print colours
	lower_bound = colours[0]
	upper_bound = colours[1]
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        h, w, d = image.shape
        search_top = 0
        search_bot = h
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)	
	#for i in range (1, 640):
	#	i = hsv[i]
	#	print i
	
	#make an array of the 1 vals that loops through and once found gets removed 
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 2, (255, 255, 255), -1)
            err = cx - w/2
            self.twist.linear.x = 0.5
            self.twist.angular.z = -float(err) / 100
            print("colour sighted")
            self.cmd_vel_pub.publish(self.twist)
	    print (dist)
    	    if M['m00'] > 2000000:
	  	    if dist < 1:
			self.nextcol()
		        self.twist.linear.x = 0
		        self.cmd_vel_pub.publish(self.twist)
			print("help")

        cv2.imshow("window", image)
        cv2.waitKey(3)

    def objleft(self):
	self.twist.linear.x = 0
	self.twist.angular.z = 2
	self.cmd_vel_pub.publish(self.twist)

    def objcent(self, rand):
	self.twist.linear.x = 0
	self.twist.angular.z = 2
	self.cmd_vel_pub.publish(self.twist)
	#time.sleep(0.5)

    def objright(self):
	self.twist.linear.x = 0
	self.twist.angular.z = 2
	self.cmd_vel_pub.publish(self.twist)

    def laser_callback(self, msg):
	global dist
	scanner = msg
	scanner.angle_min = 70
	scanner.angle_max = 110
	time.sleep(0.05)

	q = 0


	for x in range(0,639):
		if scanner.ranges[x] > 0.0001:
			q = q + 1
	
	mindistance = [None] * q	
	objpos = [None] * q
	
	m = 0

	for x in range(0,639):
		if scanner.ranges[x] > 0.0001:
			mindistance[m] = scanner.ranges[x]
			objpos[m] = x
			m = m + 1
			
	avg = sum(objpos) / len(objpos)
	dis = min(mindistance)	
	#print(min(mindistance))
	dist = dis
	a = scanner.ranges.index(dis)
	print avg

	rand = random.uniform(0.1, 1)

	if dis < 1.5:
		if a < 319:
			self.objleft()
			#time.sleep(1)
		if a > 320:
			self.objright()
			#time.sleep(1)
		#else:
		#	self.objcent(rand)
	else:
		self.twist.linear.x = 1
		self.twist.angular.z = 0
		self.cmd_vel_pub.publish(self.twist)

	

cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()



    
cv2.destroyAllWindows()

