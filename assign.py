#!/usr/bin/env python

import numpy

import cv2
import cv_bridge
import rospy
import time


from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class Follower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image,
                                          self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist,
                                           queue_size=1)
        self.twist = Twist()
	self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)

    def image_callback(self, msg):
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	lower_red = numpy.array([0, 50, 50])
	upper_red = numpy.array([1, 52, 52])             #find way to make it see colour vals
        mask = cv2.inRange(hsv, lower_red, upper_red)
        h, w, d = image.shape
        search_top = 0
        search_bot = h
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)	


        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 2, (255, 255, 255), -1)
            cv2.circle(image, (320, 0), 2, (50, 50, 50), -1)	
            err = cx - w/2
            self.twist.linear.x = 1
            self.twist.angular.z = -float(err) / 100
            #print self.twist.angular.z

            self.cmd_vel_pub.publish(self.twist)

	#if M['m00'] == 0:
         #   self.twist.linear.x = -0.5
          #  self.twist.angular.z = 0.2
           # self.cmd_vel_pub.publish(self.twist)
        cv2.imshow("window", image)
        cv2.waitKey(3)


    def laser_callback(self, msg):
	scanner = msg
	
	scanner.angle_min = 80
	scanner.angle_max = 100
	j = 1.5646858416414

	flag = 1	
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
	print(min(mindistance))

	if dis < 1:
		self.twist.linear.x = 0
		self.twist.angular.z = -2
		self.cmd_vel_pub.publish(self.twist)
		time.sleep(0.01)
	else:
		self.twist.linear.x = 1
		self.twist.angular.z = 0
		self.cmd_vel_pub.publish(self.twist)

	
#lower_red = numpy.array([0, 50, 50])
#upper_red = numpy.array([1, 200, 200])             #find way to make it see colour vals
cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()
