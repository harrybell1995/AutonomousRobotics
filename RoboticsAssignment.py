import rospy #ros python
import random #used to chose location of x,y for the robot pose
import time #used for sleeps
import cv2 #image related
import cv_bridge #also image related
import numpy #used for arrays
import sys #used to exit the program after 4 colours are found

from geometry_msgs.msg import Twist #provides kinematics
from geometry_msgs.msg import PoseStamped #this is how the robot finds where to go
from sensor_msgs.msg import Image #provides image data for the vision 

#global values that are used throughout the program
f = 1 #this value is used throughout to define which colour is being looked at
haa = 249 #this is used for a loop that delays how often a new goal is broadcasted

class Follower:

    def __init__(self):

        self.bridge = cv_bridge.CvBridge()
	self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.robot)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.twist = Twist()

	print("SEARCH BEGINS")

    def findred(self): #colour values for the red pole
	x = numpy.array([0, 200, 100]) 
	y = numpy.array([1, 255, 105])
	#print ("looking for red") 
	return x, y

    def findyellow(self): #colour values for the yellow pole
	x = numpy.array([25, 200, 100])
	y = numpy.array([35, 255, 105])
	#print ("looking for yellow") 
	return x, y    

    def findblue(self): #colour values for the blue pole
	x = numpy.array([115, 200, 100])
	y = numpy.array([125, 255, 105])
	#print ("looking for blue") 
	return x, y    

    def findgreen(self): #colour values for the green pole
	x = numpy.array([55, 200, 100])
	y = numpy.array([65, 255, 105])
	#print ("looking for green") 
	return x, y

	#triggered when a colour is found, changes the colour thats been looked for to the next one.
	#this can be looped to constantly be looking for all colours, but i found one at a time worked 
        #better for the search
    def nextcol(self): 
	global f
	print ("found colour ") 
	print(f)
	f = f + 1

	#when all colours found this function triggers, closing the program
    def finished(self):
	print ("all colours found, exiting program, goodbye!")
	sys.exit()

	#main function of the whole program
    def robot(self, msg):
	global f #uses global variable to find current colour to be looking for		
	global haa #used in the loop to delay new locations
	haa = haa + 1 #increase the loop everytime this function is called, which happens when the image is sensed
        #cv2.namedWindow("window", 1) #can uncomment this to see the robots camera, but not needed
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') #translate the image from the sensor into bgr8
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 

	if f == 1: #made these in this way to make it easier to test colours, ie go for red first to see if i had the correct variables
		colours = self.findyellow()	
	if f == 2:
		colours = self.findred()	
	if f == 3:	
		colours = self.findgreen()
        if f == 4:	
		colours = self.findblue()	
	if f == 5:
		self.finished()

	lower_bound = colours[0] #the bottom of the colour ranges from findcolour above
	upper_bound = colours[1] #top of that range
        mask = cv2.inRange(hsv, lower_bound, upper_bound) #make a binary image from pixels inside the ranges specified
        h, w, d = image.shape #height, width and depth of the image that comes from the sensor
        search_top = 0 #pixel to start at. 0 is very top
        search_bot = h #height to end at. h is the exact bottom of the image
        mask[0:search_top, 0:w] = 0 #mask anything in these areas
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask) #m becomes the mask

        #cv2.imshow("window", image) #used for viewing the camera again
        cv2.waitKey(3) #how long each frame from the camera lasts in miliseconds

	#for i in range (1, 640): #this was used to find the colour values of each object
	#	i = hsv[i]
	#	print (i)

	if M['m00'] == 0: #if nothing is detected
		if haa == 250: #if its time to get a new location
			haa = 0 #reset the counter
			goal_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1) #create a publisher

			goal = PoseStamped() #create an object of type posestamped

			goal.header.seq = 1 #used to order goals
			goal.header.stamp = rospy.Time.now() #time stamp of when goal was sent
			goal.header.frame_id = "map" #use global reference frame related to map 

			goal.pose.position.x = round(random.uniform(-4, 4), 2) #pick a random number between -4 and 4
			goal.pose.position.y = round(random.uniform(-4, 4), 2) #to two decimal places for the x, y vals

			print("HEADING TO ", goal.pose.position.y, goal.pose.position.x )#prints coods

			goal.pose.orientation.x = 0.0 # these need to be set to 0
			goal.pose.orientation.y = 0.0 # but because the bot is moving
			goal.pose.orientation.z = 0.0 # in 2d space they dont do anything
			goal.pose.orientation.w = random.uniform(0, 360) #which angle to be facing
			rospy.sleep(3) #wait 3 seconds before continuing, used because sometimes the goal is sent before the subscriber is ready
			goal_publisher.publish(goal)#publish the goal to move_base_simple/goal
	else: #if a colour is detected then
            cx = int(M['m10']/M['m00']) #make a value that says how left it is
            cy = int(M['m01']/M['m00']) #and how right it is
            #cv2.circle(image, (cx, cy), 2, (255, 255, 255), -1) #draw a white circle in the image, only used if an image window is on display
            err = cx - w/2 #error is cx divided by two
            self.twist.linear.x = 0.5 #go forwards at 0.5m/s
            self.twist.angular.z = -float(err) / 100 #rotate at the speed of err divided by 100
	    self.cmd_vel_pub.publish(self.twist) #send the twist to the robots wheels to navigate
	    haa = 0 #reset the value of time before the next loop, if this isnt here the bot will move to a new location rather than the colour pole
            #print(err)
    	    if M['m00'] > 6000000: #if there are 6,000,000 pixels in the middle of the image
		self.nextcol() #select the next colour
	        self.twist.linear.x = 0 #stop moving
	        self.cmd_vel_pub.publish(self.twist) #publish stop moving
		print("COLOUR FOUND") #post youve found the colour
		haa = 249 #next frame a new goal location is selected
		rospy.sleep(2) #sleep for 2 seconds before coninuing

rospy.init_node('follower')
follower = Follower()
rospy.spin()

#rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{ header: {stamp: now, frame_id: "map"}, pose: { position: {x: -4.5, y: 5, z: 0.0}, orientation: {w: 1}}}'
#use this to move the robot around if its taking ages in the viva
