import rospy
import random
import time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image



class Follower:

    def __init__(self):

	#r = rospy.rate(10)
	print("MOVE")
	x = 1
	while x < 2:
		self.move()
		time.sleep(10)

    def move(self):

	goal_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)

	goal = PoseStamped()

	goal.header.seq = 1
	goal.header.stamp = rospy.Time.now()
	goal.header.frame_id = "map"

	goal.pose.position.x = random.uniform(-4, 4)
	goal.pose.position.y = random.uniform(-4, 4)
	goal.pose.position.z = 0.0

	print("moving to ", goal.pose.position.y, goal.pose.position.x )

	goal.pose.orientation.x = 0.0
	goal.pose.orientation.y = 0.0
	goal.pose.orientation.z = 0.0
	goal.pose.orientation.w = random.uniform(0, 1)
	rospy.sleep(1)
	goal_publisher.publish(goal)
	#rospy.sleep(15)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
