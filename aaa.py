import rospy
import random
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

class Follower:

    def __init__(self):

	#r = rospy.rate(10)
	print("MOVE")
	self.move()

    def move(self):

	goal_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)

	goal = PoseStamped()

	goal.header.seq = 1
	goal.header.stamp = rospy.Time.now()
	goal.header.frame_id = "map"

	goal.pose.position.x = -4.5
	goal.pose.position.y = -4
	goal.pose.position.z = 0.0

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
