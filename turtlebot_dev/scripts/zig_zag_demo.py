#!/usr/bin/env python

#This node is used to store, retrieve and remove navigation locations
#It subscribes to the robot's pose and receives IDs to save with
#the poses through the service WaypointServerService
import rospy
from std_msgs.msg import Header, String
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import OccupancyGrid, MapMetaData
import tf

from util.srv import PoseTransformSrv


class ZigZag:

	#Creates the service, begins listening to the robot pose, and then spins
	def __init__(self):
		#Globals
		self.waypoints = []
		self.goal = PoseStamped()

		#Subscribers
		self.listener = tf.TransformListener()

		#Publishers
		self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, latch=True)
		self.pose_array_pub = rospy.Publisher('/zig_zag_demo/pose_array', PoseArray, latch=True)

		self.pose_transformer = rospy.ServiceProxy('pose_transform', PoseTransformSrv)

		#Init Functions
		self.init_waypoints()

		self.run()

	def getTransform(self):
		try:
			(trans, rot) = self.listener.lookupTransform('ar_marker_0', 'base_link', rospy.Time(0))
			return (trans[0], trans[1])
		except Exception as e:
			#rospy.loginfo("Ouch! " + str(e))
			return

	def init_waypoints(self):
		self.waypoints = []
		self.pose_array = PoseArray()
		pose_array_list = []

		rospy.wait_for_service('pose_transform')

		x0 = 0.8
		y0 = 1.5
		dx = 0.2
		dy = 1

		for i in range(16):
			x = 0
			y = 0

			counter = i % 4

			if counter == 0:
				x = x0
				y = y0
			elif counter == 1:
				x = x0
				y = y0 - dy
			elif counter == 2:
				x0 -= dx
				x = x0
				y = y0 - dy
			elif counter == 3:
				x = x0
				y = y0
				x0 -= dx

			pose_msg = Pose()
			pose_stamp_msg = PoseStamped()
			header = Header()
			header.frame_id = "ar_marker_0"

			pose_stamp_msg.header = header
			pose_msg.position.x = -x
			pose_msg.position.z = y
			pose_msg.orientation.w = 1;

			pose_stamp_msg.pose = pose_msg

			transformed_pose_stamp = (self.pose_transformer(String("map"), pose_stamp_msg)).transformed_pose
			transformed_pose_stamp.pose.orientation.x = 0
			transformed_pose_stamp.pose.orientation.y = 0
			transformed_pose_stamp.pose.orientation.z = 0
			transformed_pose_stamp.pose.orientation.w = 1


			pose_array_list.append(pose_msg)

			self.waypoints.append(transformed_pose_stamp)

		self.pose_array.header.frame_id = "ar_marker_0"
		self.pose_array.poses = pose_array_list

		self.pose_array_pub.publish(self.pose_array)

		rospy.loginfo("Waypoints initialized and published!")

	def areWeThereYet(self):
		tol = 0.05

		curXY = self.getTransform()
		goalXY = (self.goal.pose.position.x, self.goal.pose.position.z)
		
		if curXY == None or goalXY == None:
			return False

		#rospy.loginfo("areWeThereYet(): curr = " + str(curXY[0]) + "," + str(curXY[0]) + " goal = " + str(goalXY[0]) + "," + str(goalXY[1]))
		if (curXY[0] - goalXY[0]) < tol and (curXY[1] - goalXY[1]) < tol:
			rospy.loginfo("Goal reached!")
			return True
		return False

	def nextGoal(self):
		if not self.waypoints == []:
			self.goal = self.waypoints.pop(0)
			self.goal_pub.publish(self.goal)
			rospy.loginfo("Published new goal: " + str(self.goal.pose.position.x) + "," + str(self.goal.pose.position.z))

	def run(self):
		self.nextGoal()
		while not rospy.is_shutdown():
			if self.areWeThereYet():
				self.nextGoal()

if __name__=="__main__":
	try:
		rospy.init_node('zig_zag_demo')
		ZigZag()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Zig Zag Demo terminated.")

