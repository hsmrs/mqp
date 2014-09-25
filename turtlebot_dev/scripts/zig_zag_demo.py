#!/usr/bin/env python

#This node is used to store, retrieve and remove navigation locations
#It subscribes to the robot's pose and receives IDs to save with
#the poses through the service WaypointServerService
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import OccupancyGrid, MapMetaData
import tf


class ZigZag:

	#Creates the service, begins listening to the robot pose, and then spins
	def __init__(self):
		#Globals
		self.waypoints = []
		self.goal = PoseStamped()

		#Subscribers
		self.listener = tf.TransformListener()

		#Publishers
		self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped)
		self.pose_array_pub = rospy.Publisher('/zig_zag_demo/pose_array', PoseArray)

		#Init Functions
		self.init_waypoints()

		self.run()

	def getTransform(self):
		try:
			(trans, rot) = self.listener.lookupTransform('ar_marker_0', 'base_link', rospy.Time(0))
			return (trans.x, trans.y)
		except Exception as e:
			rospy.loginfo("Ouch! " + str(e))

	def init_waypoints(self):
		self.waypoints = []
		self.pose_array = PoseArray()
		pose_array_list = []

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

			pose_stamp_msg.pose = pose_msge
			pose_array_list.append(pose_msg)

			self.waypoints.append(pose)

		self.pose_array.poses = pose_array_list

		self.pose_array_pub.publish(self.pose_array)

		rospy.loginfo("Waypoints initialized and published!")

	def areWeThereYet(self):
		tol = 0.05

		curXY = self.getTransform()
		goalXY = (self.goal.pose.position.x, self.goal.pose.position.y)

		if (curXY[0] - goalXY[0]) < tol and (curXY[1] - goalXY[1]) < tol:
			rospy.loginfo("Goal reached!")
			return True
		return False

	def nextGoal(self):
		if not self.waypoints == []:
			self.goal = self.waypoints.pop(0)
			self.goal_pub.publish(self.goal)
			rospy.loginfo("Published new goal!")

	def run(self):
		self.nextGoal()
		while not rospy.is_shutdown():
			if self.areWeThereYet:
				self.nextGoal()

if __name__=="__main__":
	try:
		rospy.init_node('zig_zag_demo')
		ZigZag()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Zig Zag Demo terminated.")

