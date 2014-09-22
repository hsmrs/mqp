#!/usr/bin/env python

#This node is used to store, retrieve and remove navigation locations
#It subscribes to the robot's pose and receives IDs to save with
#the poses through the service WaypointServerService
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
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

		#Init Functions
		self.init_waypoints()

		rospy.on_shutdown(self.shutdown)
		self.run()

	def getTransform(self):
		try:
			(trans, rot) = self.listener.lookupTransform('ar_marker_0', 'base_link', rospy.Time(0))
			return (trans.x, trans.y)
		except:
			rospy.loginfo("Ouch!")

	def init_waypoints(self):
		self.waypoints = []

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

			pose = PoseStamped()
			header = Header()
			header.frame_id = "ar_marker_0"

			pose.header = header
			pose.pose.position.x = -x
			pose.pose.position.z = y

			self.waypoints.append(pose)

	def areWeThereYet(self):
		tol = 0.05

		curXY = self.getTransform()
		goalXY = (self.goal.pose.position.x, self.goal.pose.position.y)

		if (curXY[0] - goalXY[0]) < tol and (curXY[1] - goalXY[1]) < tol:
			return True
		return False

	def nextGoal(self):
		if not self.waypoints == []:
			goal = self.waypoints.pop(0)
			self.goal_pub.publish(goal)
			rospy.loginfo("Published new goal!")

	def run(self):
		self.nextGoal()
		while not rospy.is_shutdown():
			if self.areWeThereYet:
				self.nextGoal()


	def shutdown(self):
		pass

if __name__=="__main__":
	try:
		rospy.init_node('zig_zag_demo')
		ZigZag()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Zig Zag Demo terminated.")

