#!/usr/bin/env python

#This node is used to store, retrieve and remove navigation locations
#It subscribes to the robot's pose and receives IDs to save with
#the poses through the service WaypointServerService
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import OccupancyGrid, MapMetaData
import tf


class Sanity:

	def __init__(self):
		#Globals
		self.transform = None

		#Subscribers
		self.listener = tf.TransformListener()

		#Init Functions

		self.run()

	def getTransform(self):
		try:
			transform = self.listener.lookupTransform('ar_marker_0', 'map', rospy.Time(0))
			return transform
		except Exception as e:
			rospy.loginfo("Transform not found!")
			return None

	def getHistoricTransform(self):
		try:
			time = self.listener.getLatestCommonTime('ar_marker_0', 'map')
			transform = self.listener.lookupTransform('ar_marker_0', 'map', time)
			rospy.loginfo("Historic transform found!")
			return transform
		except Exception as e:
			rospy.loginfo("Historic transform not found! ROS is broken!")
			return None

	def run(self):
		while not rospy.is_shutdown():
			self.transform = self.getTransform()
			if self.transform == None:
				self.getHistoricTransform()
			rospy.sleep(1.0)

if __name__=="__main__":
	try:
		rospy.init_node('sanity check')
		Sanity()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("sanity check terminated.")

