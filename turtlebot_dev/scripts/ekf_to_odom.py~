#!/usr/bin/env python

import rospy
from std_msgs.msg import Header, String
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
import tf
import numpy
import math

class Converter():
	def __init__(self):
		self.ekfUpdated = False
		self.odomUpdated = False
		self.ekfPose = PoseWithCovarianceStamped()
		self.odomOrig = Odometry()
		
		self.ekfSub = rospy.Subscriber("/robot_pose_ekf/odom", PoseWithCovarianceStamped, self.ekfPoseCallback)
		self.twistSub = rospy.Subscriber("/odom", Odometry, self.odomCallback)
		self.odomPub = rospy.Publisher("/ekf_odom", Odometry)
		
		self.run()
		
	def ekfPoseCallback(self, msg):
		self.ekfPose = msg
		self.ekfUpdated = True
	
	def odomCallback(self, msg):
		self.odomOrig = msg
		self.odomUpdated = True
	
	def convertAndPublish(self):
		odomMessage = Odometry()
		odomMessage.pose = self.ekfPose.pose
		odomMessage.header = self.ekfPose.header
		odomMessage.child_frame_id = self.odomOrig.child_frame_id
		odomMessage.twist = self.odomOrig.twist
		
		self.odomPub.publish(odomMessage)
	
	def run(self):
		while not rospy.is_shutdown():
			if self.ekfUpdated and self.odomUpdated:
				self.convertAndPublish()
				self.ekfUpdated = False
				self.odomUpdated = False

if __name__ == "__main__":
	try:
		rospy.init_node("ekf_to_odom")
		Converter()
		rospy.spin()
	except Exception as e:
		rospy.loginfo("ekf_to_odom failed with message " + str(e))
