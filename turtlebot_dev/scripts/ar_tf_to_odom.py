#!/usr/bin/env python

#This node is used to store, retrieve and remove navigation locations
#It subscribes to the robot's pose and receives IDs to save with
#the poses through the service WaypointServerService
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf
from tf.transformations import quaternion_from_euler


class ArTfToOdom:

	def __init__(self):
		#Globals
		self.transform = None

		#Subscribers
		self.listener = tf.TransformListener()

		#Publishers
		self.vo_publisher = rospy.Publisher("ar/vo", Odometry)

		#Init Functions

		self.run()

	def getTransform(self, frame):
		try:
			rospy.loginfo("Getting transform")
			(trans, rot) = self.listener.lookupTransform('ar_marker_0', frame, rospy.Time(0))
			return (trans, rot)
		except:
			rospy.loginfo("Error getting transform")
			return None

	def sendVisualOdom(self, frame):
		try:
			x = self.transform[0][0]
			y = self.transform[0][1]
			z = self.transform[0][2]
			roll = self.transform[1][0]
			pitch = self.transform[1][1]
			yaw = self.transform[1][2]

			odom_msg = Odometry()
			header_msg = Header()
			pose_msg = PoseWithCovarianceStamped()
			twist_msg = TwistWithCovarianceStamped()

			header.frame_id = "ar_marker_0"
			odom_msg.child_frame_id = "ar_marker_0"
			pose_msg.pose.pose.position.x = x
			pose_msg.pose.pose.position.x = y
			pose_msg.pose.pose.position.x = z

			quat = quaternion_from_euler(roll, pitch, yaw)

			pose_msg.pose.pose.orientation.x = quat[0]
			pose_msg.pose.pose.orientation.y = quat[1]
			pose_msg.pose.pose.orientation.z = quat[2]
			pose_msg.pose.pose.orientation.w = quat[3]

			twist_msg.twist.covariance = [999 for i in range(36)]

			odom_msg.pose = pose_msg
			odom_msg.twist = twist_msg

			rospy.loginfo("Publishing Visual Odometry!")
			self.vo_pub.publish(odom_msg)
		except:
			rospy.loginfo("Error Publishing Visual Odometry")

	def run(self):
		while not rospy.is_shutdown():
			transform = self.getTransform("base_link")
			if transform == None:
				rospy.loginfo("No transform given, cannot publish visual odometry")
				continue
			else:
				self.transform = transform
				sendVisualOdom("base_link")
			rospy.sleep(1)

if __name__=="__main__":
	try:
		rospy.init_node('ar_tf_to_odom')
		ArTfToOdom()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("AR tf to odom terminated.")

