#!/usr/bin/env python

#This node is used to store, retrieve and remove navigation locations
#It subscribes to the robot's pose and receives IDs to save with
#the poses through the service WaypointServerService
import rospy
from std_msgs.msg import Header, String
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, PoseArray, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData
import tf
import math

from util.srv import PoseTransformSrv


class ZigZag:

	#Creates the service, begins listening to the robot pose, and then spins
	def __init__(self):
		#Globals
		self.waypoints = []
		self.goal = PoseStamped()
		self.pose = Pose()
		self.waypointsDone = 0

		self.TOTAL_WAYPOINTS = 18

		#Subscribers
		self.listener = tf.TransformListener()
		self.pose_sub = rospy.Subscriber("/robot_pose_ekf/odom", PoseWithCovarianceStamped, self.pose_callback)

		#Publishers
		self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, latch=True)
		self.pose_array_pub = rospy.Publisher('/zig_zag_demo/pose_array', PoseArray, latch=True)

		self.pose_transformer = rospy.ServiceProxy('pose_transform', PoseTransformSrv)

		#Init Functions
		self.init_waypoints()

		self.run()

	def pose_callback(self, pose_msg):
		self.pose = pose_msg.pose.pose

	def getTransform(self):
		try:
			(trans, rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
			return (trans[0], trans[1])
		except Exception as e:
			#rospy.loginfo("Ouch! " + str(e))
			return

	def init_waypoints(self):
		self.waypoints = []
		self.easyWaypoints = []
		self.possibleWaypoints = []
		self.easyPossibleWaypoints = []
		self.pose_array = PoseArray()
		pose_array_list = []

		rospy.wait_for_service('pose_transform')

		x0 = 0.8
		y0 = 1.5
		dx = 0.2
		dy = 1

		for i in range(self.TOTAL_WAYPOINTS):
			x = 0
			y = 0

			counter = i % 4

			if counter == 0:
				x = x0
				y = y0
				angle = math.radians(270)
			elif counter == 1:
				x = x0
				y = y0 - dy
				angle = math.radians(180)
			elif counter == 2:
				x0 -= dx
				x = x0
				y = y0 - dy
				angle = math.radians(90)
			elif counter == 3:
				x = x0
				y = y0
				x0 -= dx
				angle = math.radians(180)

			pose_msg = Pose()
			pose_stamp_msg = PoseStamped()
			header = Header()
			header.frame_id = 'ar_marker_0'

			pose_stamp_msg.header = header
			pose_msg.position.x = -x
			pose_msg.position.z = y
			#pose_msg.orientation.w = 1
			#angle = math.radians(90) # angles are expressed in radians
			quat = tf.transformations.quaternion_from_euler(0.0, angle, 0.0)
			pose_msg.orientation = Quaternion(*quat.tolist())

			pose_stamp_msg.pose = pose_msg
			self.easyPossibleWaypoints.append(pose_msg)

			transformed_pose_stamp = (self.pose_transformer(String("map"), pose_stamp_msg)).transformed_pose
			"""
			transformed_pose_stamp.pose.orientation.x = 0
			transformed_pose_stamp.pose.orientation.y = 0
			transformed_pose_stamp.pose.orientation.z = 0
			transformed_pose_stamp.pose.orientation.w = 1
			"""
			quat = tf.transformations.quaternion_from_euler(0.0, 0.0, angle)
			transformed_pose_stamp.pose.orientation = Quaternion(*quat.tolist())
			pose_array_list.append(pose_msg)

			self.possibleWaypoints.append(transformed_pose_stamp)

		self.pose_array.header.frame_id = "ar_marker_0"
		self.pose_array.poses = pose_array_list

		self.pose_array_pub.publish(self.pose_array)
		self.waypointsDone = 0
		
		self.waypoints.append(self.possibleWaypoints[0])
		self.waypoints.append(self.possibleWaypoints[1])
		self.waypoints.append(self.possibleWaypoints[5])
		self.waypoints.append(self.possibleWaypoints[4])
		self.waypoints.append(self.possibleWaypoints[8])
		self.waypoints.append(self.possibleWaypoints[9])
		self.waypoints.append(self.possibleWaypoints[13])
		self.waypoints.append(self.possibleWaypoints[12])
		self.waypoints.append(self.possibleWaypoints[16])
		self.waypoints.append(self.possibleWaypoints[17])
		
		self.easyWaypoints.append(self.easyPossibleWaypoints[0])
		self.easyWaypoints.append(self.easyPossibleWaypoints[1])
		self.easyWaypoints.append(self.easyPossibleWaypoints[5])
		self.easyWaypoints.append(self.easyPossibleWaypoints[4])
		self.easyWaypoints.append(self.easyPossibleWaypoints[8])
		self.easyWaypoints.append(self.easyPossibleWaypoints[9])
		self.easyWaypoints.append(self.easyPossibleWaypoints[13])
		self.easyWaypoints.append(self.easyPossibleWaypoints[12])
		self.easyWaypoints.append(self.easyPossibleWaypoints[16])
		self.easyWaypoints.append(self.easyPossibleWaypoints[17])

		rospy.loginfo("Waypoints initialized and published!")

		#test_pose = PoseStamped()
		#test_pose.header.frame_id = 'map'
		#test_pose.pose.position.x = 1
		#test_pose.pose.position.y = -.63
		#test_pose.pose.orientation.w = 1
		#self.waypoints[0] = test_pose

	def areWeThereYet(self):
		tol = 0.12

		curXY = (self.pose.position.x, self.pose.position.y)
		goalXY = (self.goal.pose.position.x, self.goal.pose.position.y)
		
		if curXY == None or goalXY == None:
			return False

		#rospy.loginfo("areWeThereYet(): curr = " + str(curXY[0]) + "," + str(curXY[0]) + " goal = " + str(goalXY[0]) + "," + str(goalXY[1]))
		if abs(curXY[0] - goalXY[0]) < tol and abs(curXY[1] - goalXY[1]) < tol:
			self.waypointsDone = self.waypointsDone + 1
			rospy.loginfo("Goal " + str(self.waypointsDone) + " reached!\nCurrent pos:" + str(self.pose) + "\nGoal pos:" + str(self.goal.pose) + "\nEasy goal pos:" + str(self.easyWaypoints[self.waypointsDone - 1]))
			return True
		#else:
		#	rospy.loginfo(math.sqrt((curXY[0] - goalXY[0])**2 +(curXY[1] - goalXY[1])**2))
		return False

	def nextGoal(self):
		rospy.loginfo("waypoints size: " + str(len(self.waypoints)))
		if not self.waypoints == []:
			self.goal = self.waypoints.pop(0)
			rospy.sleep(10)
			self.goal_pub.publish(self.goal)
			rospy.loginfo("Published new goal: " + str(self.goal.pose.position.x) + "," + str(self.goal.pose.position.y))
		else:
			rospy.spin()

	def run(self):
		self.nextGoal()
		while not rospy.is_shutdown():
			if self.areWeThereYet():
				self.nextGoal()
			rospy.sleep(0.2)

if __name__=="__main__":
	try:
		rospy.init_node('zig_zag_demo')
		ZigZag()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Zig Zag Demo terminated.")

