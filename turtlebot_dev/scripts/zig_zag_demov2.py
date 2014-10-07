#!/usr/bin/env python

#This node is used to store, retrieve and remove navigation locations
#It subscribes to the robot's pose and receives IDs to save with
#the poses through the service WaypointServerService
import roslib; roslib.load_manifest('move_base');
import rospy
from std_msgs.msg import Header, String
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, PoseArray, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData
import tf
from tf.transformations import euler_from_quaternion
import math
from ar_track_alvar.msg import AlvarMarkers

from util.srv import PoseTransformSrv
import actionlib
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal


class ZigZag:

	#Creates the service, begins listening to the robot pose, and then spins
	def __init__(self):
		#Globals
		self.TOTAL_WAYPOINTS = 18
		self.LEFT = -1
		self.RIGHT = 1
		
		self.waypoints = []
		self.goal = PoseStamped()
		self.pose = Pose()
		self.marker = None
		self.waypointsDone = 0
		
		self.lastDirection = self.LEFT
		self.findingTag = False

		#Subscribers
		self.listener = tf.TransformListener()
		self.pose_sub = rospy.Subscriber("/robot_pose_ekf/odom", PoseWithCovarianceStamped, self.pose_callback)
		self.marker_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_callback)

		#Publishers
		self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, latch=True)
		self.pose_array_pub = rospy.Publisher('/zig_zag_demo/pose_array', PoseArray, latch=True)

		self.pose_transformer = rospy.ServiceProxy('pose_transform', PoseTransformSrv)
		
		#Action client
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.client.wait_for_server()
		
		#Init Functions
		self.init_waypoints()

		self.run()

	def pose_callback(self, pose_msg):
		self.pose = pose_msg.pose.pose
		
	def marker_callback(self, marker_msg):
		if len(marker_msg.markers) == 0:
			self.marker = None
			return
		if self.marker == None:
			rospy.loginfo("found a tag!")
			
		if self.findingTag:
			self.client.cancel_goal()
			
		self.marker = marker_msg.markers[0].pose.pose
		if self.marker.position.y < 0:
			self.lastDirection = self.LEFT
		else:
			self.lastDirection = self.RIGHT
	
	def findMarker(self):
		pose_msg = Pose()
		pose_stamp_msg = PoseStamped()
		header = Header()
		
		header.frame_id = 'base_link'

		pose_stamp_msg.header = header
		pose_msg.position.x = 0
		pose_msg.position.y = 0
		pose_msg.position.z = 0
		
		action = MoveBaseGoal()
			
		while self.marker == None:
			rospy.loginfo("no tag yet...")
			self.findingTag = True
			quat = tf.transformations.quaternion_from_euler(0.0, 0.0, math.pi/4 * self.lastDirection)
			pose_msg.orientation = Quaternion(*quat.tolist())

			pose_stamp_msg.pose = pose_msg
		
			action.target_pose = pose_stamp_msg
			self.client.send_goal(action)
			self.client.wait_for_result()
		
		rospy.loginfo("centering tag\nangle: " + str(math.degrees(math.atan(self.marker.position.y/self.marker.position.x))))
		self.findingTag = False
		angle = math.atan(self.marker.position.y/self.marker.position.x)
		quat = tf.transformations.quaternion_from_euler(0.0, 0.0, angle)
		pose_msg.orientation = Quaternion(*quat.tolist())
		
		pose_stamp_msg.pose = pose_msg
	
		action.target_pose = pose_stamp_msg
		self.client.send_goal(action)
		self.client.wait_for_result()
		if self.marker == None:
			return self.findMarker()
		rospy.loginfo("final angle: " + str(math.degrees(math.atan(self.marker.position.y/self.marker.position.x))))
		return

	def init_waypoints(self):
	
		self.waypoints = []
		self.easyWaypoints = []
		self.possibleWaypoints = []
		self.easyPossibleWaypoints = []
		self.pose_array = PoseArray()
		pose_array_list = []

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
				angle = math.radians(0)
			elif counter == 1:
				x = x0
				y = y0 - dy
				angle = math.radians(90)
			elif counter == 2:
				x0 -= dx
				x = x0
				y = y0 - dy
				angle = math.radians(180)
			elif counter == 3:
				x = x0
				y = y0
				x0 -= dx
				angle = math.radians(90)

			pose_msg = Pose()
			pose_stamp_msg = PoseStamped()
			header = Header()
			header.frame_id = 'ar_marker_0'

			pose_stamp_msg.header = header
			pose_msg.position.x = x
			pose_msg.position.y = y
			quat = tf.transformations.quaternion_from_euler(0.0, 0.0, angle)
			pose_msg.orientation = Quaternion(*quat.tolist())

			pose_stamp_msg.pose = pose_msg

			self.possibleWaypoints.append(pose_stamp_msg)
			

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
		
		"""
		#TODO remember to remove this test
		pose_msg = Pose()
		pose_stamp_msg = PoseStamped()
		header = Header()
		header.frame_id = 'odom'

		pose_stamp_msg.header = header
		pose_msg.position.x = 1
		pose_msg.position.y = 0
		pose_msg.position.z = 0
		#pose_msg.orientation.w = 1
		#angle = math.radians(90) # angles are expressed in radians
		quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
		pose_msg.orientation = Quaternion(*quat.tolist())

		pose_stamp_msg.pose = pose_msg
		
		self.waypoints = [pose_stamp_msg]
		"""
		rospy.loginfo("Waypoints initialized and published!")

	def nextGoal(self):
		rospy.loginfo("waypoints size: " + str(len(self.waypoints)))
		self.findMarker()
		if not self.waypoints == []:
			self.goal = self.waypoints.pop(0)
			goalX = self.goal.pose.position.x
			goalY = self.goal.pose.position.y
			
			markerX = self.marker.position.x
			markerY = self.marker.position.y
			angleFromMarker = euler_from_quaternion([self.marker.orientation.x, self.marker.orientation.y, self.marker.orientation.z, self.marker.orientation.w])[2]
			distanceFromMarker = math.sqrt(markerX * markerX + markerY * markerY)
			currX = distanceFromMarker * math.cos(angleFromMarker)
			currY = distanceFromMarker * math.sin(angleFromMarker)
			rospy.loginfo("current estimated pos\nx: " + str(currX) + "\ny: " + str(currY))
			
			dx = goalX - currX
			dy = goalY - currY
			rospy.loginfo("movement required\ndx: " + str(dx) + "\ndy: " + str(dy))
			
			distanceToWaypoint = math.sqrt(dx * dx + dy * dy)
			rawAngleToWaypoint = math.atan(dy/dx)
			if dx < 0:
				rawAngleToWaypoint = -self.getComplimentaryAngle(rawAngleToWaypoint)
			angleToWaypoint = self.getOptimalAngle(-(self.getComplimentaryAngle(angleFromMarker) + rawAngleToWaypoint))
			rospy.loginfo("angle components:\n marker:" + str(math.degrees(self.getComplimentaryAngle(angleFromMarker))) + "\nwaypoint: " + str(math.degrees(rawAngleToWaypoint)))
			
			pose_msg = Pose()
			pose_stamp_msg = PoseStamped()
			header = Header()
			action = MoveBaseGoal()
			
			header.frame_id = 'base_link'

			pose_stamp_msg.header = header
			pose_msg.position.x = 0
			pose_msg.position.y = 0
			pose_msg.position.z = 0
			
			quat = tf.transformations.quaternion_from_euler(0.0, 0.0, angleToWaypoint)
			pose_msg.orientation = Quaternion(*quat.tolist())

			pose_stamp_msg.pose = pose_msg
		
			action.target_pose = pose_stamp_msg
			
			rospy.loginfo("goal:\nrotation: " + str(math.degrees(angleToWaypoint)) + "\ndistance: " + str(distanceToWaypoint))
			#raw_input("continue?")
			
			self.client.send_goal(action)
			self.client.wait_for_result()
			
			pose_msg.position.x = distanceToWaypoint
			quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
			pose_msg.orientation = Quaternion(*quat.tolist())

			pose_stamp_msg.pose = pose_msg
		
			action.target_pose = pose_stamp_msg
			
			self.client.send_goal(action)
			self.client.wait_for_result()
			
			
		elif self.waypointsDone == 0:
			rospy.spin()
		else:
			rospy.signal_shutdown("done")

	def getComplimentaryAngle(self, angle):
		angle = math.degrees(angle)
		if (angle > 0):
			angle = angle % 180
			return math.radians(180 - angle)
		else:
			angle = -(abs(angle) % 180)
			return math.radians(-180 - angle)
	
	def getOptimalAngle(self, angle):
		angle = math.degrees(angle)
		if (angle > 0):
			if (angle < 180):
				return math.radians(angle)
			else:
				return math.radians(-(180 - (angle % 180)))
		else:
			if (angle > -180):
				return math.radians(angle)
			else:
				return math.radians(180 - (abs(angle) % 180))

	def run(self):
		"""
		self.nextGoal()
		while not rospy.is_shutdown():
			if self.client.wait_for_result(rospy.Duration.from_sec(2.0)):
				rospy.loginfo("got result: " + self.printStatus())
				self.nextGoal()
			else:
				rospy.loginfo("Haven't reach goal yet, sleeping...")
			rospy.sleep(0.2)
		"""
		while not rospy.is_shutdown():
			self.nextGoal()
	
	def printStatus(self):
		code = self.client.get_state()
		if code == 0:
			return "PENDING"
		elif code == 1:
			return "ACTIVE"
		elif code == 2:
			return "PREEMPTED"
		elif code == 3:
			return "SUCCESS"
		else:
			return "Other, see actionlib_msgs/GoalStatus.msg"

if __name__=="__main__":
	try:
		rospy.init_node('zig_zag_demo')
		ZigZag()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Zig Zag Demo terminated.")

