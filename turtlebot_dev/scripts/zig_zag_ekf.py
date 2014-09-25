#!/usr/bin/env python

#This node is used to store, retrieve and remove navigation locations
#It subscribes to the robot's pose and receives IDs to save with
#the poses through the service WaypointServerService
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
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
		self.vo_pub = rospy.Publisher('/vo', Odometry)

		#Init Functions
		self.init_waypoints()

		self.run()

	def getTransform(self, frame):
		try:
			(trans, rot) = self.listener.lookupTransform('ar_marker_0', frame, rospy.Time(0))
			return (trans, rot)
		except:
			rospy.loginfo("Ouch!")

	def publishVisualOdom(self):
		x = 0.0;
		y = 0.0;
		th = 0.0;

		vx = 0.1;
		vy = -0.1;
		vth = 0.1;

  		current_time = rospy.Time.now()
  		last_time = rospy.Time.now()

		while not rospy.is_shutdown():
			current_time = rospy.Time.now()

			#compute odometry in a typical way given the velocities of the robot
    		dt = (current_time - last_time).toSec()
    		delta_x = (vx * cos(th) - vy * sin(th)) * dt
    		delta_y = (vx * sin(th) + vy * cos(th)) * dt
			delta_th = vth * dt

    		x += delta_x;
    		y += delta_y;
    		th += delta_th;

    		#since all odometry is 6DOF we'll need a quaternion created from yaw
    		odom_quat = tf.transformations.quaternion_from_euler(0, 0, th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;

	def init_waypoints(self):
		self.waypoints = []

		x = 0.0
		y = -0.2
		dx = 1.0
		dy = 0.2

		for i in range(16):
			
			counter = i % 4

			if counter == 0:
				x = 0.0
				y += dy
			elif counter == 1:
				x = 1.0
				y = y 
			elif counter == 2:
				x = 1.0
				y += dy
			elif counter == 3:
				x = 0.0
				y = y0

			pose = PoseStamped()
			header = Header()
			header.frame_id = "map"

			pose.header = header
			pose.pose.position.x = x
			pose.pose.position.y = y

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
			if self.areWeThereYet():
				self.nextGoal()

if __name__=="__main__":
	try:
		rospy.init_node('zig_zag_demo')
		ZigZag()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Zig Zag Demo terminated.")

