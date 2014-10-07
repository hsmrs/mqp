#!/usr/bin/env python

#This node is used to store, retrieve and remove navigation locations
#It subscribes to the robot's pose and receives IDs to save with
#the poses through the service WaypointServerService
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import OccupancyGrid, MapMetaData
from ar_track_alvar.msg import AlvarMarkers
import tf
from tf.transformations import euler_from_quaternion
import numpy
import math


class GetAngles:

    def __init__(self):

        #Subscribers
        self.pose_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.pose_callback)
        
        rospy.loginfo("Initializing")

    def pose_callback(self, pose_msg):
        x = pose_msg.markers[0].pose.pose.orientation.x
        y = pose_msg.markers[0].pose.pose.orientation.y
        z = pose_msg.markers[0].pose.pose.orientation.z
        w = pose_msg.markers[0].pose.pose.orientation.w
        '''
        print "x =", x
        print "y =", y
        print "z =", z
        print "w =", w
        '''
        (roll,pitch,yaw) = euler_from_quaternion([x,y,z,w])
        
        print "yaw =", math.degrees(yaw)

if __name__=="__main__":
    try:
        rospy.init_node('ar_pose_angle_node')
        GetAngles()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("no more angle for you.")

