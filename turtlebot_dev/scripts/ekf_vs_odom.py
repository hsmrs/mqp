#!/usr/bin/env python

#This node is used to store, retrieve and remove navigation locations
#It subscribes to the robot's pose and receives IDs to save with
#the poses through the service WaypointServerService
import rospy
from std_msgs.msg import Header, String
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import OccupancyGrid, MapMetaData
import tf
import numpy
import math


class Analysis():

    def __init__(self):
        #Globals
        self.odom = PoseWithCovarianceStamped()
        self.ekf_odom = PoseWithCovarianceStamped
        self.odom_updated = False
        self.ekf_updated = False

        #Subscribers
        self.pose_sub = rospy.Subscriber("/robot_pose_ekf/odom", Odometry, self.ekf_pose_callback)
        self.pose_sub = rospy.Subscriber("/odom", PoseWithCovarianceStamped, self.odom_callback)

        #Publishers
        self.dif_pub = rospy.Publisher("/odom_differences", String)

        #Init Functions

        self.run()

    def ekf_pose_callback(self, msg):
        self.ekf_odom = msg
        self.ekf_updated = True


    def odom_callback(self, msg):
        self.odom = msg.pose
        self.odom_updated = True

    def publishDifference(self):
        dx = self.odom.pose.pose.position.x - self.ekf_odom.pose.pose.position.x
        dy = self.odom.pose.pose.position.y - self.ekf_odom.pose.pose.position.y
        dz = self.odom.pose.pose.position.z - self.ekf_odom.pose.pose.position.z

        msg = "delta x: " + str(dx) + "\n" + \
                "delta y: " + str(dy) + "\n" + \
                "delta z: " + str(dz) + "\n"

        self.dif_pub.publish(msg)


    def run(self):
        if self.odom_updated and self.ekf_updated:
            self.odom_updated = False
            self.ekf_updated = False
            self.publishDifference()


if __name__=="__main__":
    try:
        rospy.init_node('ekf_vs_odom')
        Analysis()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ekf odom analysis terminated.")
