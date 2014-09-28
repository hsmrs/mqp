#!/usr/bin/env python

#This node is used to store, retrieve and remove navigation locations
#It subscribes to the robot's pose and receives IDs to save with
#the poses through the service WaypointServerService
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import OccupancyGrid, MapMetaData
import tf
import numpy
import math


class Sanity:

    def __init__(self):
        #Globals
        self.transform = None

        #Subscribers
        self.listener = tf.TransformListener()

        #Init Functions
        self.init_waypoints()

        self.run()

    def transformPose(self, pose_msg, target_frame):
        from_frame = pose_msg.header.frame_id

        direction = (pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z)
        from_quat = (pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w)
        M = numpy.identity(4) 
        M[:3, 3] = direction[:3]

        EPS = numpy.finfo(float).eps * 4.0 

        q = numpy.array(from_quat[:4], dtype=numpy.float64, copy=True) 
        nq = numpy.dot(q, q) 
        if nq < EPS: 
            N =  numpy.identity(4) 
        else:
            q *= math.sqrt(2.0 / nq) 
            q = numpy.outer(q, q) 
            N =  numpy.array(( 
            (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0), 
            (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0), 
            (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0), 
            (                0.0,                 0.0,                 0.0, 1.0) 
            ), dtype=numpy.float64) 


        from_pose_con = numpy.dot(M, N)

        self.listener.waitForTransform(from_frame, target_frame, rospy.Time(0), rospy.Duration(20))
        transform = self.listener.lookupTransform(from_frame, target_frame, rospy.Time(0))

        tfMatrix = numpy.dot(transform[0], transform[1])

        target_pose_matrix = numpy.dot(tfMatrix, from_pose_con)

        target_xyz = tuple(numpy.array(target_pose_matrix, copy=False)[:3, 3].copy())[:3]

        p = numpy.empty((4, ), dtype=numpy.float64) 
        M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:4, :4] 
        t = numpy.trace(M) 
        if t > M[3, 3]: 
            p[3] = t 
            p[2] = M[1, 0] - M[0, 1] 
            p[1] = M[0, 2] - M[2, 0] 
            p[0] = M[2, 1] - M[1, 2] 
        else: 
            i, j, k = 0, 1, 2 
            if M[1, 1] > M[0, 0]: 
                i, j, k = 1, 2, 0 
            if M[2, 2] > M[i, i]: 
                i, j, k = 2, 0, 1 
            t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3] 
            p[i] = t 
            p[j] = M[i, j] + M[j, i] 
            p[k] = M[k, i] + M[i, k] 
            p[3] = M[k, j] - M[j, k] 
        p *= 0.5 / math.sprt(t * M[3, 3]) 
        
        new_quat = tuple(p)

        r = geometry_msgs.msg.PoseStamped() 
        r.header.stamp = pose_msg.header.stamp 
        r.header.frame_id = target_frame 
        r.pose = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(*target_xyz), geometry_msgs.msg.Quaternion(*new_quat))

        return r

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
            pose_msg.orientation.w = 1;

            pose_stamp_msg.pose = pose_msg

            test_pose = self.transformPose(pose_stamp_msg, "map")


            pose_array_list.append(test)

            self.waypoints.append(pose_stamp_msg)

        self.pose_array.header.frame_id = "map"
        self.pose_array.poses = pose_array_list

        self.pose_array_pub.publish(self.pose_array)

        rospy.loginfo("Waypoints initialized and published!")

    def run(self):
        print self.waypoints

if __name__=="__main__":
    try:
        rospy.init_node('tf_practice')
        Sanity()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("sanity check terminated.")

