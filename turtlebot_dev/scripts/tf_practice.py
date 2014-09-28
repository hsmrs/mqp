#!/usr/bin/env python

#This node is used to store, retrieve and remove navigation locations
#It subscribes to the robot's pose and receives IDs to save with
#the poses through the service WaypointServerService
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, PoseArray, Point, Quaternion
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

    def quaternion_matrix(self, quaternion): 
        """Return homogeneous rotation matrix from quaternion. 

        >>> R = quaternion_matrix([0.06146124, 0, 0, 0.99810947]) 
        >>> numpy.allclose(R, rotation_matrix(0.123, (1, 0, 0))) 
        True 

        """ 
        q = numpy.array(quaternion[:4], dtype=numpy.float64, copy=True) 
        nq = numpy.dot(q, q) 
        if nq < 0.1: 
          return numpy.identity(4) 
        q *= math.sqrt(2.0 / nq) 
        q = numpy.outer(q, q) 
        return numpy.array(( 
          (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0), 
          (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0), 
          (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0), 
          (                0.0,                 0.0,                 0.0, 1.0) 
          ), dtype=numpy.float64)

    def translation_matrix(self, direction): 
        """Return matrix to translate by direction vector.  

        >>> v = numpy.random.random(3) - 0.5 
        >>> numpy.allclose(v, translation_matrix(v)[:3, 3]) 
        True 

        """ 
        M = numpy.identity(4) 
        M[:3, 3] = direction[:3] 
        return M 

    def translation_from_matrix(self, matrix): 
        """Return translation vector from translation matrix. 
    
        >>> v0 = numpy.random.random(3) - 0.5 
        >>> v1 = translation_from_matrix(translation_matrix(v0)) 
        >>> numpy.allclose(v0, v1) 
        True 
    
        """ 
        return numpy.array(matrix, copy=False)[:3, 3].copy() 

    def quaternion_from_matrix(self, matrix): 
        """Return quaternion from rotation matrix. 

        >>> R = rotation_matrix(0.123, (1, 2, 3)) 
        >>> q = quaternion_from_matrix(R) 
        >>> numpy.allclose(q, [0.0164262, 0.0328524, 0.0492786, 0.9981095]) 
        True 

        """ 
        q = numpy.empty((4, ), dtype=numpy.float64) 
        M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:4, :4] 
        t = numpy.trace(M) 
        if t > M[3, 3]: 
          q[3] = t 
          q[2] = M[1, 0] - M[0, 1] 
          q[1] = M[0, 2] - M[2, 0] 
          q[0] = M[2, 1] - M[1, 2] 
        else: 
          i, j, k = 0, 1, 2 
          if M[1, 1] > M[0, 0]: 
              i, j, k = 1, 2, 0 
          if M[2, 2] > M[i, i]: 
              i, j, k = 2, 0, 1 
          t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3] 
          q[i] = t 
          q[j] = M[i, j] + M[j, i] 
          q[k] = M[k, i] + M[i, k] 
          q[3] = M[k, j] - M[j, k] 
        q *= 0.5 / math.sqrt(t * M[3, 3]) 
        return q 

    def fromTranslationRotation(self, translation, rotation): 
        """ 
        :param translation: translation expressed as a tuple (x,y,z) 
        :param rotation: rotation quaternion expressed as a tuple (x,y,z,w) 
        :return: a :class:`numpy.matrix` 4x4 representation of the transform 
        :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise 

        Converts a transformation from :class:`tf.Transformer` into a representation as a 4x4 matrix. 
        """ 

        return numpy.dot(self.translation_matrix(translation), self.quaternion_matrix(rotation))

    def xyz_to_mat44(self, pos): 
        return self.translation_matrix((pos.x, pos.y, pos.z)) 

    def xyzw_to_mat44(self, ori): 
        return self.quaternion_matrix((ori.x, ori.y, ori.z, ori.w))

    def asMatrix(self, target_frame, hdr): 
        """ 
        :param target_frame: the tf target frame, a string 
        :param hdr: a message header 
        :return: a :class:`numpy.matrix` 4x4 representation of the transform 
        :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise 

        Uses :meth:`lookupTransform` to look up the transform for ROS message header hdr to frame 
        target_frame, and returns the transform as a :class:`numpy.matrix` 
        4x4. 
        """ 
        self.listener.waitForTransform(target_frame, hdr.frame_id, hdr.stamp, rospy.Duration(40))
        translation,rotation = self.listener.lookupTransform(target_frame, hdr.frame_id, hdr.stamp) 
        return self.fromTranslationRotation(translation, rotation) 

    def transformPose(self, target_frame, ps): 
        """
        :param target_frame: the tf target frame, a string 
        :param ps: the geometry_msgs.msg.PoseStamped message 
        :return: new geometry_msgs.msg.PoseStamped message, in frame target_frame 
        :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise 

        Transforms a geometry_msgs PoseStamped message to frame target_frame, returns a new PoseStamped message. 
        """ 
        # mat44 is frame-to-frame transform as a 4x4 
        mat44 = self.asMatrix(target_frame, ps.header) 

        # pose44 is the given pose as a 4x4 
        pose44 = numpy.dot(self.xyz_to_mat44(ps.pose.position), self.xyzw_to_mat44(ps.pose.orientation)) 

        # txpose is the new pose in target_frame as a 4x4 
        txpose = numpy.dot(mat44, pose44) 

        # xyz and quat are txpose's position and orientation 
        xyz = tuple(self.translation_from_matrix(txpose))[:3] 
        quat = tuple(self.quaternion_from_matrix(txpose)) 

        # assemble return value PoseStamped 
        r = PoseStamped() 
        r.header.stamp = ps.header.stamp 
        r.header.frame_id = target_frame 
        r.pose = Pose(Point(*xyz), Quaternion(*quat)) 
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

            test_pose = self.transformPose("map", pose_stamp_msg)


            pose_array_list.append(test_pose)

            self.waypoints.append(test_pose)

        self.pose_array.header.frame_id = "map"
        self.pose_array.poses = pose_array_list

        #self.pose_array_pub.publish(self.pose_array)

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

