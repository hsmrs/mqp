#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from math import cos, sin, sqrt, pi

enc_const = .02/1.5     #m/s per tick count
dt = 1.0/20.0           #average amount of time between samples
B = .5                  #meter distance between wheels

def cb_Left(data):
    global left_vel
    left_vel = float(data.data) * enc_const

def cb_Right(data):
    global right_vel
    right_vel = float(data.data) * enc_const

def start():
    global left_vel
    global right_vel
    left_vel = 0.0
    right_vel = 0.0

    global poseX
    poseX = 0.0
    global poseY
    poseY = 0.0
    global poseR
    poseR = 0.0

    rospy.Subscriber("husky/encoderLeft_vel", Int16, cb_Left)
    rospy.Subscriber("husky/encoderRight_vel", Int16, cb_Right)
    pub = rospy.Publisher('husky/odom', Odometry)
    rospy.init_node('husky_odom')

    left_initial = 0.0          #left_vel temp
    right_initial = 0.0         #right_vel temp
    #before = rospy.Time.now()   #time temp
    msg = Odometry()            #message to be published
    seq = 0
    msg.header.frame_id = "husky/odom";

    theta = 0.0                 #orientation in radians
    d1 = 0.0                    #right wheel distance traveled
    d2 = 0.0                    #left wheel distance traveled
    rate = rospy.Rate(1/dt)     

    while(1):
        now = rospy.Time.now()
        d1 = right_initial * dt
        d2 = left_initial * dt 
        
        #get change in pose
        D = .5*(d1 + d2)
        poseX += D*cos(theta)
        poseY += D*sin(theta)     
        poseR += (d1 - d2)/B
        quat = quaternion_from_euler(0, 0, poseR)


        #update variables
        msg.header.stamp = now
        msg.header.seq = seq
        msg.pose.pose.position.x = poseX
        msg.pose.pose.position.y = poseY
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        #and update temps
        #before = now
        theta = poseR
        left_initial = left_vel
        right_initial = right_vel
        if (seq%20) == 0:
            print("left", str(left_initial), "right", str(right_initial), "theta", str(theta))
        

        #pub odom
        pub.publish(msg)
        seq += 1
        rate.sleep()
        
       

if __name__ == '__main__':
    start()
