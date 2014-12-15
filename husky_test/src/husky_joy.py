#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# Author: Andrew Dai
# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
def callback(data):
    global twist 
    twist.linear.x = 4*data.axes[1]
    twist.angular.z = 4*data.axes[0]

def getTwist():
    global twist
    return twist

# Intializes everything
def start():
    global twist 
    twist = Twist()
    # publishing to "turtle1/cmd_vel" to control turtle1
    global pub
    pub = rospy.Publisher('husky/cmd_vel', Twist)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.init_node('Joy2Husky')
    rate = rospy.Rate(30)
    
    while(1):
        twist = getTwist()
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    start()

