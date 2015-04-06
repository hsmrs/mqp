#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16

def callback(data):
    global servo1
    global servo2

    x = .25*data.linear.x
    z = .25*data.angular.z
    motorLeft = x - .5*.5*z
    motorRight = x + .5*.5*z

    if motorLeft > 1:
        motorLeft = 1
    elif motorLeft < -1:
        motorLeft = -1
    if motorRight > 1:
        motorRight = 1
    elif motorRight < -1:
        motorRight = -1

    #reverse motor directions
    servo1 = 90 - motorLeft*90
    servo2 = 90 - motorRight*90

# Intializes everything
def start():
    global servo1
    servo1 = 90
    global servo2
    servo2 = 90

    # publishing to servo topics to control robot
    global pub1
    global pub2
    pub1 = rospy.Publisher('/husky/servo_left', UInt16)
    pub2 = rospy.Publisher('/husky/servo_right', UInt16)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("husky/cmd_vel", Twist, callback)
    # starts the node
    rospy.init_node('husky_drive')
    rate = rospy.Rate(30)
    
    while(1):
        pub1.publish(servo1)
        pub2.publish(servo2)
        rate.sleep()

if __name__ == '__main__':
    start()
