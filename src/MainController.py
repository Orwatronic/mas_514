#!/usr/bin/env python
"""
Main Control Node
"""
import rospy
import numpy as np
from mas507.msg import ServoSetpoints, WebJoystick

class Joystick(object):
    def __init__(self):
        self.x = 0
        self.y = 0

    def callback(self, msg):
        self.x = msg.x
        self.y = msg.y

if __name__ == '__main__':
    try:
        # Init ROS node
        rospy.init_node('mainController', anonymous=True)

        # Web Joysticks
        leftJoystick = Joystick()
        rightJoystick = Joystick()

        # Subscribers
        sub_leftJoystick = rospy.Subscriber('webJoystickLeft', WebJoystick, leftJoystick.callback)
        sub_rightJoystick = rospy.Subscriber('webJoystickRight', WebJoystick, rightJoystick.callback)

        # Publishers
        pub_servoSetpoints = rospy.Publisher('servoSetpoints', ServoSetpoints, queue_size=1)
        
        # Start Synchronous ROS node execution
        t = 0
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            servoSetpoints = ServoSetpoints()

            servoSetpoints.leftWheel  = 315 - rightJoystick.y/2 + leftJoystick.x/7
            servoSetpoints.rightWheel = 305 + rightJoystick.y/2 + leftJoystick.x/7
            pub_servoSetpoints.publish(servoSetpoints)

            t = t + 0.1

            # Sleep remaining time
            rate.sleep()

    except rospy.ROSInterruptException:
        pass