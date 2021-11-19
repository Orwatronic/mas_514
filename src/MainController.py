#!/usr/bin/env python
"""
Main Control Node
"""
import rospy
import numpy as np
import os
from mas514.msg import ServoSetpoints, WebJoystick
from sensor_msgs.msg import Image
from StrawberryDetector import StrawberryDetector

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

        # Publishers
        pub_servoSetpoints = rospy.Publisher('servoSetpoints', ServoSetpoints, queue_size=1)
        pub_strawberry_detection = rospy.Publisher('strawberry_detection', Image, queue_size=1)
        
        # Strawberry detector
        intrinsicCalibration =  np.load('%s/catkin_ws/src/mas514/data/intrinsicCalibration.npz' % (os.path.expanduser("~")))
        strawberryDetector = StrawberryDetector(pub_strawberry_detection, intrinsicCalibration['mtx'], intrinsicCalibration['dist'])

        # Subscribers
        sub_calibrated = rospy.Subscriber('image_calibrated', Image, strawberryDetector.callback)
        sub_leftJoystick = rospy.Subscriber('webJoystickLeft', WebJoystick, leftJoystick.callback)
        sub_rightJoystick = rospy.Subscriber('webJoystickRight', WebJoystick, rightJoystick.callback)

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
