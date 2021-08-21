#!/usr/bin/env python
"""
Node for controlling Jetbot robot
"""
import rospy
import numpy as np
from std_msgs.msg import Int64
from mas514.msg import ServoSetpoints
import Adafruit_PCA9685

class JetbotController(object):
    def __init__(self):
        # Adafruit PCA board instance setup
        self.adafruit = Adafruit_PCA9685.PCA9685(busnum=1)
        self.adafruit.set_pwm_freq(50)
        
    def setPwm(self, u):
        """
        Set RC parameter for wheel motors driven by Adafruit
        PCA9685 with 12 bit resolution (2^12 = 4096) for each servo
        On 50 Hz rate (20 ms period), 409 ticks=2ms, 204ticks=1ms

        Parameters
        ----------
        u[i]:
            Uptime of RC signal in ticks
            Between [204 and 408]
            where, theoretically, 306 is stand-still, 204 is maximum reverse
            and 408 is maximum forward.

            i=0: right motor [204..408]
            i=1: left motor [204..408]
            i=2: servo 1 [204..816]
            i=3: servo 2 [204..816]
            i=4: servo 3 [204..816]
        """

        for i in range(0, 5):
            # Lower RC protocoll limit
            u[i] = max(204, u[i])

            # Upper RC protocoll limit
            if i < 2:
                # Upper limit for wheel motors
                u[i] = min(408, u[i])

            else:
                # Upper limit for arm motors
                u[i] = min(816, u[i])

            
            # Set PWM
            self.adafruit.set_pwm(i, 0, u[i])



if __name__ == '__main__':
    try:
        # Init ROS node
        rospy.init_node('jetbotController', anonymous=True)

        # Jetbot controller
        jetbotController = JetbotController()

        # Callback for handling incoming ROS message
        def callback(msg):
            # Stack ROS message to array
            u = [
                msg.rightWheel,
                msg.leftWheel,
                msg.servo1,
                msg.servo2,
                msg.servo3
            ]
            
            # Set Servo PWMs
            jetbotController.setPwm(u)

        # Async subscription to servoSetpoints message
        rospy.Subscriber("servoSetpoints", ServoSetpoints, callback)
        
        # # Keep ROS node alive
        rospy.spin()

        


    except rospy.ROSInterruptException:
        pass

        



