#!/usr/bin/env python
"""
Main Control Node
"""
import rospy
import numpy as np
from mas507.msg import ServoSetpoints

if __name__ == '__main__':
    try:
        # Init ROS node
        rospy.init_node('mainController', anonymous=True)

        # Publishers
        pub_servoSetpoints = rospy.Publisher('servoSetpoints', ServoSetpoints, queue_size=1)
        
        # Start Synchronous ROS node execution
        t = 0
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            servoSetpoints = ServoSetpoints()

            servoSetpoints.leftWheel = 330
            servoSetpoints.rightWheel = 330 + 0*40*np.sin(0.1*2*np.pi*t)
            pub_servoSetpoints.publish(servoSetpoints)

            t = t + 0.1

            # Sleep remaining time
            rate.sleep()

    except rospy.ROSInterruptException:
        pass