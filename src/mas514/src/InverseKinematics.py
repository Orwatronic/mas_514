#!/usr/bin/env python
"""
InverseKinematic Node
"""
import rospy
import numpy as np
import os
from mas514.msg import ServoSetpoints #, WebJoystick
from geometry_msgs.msg import Twist
#from sensor_msgs.msg import Image

vx = 0.0
vth = 0.0
def cmd_vel_callback(msg):
	global vx
	global vth
	vx = msg.linear.x
	vth = msg.angular.z



if __name__ == '__main__':
    try:
        # Init ROS node
        rospy.init_node('InverseKinematics', anonymous=True)


        pi = 3.141592653589793
        # JetBot Parameters [m]
        b = 0.058
        r = 0.03
	# Subscribers
	sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)

        # Publishers
        pub_servoSetpoints = rospy.Publisher('servoSetpoints', ServoSetpoints, queue_size=1)

        # Start Synchronous ROS node execution
        t = 0
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            #start here
            v = vx 
            omega = vth



        # Inverse Kinematics - varpi = wheel speed [rad/s]
            varpi_l = (v-(b*omega))/r
            varpi_r = (v+(b*omega))/r
        
        # Motor Parameter max/min speed [rad/s] (data sheet in canvas, not fully sure if it is +-120 or +-140 rpm that is correct) 
            varpi_max_fwd = 140*(2*pi)/60
            varpi_max_bwd = -140*(2*pi)/60
        
        # Motor Drive Parameters
            u_max_fwd =  204
            u_max_bwd =  408
            u_zero_left = 309.25 #320.75
            u_zero_right = 309.25

        #Linear interpolation - angular velocity to motor drive input ticks
            x = [varpi_max_bwd, 0, varpi_max_fwd]
        # The left and the right motor have opposite sign on the input signal since they goes in oppesite direction due to the mounting on the robot frame
            y_left = [u_max_fwd, u_zero_left, u_max_bwd]
            y_right = [u_max_bwd, u_zero_right, u_max_fwd]
  
            u_left = np.interp(varpi_l, x, y_left)
        
            #print("varpi_l:")
            #print(varpi_l)

            #print("u_left:")
            #print(u_left)

            u_right = np.interp(varpi_r, x, y_right)

            #print("varpi_r:")
            #print(varpi_r)

            #print("u_right:")
            #print(u_right)
            #to here
            servoSetpoints = ServoSetpoints()

            servoSetpoints.leftWheel  = u_left
            servoSetpoints.rightWheel = u_right

            pub_servoSetpoints.publish(servoSetpoints)

            t = t + 0.1

            # Sleep remaining time
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
