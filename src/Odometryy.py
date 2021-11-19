# -*- coding: utf-8 -*-
"""
Created on Wed Nov 17 12:17:57 2021

@author: modal
"""

#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
from std_msgs.msg import Int16
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


#Initial values
angle_left = 0.0
angle_right = 0.0


def left_wheel(angleL):
    global angle_left
    angle_left = angleL.data

def right_wheel(angleR):
    global angle_right
    angle_right = angleR.data   

if __name__ == '__main__':
    try:
        rospy.init_node('odometry_publisher')

        odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        odom_broadcaster = tf.TransformBroadcaster()

        subLeftWheel = rospy.Subscriber("angle_left_wheel", Int16, left_wheel)
        subRightWheel = rospy.Subscriber("angle_right_wheel", Int16, right_wheel)
        
        #Jetbot parameters
        b = 0.058
        R = 0.03

        angle2rad = (math.pi)/180
        
        #Initial values
        x = 0.0
        y = 0.0
        th = 0.0
        vx = 0
        vy = 0
        vth = 0
        thetaL = 0.0
        thetaR = 0.0
        thetaL_old = 0.0
        thetaR_old = 0.0

        current_time = rospy.Time.now()
        last_time = rospy.Time.now()

        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()

            # Convert Encoder signal in deg to rad
            thetaL = -angle_left*angle2rad
            thetaR = angle_right*angle2rad

            # Estimate wheel velocity
            omegaL = (thetaL - thetaL_old)/dt #rad/s
            omegaR = (thetaR - thetaR_old)/dt
            # Forward Kinematics
            vth = (R/(2*b))*(omegaR-omegaL)
            vx = 0.5*R*(omegaR + omegaL)
            vy = 0.0

            #print("omegaR")
            #print(omegaR)

            #print("omegaL:")
            #print(omegaL)

            # compute odometry in a typical way given the velocities of the robot
            delta_x = (vx * cos(th)) * dt
            delta_y = (vx * sin(th)) * dt
            delta_th = vth * dt

            x += delta_x
            y += delta_y
            th += delta_th

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

            # first, we'll publish the transform over tf
            odom_broadcaster.sendTransform(
                (x, y, 0.),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

            # publish the message
            odom_pub.publish(odom)

            last_time = current_time
            thetaL_old = thetaL
            thetaR_old = thetaR
            r.sleep()

    except rospy.ROSInterruptException:
        pass