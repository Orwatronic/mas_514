#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
from std_msgs.msg import Int16
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

#Initial values
v_x=0
v_th=0

def cmd_vel_callback(msg):
	global v_x
	global v_th 
	v_x=msg.linear.x
	v_th=msg.angular.z    

if __name__ == '__main__':
    try:
        rospy.init_node('odometry_publisher')

        odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        odom_broadcaster = tf.TransformBroadcaster()

        sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
        
        #Jetbot parameters
        b = 0.058 # half distance between center of wheels
        R = 0.03 # wheel radius

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

        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()

            # Velocity of robot
            vx = v_x
            vy = 0.0
            vth = v_th

            # Distance traveled since last cycle in global frame
            delta_x = cos(vth) * vx
            delta_y = -sin(vth) * vx

            # Robot Position in global frame
            x += (cos(th) * delta_x  - sin(th) * delta_y)
            y += (sin(th) * delta_x + cos(th) * delta_y)    
            th += vth

            #print("omegaL")
            #print(omegaL)

            #print("omegaR:")
            #print(omegaR)

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
