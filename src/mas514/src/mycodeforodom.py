#!/usr/bin/env python
import math
from math import sin, cos, pi
import rospy
from std_msgs.msg import Int16
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


angle_left=0.0
angle_right = 0.0


def left_wheel(data):
	global angle_left 
        angle_left= data.data
        
def right_wheel(data):
	global angle_right 
        angle_right= data.data


if __name__ == '__main__':
	rospy.init_node('odometry_publisher')


	#publisher  the odometry data
	odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
	odom_broadcaster = tf.TransformBroadcaster()

        #get the data from encoders reading
	rospy.Subscriber("angle_left_wheel", Int16, left_wheel)
	rospy.Subscriber("angle_right_wheel", Int16, right_wheel)

	r = 0.03
	b = 0.08
	pi = 3.14
	x = 0.0
	y = 0.0
	th = 0.0

	vx = 0.0
	vy = 0.0
	vth = 0.0

	#theta initialization
	theta_right_new = 0.0
	theta_right_old = 0.0
	theta_left_new = 0.0
	theta_left_old = 0.0

	#time variables
	current_time = rospy.Time.now()
	last_time = rospy.Time.now()


	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
   	    current_time = rospy.Time.now()

	    # compute odometry in a typical way given the velocities of the robot
	    dt = (current_time - last_time).to_sec()
	    theta_right_new = angle_right*pi/180 #get wheel right angle in [rad]
	    theta_left_new = -angle_left*pi/180 #get wheel left angle in [rad]

	    #compute the right and left velocity for wheels
	    wr = (theta_right_new - theta_right_old)/dt #compute right_wheel angular velocity 
	    wl = (theta_left_new - theta_left_old)/dt  #compute right_wheel angular velocity
	    vth = (r/(2*b))*(wr-wl) #compute robot angular velocity
	    vx = (r*(wr+wl))/2   #compute robot speed in x-direction
	    vy = 0 #compute robot speed in y-direction
	    
	    

	    delta_x = (vx * cos(th) - vy * sin(th)) * dt
	    delta_y = (vx * sin(th) + vy * cos(th)) * dt
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
	    
	    theta_right_old = theta_right_new 
	    theta_left_old = theta_left_new  

	    last_time = current_time
	    r.sleep()
