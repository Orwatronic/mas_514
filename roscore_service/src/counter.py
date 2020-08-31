#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64
if __name__ == "__main__":
    rospy.init_node("counter_publisher")
    rate = rospy.get_param("/counter_publisher_rate")
    counter = 0
    pub = rospy.Publisher("counter", Int64, queue_size=1)
    rate = rospy.Rate(rate)
    rospy.loginfo("Starting publishing...")
    while not rospy.is_shutdown():
        pub.publish(counter)
        counter += 1
        rate.sleep()