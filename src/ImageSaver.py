#!/usr/bin/env python
"""
Node for saving Raw Jetbot images
"""
import os
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

class ImageSaver(object):
    idx = 0

    def __init__(self):
        self.imagePath = '%s/catkin_ws/src/mas507/temp/' % (os.path.expanduser("~"))
        self.cvBridge = CvBridge()

        # Ensure folder exist and create folder if non-existing
        if not os.path.isdir(self.imagePath):
            os.mkdir(self.imagePath)

    def callback(self, msg):
        if msg.data:
            # Read RAW image
            image = rospy.wait_for_message('/image_raw', Image)

            # Try to save image
            try:
                cv_image = self.cvBridge.imgmsg_to_cv2(image)
                filename = self.imagePath + "image" + str(self.idx) + ".png"
                cv2.imwrite(filename, cv_image)

            except CvBridgeError as e:
                print(e)

            # Increment image counter
            self.idx += 1

            # Reset ROS trigger message
            out = Bool()
            out = False
            pub.publish(out)

            # Print info to console
            print("Saved: " + filename)


if __name__ == '__main__':
    try:
        # Initialize node
        rospy.init_node('imageSaver')

        # Publisher
        pub = rospy.Publisher('saveImage', Bool, queue_size=1)

        # Image saving class
        imageSaver = ImageSaver()
        
        # Subscribers
        rospy.Subscriber("saveImage", Bool, imageSaver.callback)

        # Keep node alive
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


