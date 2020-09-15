#!/usr/bin/env python
"""
Node for capturing Jetbot camera
"""

import os
import rospy
import cv2
from cv2 import aruco
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int64

class JetbotCamera(object):
    # Set ccamera capture settings
    FRAME_RATE = 30
    VIDEO_WIDTH = 720
    VIDEO_HEIGHT = 540

    def __init__(self):        
        # Initilize GStreamer capture
        self.capture = self.get_gstreamer_capture()

        # Load camera calibration matrix from  Python calibration
        data = np.load('%s/catkin_ws/src/mas507/data/intrinsicCalibration.npz' % (os.path.expanduser("~")))
        self.calibration_matrix = data['mtx']
        self.distortion_parameters = data['dist']

        # Load color calibration matrix
        data = np.load('%s/catkin_ws/src/mas507/data/colorCalibration.npz' % (os.path.expanduser("~")))
        self.gain_matrix = data["arr_0"] # Original load without fix
        data.close()

        # Splits up calibration into making a map, and doing the remapping in calibrate
        # member function
        # This is faster than doing cv2.undistort()
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.calibration_matrix,
            self.distortion_parameters,
            np.eye(3),
            self.calibration_matrix,
            (self.VIDEO_WIDTH, self.VIDEO_HEIGHT),
            cv2.CV_32FC1
        )

    def read(self):
        # Read raw image
        _, cv_raw = self.capture.read()
    
        # Perform color calibration
        image_float = np.array(cv_raw, float) * self.gain_matrix

        # Clip to keep values between 0 and 255 
        # Using [:] sets the value of object "image" without making a new object
        # i.e. just replaces in-object which does not need a return statement        
        cv_raw[:] = np.clip(image_float, 0, 255)

        return cv_raw

    def calibrate(self, image):
        """
        Perform calibration of input OpenCV image
        """

        # Make copy of input image

        # # Perform color calibration
        # image_float = np.array(image, float) * self.gain_matrix

        # # Clip to keep values between 0 and 255 
        # # Using [:] sets the value of object "image" without making a new object
        # # i.e. just replaces in-object which does not need a return statement        
        # calibrated[:] = np.clip(image_float, 0, 255)

        # Apply camera distortion calibration
        calibrated = image.copy()
        calibrated[:] = cv2.remap(calibrated, self.map1, self.map2, cv2.INTER_LINEAR)
        return calibrated

    
    # Private methods
    def get_gstreamer_capture(self):
        """
        gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
        Defaults to 1280x720 @ 60fps
        Flip the image by setting the flip_method (most common values: 0 and 2)
        display_width and display_height determine the size of the window on the screen
        """
        gstreamer_pipeline = (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink drop=true sync=false"
            % (
                self.VIDEO_WIDTH,
                self.VIDEO_HEIGHT,
                self.FRAME_RATE,
                self.VIDEO_WIDTH,
                self.VIDEO_HEIGHT,
            )
        )

        # Using default values in gstreamer_pipeline to capture video
        video_capture = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)
        return video_capture


if __name__ == '__main__':
    # Tries to start image publisher if roscore is properly running
    try:
        # Initialize nodes
        rospy.init_node('jetbotCamera')

        # Initilize Jetbot camera instance
        camera = JetbotCamera()

        # CvBridge for converting cv2 to ROS images
        bridge = CvBridge()

        # ROS Image Publishers
        pub_raw = rospy.Publisher('image_raw', Image, queue_size=1)
        pub_calibrated = rospy.Publisher('image_calibrated', Image, queue_size=1)
        pub_markers = rospy.Publisher('image_markers', Image, queue_size=1)

        # Start Synchronous ROS node execution
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Read raw image
            cv_raw = camera.read()

            # Calibrate raw image
            cv_calibrated = camera.calibrate(cv_raw)

            # Detect Aruco markers
            cv_gray = cv2.cvtColor(cv_calibrated, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
            parameters = aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(
                cv_gray, aruco_dict, parameters=parameters
            )

            # Print Aruco marker detection onto image
            cv_markers = aruco.drawDetectedMarkers(cv_calibrated.copy(), corners, ids)

            # Calculate Aruco marker pose
            marker_size = 0.11 # in meters
            rvecs, tvecs, trash = aruco.estimatePoseSingleMarkers(
                corners, marker_size, camera.calibration_matrix, camera.distortion_parameters
            )

            # Print coordinate system onto markers
            if ids is not None:
                for i in range(0, len(ids)):
                    cv_markers = aruco.drawAxis(
                        cv_markers,
                        camera.calibration_matrix,
                        camera.distortion_parameters,
                        rvecs[i],
                        tvecs[i],
                        marker_size/2
                    )
                    
                    # Convert to rotation matrix in SO(3)
                    R, jacobian = cv2.Rodrigues(rvecs[i])

                    print('x={}, y={}, z={}'.format(tvecs[i][0,0], tvecs[i][0,1], tvecs[i][0,2]))
                    
            
            # Publish images to ROS messages
            pub_raw.publish(bridge.cv2_to_imgmsg(cv_raw, "bgr8"))        
            pub_calibrated.publish(bridge.cv2_to_imgmsg(cv_calibrated, "bgr8"))
            pub_markers.publish(bridge.cv2_to_imgmsg(cv_markers, "bgr8"))

            # Sleep remaining time
            rate.sleep()


    except rospy.ROSInterruptException:
        pass
