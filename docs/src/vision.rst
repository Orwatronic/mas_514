Vision
======

The Jetbot camera is supposed to be used to detect both the Aruco markers for navigating and the strawberries to pick with the developed robot arm. It is therefore crucial to calibrate the camera before starting to develop the image processing functionality. The calibration is carried out in 3 major steps.

Capture images
--------------
Capture N images of a checkerboard from random poses (position + orientation) around  the "typical" working distance and store the images for calibration. It is recommended to use about 20 images for the calibration process. The MAS507 ROS Package features a ImageSaving node which typically is used to save the images from the Jetbot while the WebViz application is used to see what the camera is "looking" at while capturing the images. Save the images using the following command after that ROS core and the MAS507 ROS package are started:

- :code:`rostopic pub saveImage std_msgs/Bool 1`
- When the image is saved to :code:`~/catkin_ws/src/mas507/calibration/images/`  execute :code:`CTRL + C` to terminate the saving command after that the image is saved.
- Repeat N times.

Calibration using OpenCV
------------------------
The calibration is done by executing the calibration Python script inside the :code:`~/catkin_ws/src/mas507/calibration` folder. The script looks like this:

.. literalinclude:: ../../calibration/calibrateCamera.py
    :language: python

This script will save the calibration data to a :code:`.npz` file which will be loaded by the ImageProcessing node later on to rectify the raw image captured by the camera. 



