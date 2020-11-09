Camera Calibration
==================

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

Rectification
-------------
The next step is to rectify the image, which is carried out in the :code:`JetbotCamera` node. Before restarting the MAS507 package, please copy or move the :code:`intrinsicCalibration.npz` file from :code:`~/catkin_ws/src/mas507/calibration/` to :code:`~/catkin_ws/src/mas507/data/` and launch the package using :code:`roslaunch mas507 start.launch`.

The raw image and the calibrated image can now be inspected using the WebViz application.

Color Calibration
-----------------
The image is also color calibrated, but this step is not supposed to be changed by the students since the lecturer have already ensured that all the Jetbot cameras has been calibrated for possible color distortions. The color calibration procedure is seen also in the :code:`JetbotCamera` node, where the color calibration matrix is loaded from :code:`colorCalibration.npz` also found in the :code:`~/catkin_ws/src/mas507/data/` folder. For more information about the color calibration procedure, please check out :download:`this lecture on color calibration <../pdfs/Lec3_colorCalibration.pdf>`.

If another image resolution is required, the color calibration has to be executed once again. The steps are the following:

- Capture at least 20 images of a white wall or a piece of paper in decent light conditions.
- Save these images to a suitable location e.g. a folder named :code:`./whiteimages`.
- Create a file with the below content in the parent folder of :code:`./whiteimages` and execute the Python script. Remember to change the parameters for width and height according to the correct resolution.

.. literalinclude:: ../../calibration/calibrateColor.py
    :language: python




