Odometry Information
=====================

The odometry information is needed for the navigation stack in order to work properly, and this information should be sent using tf and nav_msgs/odometry massages.
To do so first, the odometry node should subscribe to the encoder node that built in Arduino using rosserial, to get the right and left angle for the wheels.

Encoder Code
------------

.. literalinclude:: ../../src/Encoder_Code.py
    :language: python

Odom Code
-----------

.. literalinclude:: ../../src/Node.py
    :language: python

And, we can add this code to a launch file in order to run several nodes at the same time.

.. literalinclude:: ../../src/Launch_file_code.py
    :language: python

Odometry Message
-----------------
Using tf
