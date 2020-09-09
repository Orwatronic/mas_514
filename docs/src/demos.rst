###############
Classroom Demos
###############

*************************
Lecture 4: ROS and Python
*************************

Synchronous Publisher
=====================

.. code-block:: python

    #!/usr/bin/env python
    """
    DemoPublisher Lecture 4

    Try to echo node topic counter by executing:
    rostopic echo counter
    """

    # Import needed libraries
    import rospy
    from std_msgs.msg import Int32

    # Main function
    if __name__ == '__main__':
        #!/usr/bin/env python
    """
    DemoSyncPublisher

    Try to echo node topic counter by executing:
    rostopic echo counter
    """

    # Import needed libraries
    import rospy
    from std_msgs.msg import Int32

    # Main function
    if __name__ == '__main__':
        try:
            # Initialize node
            rospy.init_node('demoPublisher')

            # Publisher
            pub = rospy.Publisher('counter', Int32, queue_size=1)

            # Counter variable
            counter = 0

            # Create a while loop at 1s
            rate = rospy.Rate(1)
            while not rospy.is_shutdown():
                # counter += 1
                counter = counter + 1

                # Publish counter to ROS message
                pub.publish(counter)

                # Sleep remaining time
                rate.sleep()

        except rospy.ROSInterruptException:
            pass


Asynchronous Subscriber
=======================

.. code-block:: python

    #!/usr/bin/env python
    """
    DemoAsyncSubscriber
    """

    # Import needed libraries
    import rospy
    from std_msgs.msg import Int32

    # Main function
    if __name__ == '__main__':
        try:
            # Initialize node
            rospy.init_node('demoSubscriber')

            # Create callaback to handle data from subscriber
            def callback(msg):
                if msg.data:
                    print('Recieved counter=' + str(msg.data))

            # Subscriber
            rospy.Subscriber('counter', Int32, callback)

            # Keep node alive
            rospy.spin()
            
        except rospy.ROSInterruptException:
            pass


Asynchronous Subscriber
=======================

.. code-block:: python

    #!/usr/bin/env python
    """
    DemoAsyncClassSubscriber
    """

    # Import needed libraries
    import rospy
    from std_msgs.msg import Int32

    # Create ClassDef here
    class MyRosSubscriber(object):
        def __init__(self, gain, name):
            self.gain = gain
            self.name = name

        def callback(self, msg):
            print(self.name + ': counter*self.gain = ' + str(msg.data*self.gain))


    # Main function
    if __name__ == '__main__':
        try:
            # Initialize node
            rospy.init_node('demoClassSubscriber')

            # Create class instance
            myRosSubscriber1 = MyRosSubscriber(2, 'class1')
            myRosSubscriber2 = MyRosSubscriber(4, 'class2')
            
            # Subscribers
            rospy.Subscriber('counter', Int32, myRosSubscriber1.callback)
            rospy.Subscriber('counter', Int32, myRosSubscriber2.callback)

            # Keep node alive
            rospy.spin()
            
        except rospy.ROSInterruptException:
            pass

Python Class
============

.. code-block:: python

    # Imports Numpy package as np for easier usage later e.g. np.sin(1)
    import numpy as np

    # Class definition for MyClass
    class MyClass(object):
        def __init__(self, a, b=3):
            # Init is called when class instance is created
            print('Hi, MyClass was created!')

            # Copy input arguments to self which is the class object
            self.a = a
            self.b = b

        def print_a_and_b(self):
            # Print a and b
            print('a=' + str(self.a) + ' b=' + str(self.b))

        def __del__(self):
            # This function is called when the class is deleted
            print('I was deleted')

    # Demo usage of MyClass
    if __name__ == '__main__':
        # Creates class instance with a and b given
        myClass1 = MyClass(30, 40)

        # Creates class instance with only a given
        myClass2 = MyClass(-4)

        # Print a and b
        myClass1.print_a_and_b()
        myClass2.print_a_and_b()
        
        # Print class variables a and b for myClass1 instance of MyClass
        print(myClass1.a)
        print(myClass1.b)