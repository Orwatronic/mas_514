############################
Kinematics and Rigid Motions
############################

This section will present some handy examples and functions which can be used to model the robot kinematics using matrix algebra in Numpy.
        

*****************
Elbow Manipulator
*****************

.. figure:: ../figs/kinematics/robot.jpg
    :figclass: align-center

    Elbow manipulator with three revolute joints.

Python Implementation
=====================

.. code-block:: python
    :linenos:

    from src import math3d
    import numpy as np

    # Static DH Parameters
    d1 = 2
    a2 = 1.5
    a3 = 1.5

    # Define forward kinematics
    def forward(q, d1, a2, a3):
        # DH Transformations
        T01 = math3d.DH(q[0], d1, 0, np.pi/2)
        T12 = math3d.DH(q[1], 0, a2, 0)
        T23 = math3d.DH(q[2], 0, a3, 0)

        # Trasnform from coordinate {0} -> {3}
        T03 = T01.dot(T12).dot(T23) # or T03 = T01@T12@T23 not T03 = T01*T12*T23

        # Return result p = [x, y, z]
        return T03[0:3,3]


    # Two-Link solution
    def two_link(x, y, a1, a2, conf):
        D = (x**2 + y**2 - a1**2 - a2**2)/(2*a1*a2)

        theta2 = np.arctan2(conf*np.sqrt(1-D**2), D)
        theta1 = np.arctan2(y, x) - np.arctan2(a2*np.sin(theta2), a1 + a2*np.cos(theta2))

        return (theta1, theta2)

    # Define inverse kinematics
    def inverse(p, d1, a2, a3, conf):
        # Create zero array of size 3
        q = np.zeros(3)
        
        # Solve first angle
        q[0] = np.arctan2(p[1], p[0])

        # Calculate T01
        T01 = math3d.DH(q[0], d1, 0, np.pi/2)

        # Convert p to homogenour point in {0}
        P0 = np.ones(4)
        P0[0:3] = p

        # Transform P0 from {0} to {1}
        P1 = math3d.inv(T01).dot(P0)

        # Solve two link problem
        x = P1[0]
        y = P1[1]
        q[1], q[2] = two_link(x, y, a2, a3, conf)
        
        return q


    # Test IK
    q1 = np.array([30, 20, -20])/180*np.pi
    p1 = forward(q1, d1, a2, a3)

    # Test FK
    q2 = inverse(p1, d1, a2, a3, -1)

    # Compare and check that q1=q2
    print(q1)
    print(q2)




.. bibliography:: ../refs.bib
    :style: unsrt
    :cited: