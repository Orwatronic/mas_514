############################
Kinematics and Rigid Motions
############################

This section will present some handy examples and functions which can be used to model the robot kinematics using matrix algebra in Numpy.
        

.. *****************
.. Elbow Manipulator
.. *****************

.. .. figure:: ../figs/kinematics/elbow-manipulator.png
..     :width: 12cm
..     :figclass: align-center

..     Elbow manipulator with three revolute joints :cite:`spong2006robot`

.. Python Implementation
.. =====================

.. .. code-block:: python

..     import numpy as np
..     import math3d
    
..     def forward(q, d1, a2, a3):
..         '''
..         Forward kinematics of Elbow manipulator
..         '''

..         # Calculate homogenous DH transformations
..         T01 = math3d.DH(q[0] + np.pi/2, d1, 0, np.pi/2)
..         T12 = math3d.DH(q[1], 0, a2, 0)
..         T23 = math3d.DH(q[2], 0, a3, 0)

..         # Calculate homogenous transformation from {0} to {3}
..         T03 = T01.dot(T12).dot(T23)

..         # Return tool positio (x, y, z) relative to {0}
..         return T03[0:3,3]


***************
3D Math Library
***************
This library is found in the MAS507 repository in the current directory :code:`*/src/math3d.py`. The functions presented below will be handy when working with the kinematic modeling of the robot arm and to calculate the required rigid motions to pick the strawberries.

.. literalinclude:: ../../src/math3d.py
    :language: python
    :linenos:




.. bibliography:: ../refs.bib
    :style: unsrt
    :cited: