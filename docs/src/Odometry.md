# Odometry
## Puplishing
The navigation stack uses tf to determine the robot's location in the world and relate sensor data to a static map. However, tf does not provide any information about the velocity of the robot. Because of this, the navigation stack requires that any odometry source publish both a transform and a nav_msgs/Odometry message over ROS that contains velocity information. This tutorial explains the nav_msgs/Odometry message and provides example code for publishing both the message and transform over ROS and tf respectively.

## Odometry Message
## Using tf
## Writing the Code

