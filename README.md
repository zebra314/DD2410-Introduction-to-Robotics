# Introduction to Robotics (DD2410)

## Lab 1: Introduction to ROS

### Presentation questions

why do you need a rospy.init_node(...)?
The node is the basic unit of operation in ROS. By using the function, we
tell the ROS master that the node is up and running.

What does pub = rospy.Publisher(...) and pub.publish(...) do?

What is the queue_size parameter used for when you are calling rospy.Publisher(...)?
