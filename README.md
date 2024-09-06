# Introduction to Robotics (DD2410)

## Lab 1: Introduction to ROS

### Description

Complete the controller.py script to help turtlebot Burger robot to explore the enviroment. Following code is
pseudo code.

```python
# Init stuff
while True:
  path, gain = get_path_from_action_server()
  if path is empty:
      exit() # Done
  while path is not empty:
    path, setpoint = get_updated_path_and_setpoint_from_service(path)
    setpoint_transformed = transform_setpoint_to_robot_frame(setpoint)
    publish(setpoint_transformed)
    sleep() 
```

### Presentation questions (prepared)

1. why do you need a rospy.init_node(...)?

Node is the basic unit of operation in ROS. By using the function, we tell the ROS master that the node is up and running. After the node is registered in the ROS master, it can interact with other nodes and perform its tasks.

2. What does pub = rospy.Publisher(...) and pub.publish(...) do?

The first function initializes a publisher object and the second function uses the publisher object to publish a message to a topic.

3. What is the queue_size parameter used for when you are calling rospy.Publisher(...)?

The messages need to be processed and then sent to the topic. If the messages are processed faster than they are sent, they will be stored in buffer before they are sent. The queue size parameter specifies the size of the buffer. The structure of the buffer is a queue, which is FIFO (First In First Out). If the buffer is full,
it will throw away the oldest message.

4. What is the purpose of the rospy.Rate(...) function?

The rospy.Rate(...) function is used to set the rate at which the loop runs. It is used to control the frequency of the loop.

5. What is the purpose of the rospy.sleep() function?

The rospy.sleep(...) function pauses the node for a time to ensure the rate is equal to which we set up by the rospy.Rate(...) function.