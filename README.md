# Introduction to Robotics (DD2410)

## Catalog

1. [Lab 1: Introduction to ROS](#lab-1-introduction-to-ros)
2. [Lab 2: Inverse Kinematics](#lab-2-inverse-kinematics)
3. [Lab 3: Path Planning](#lab-3-path-planning)
4. [Lab 4: Mapping](#lab-4-mapping)

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

### Usage

```bash
make assignment1
. ./devel/setup.zsh
roslaunch irob_assignment_1 demo.launch
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

6. What is the differnce between a subscriber, service and action server?

A subscriber is a node that listens to a topic and receives messages from it. A service is a node that listens to a service request and sends a response. An action server is a node that listens to an action request and sends a response. The difference between a service and an action server is that an action server can send feedback messages to the client.

## Lab 2: Inverse Kinematics

### Description

In this assignment you will program the inverse kinematic (IK) algorithm for a robot, which will move its joints so that it follows a desired path with the end-effector. 

It is composed of two parts:

A 3 DOF SCARA robot, with an inverse kinematic solution that is possible in analytic form. Solving this part is the minimum requirement to pass the assignment, and get an E grade. Kattis will give you a score of 20 or above if this part is solved.

A 7 DOF KUKA robot, with the inverse kinematic solution to be implemented with iterative algorithms. Solving this part is required for a C grade (the best grade possible for this assignment). Kattis will give you a score of 22 if both parts are solved.

The python file:

[kinematics_assignment/scripts/IK_functions.py](./assignment_2_inverse_kinematic/kinematics_assignment/scripts/IK_functions.py)

### Usage

```bash
make assignment2
. ./devel/setup.zsh
roslaunch kinematics_assignment kuka.launch # or scara.launch
```

### Refernces

1. [Video - Computing the Robot Jacobian of Serial Manipulators | Robotic Systems (OLD)](https://www.youtube.com/watch?v=V1TXcU1r-ns)

2. [Passage - The Ultimate Guide to Jacobian Matrices for Robotics](https://automaticaddison.com/the-ultimate-guide-to-jacobian-matrices-for-robotics/)

3. [Passage - Overview of Jacobian IK](https://medium.com/unity3danimation/overview-of-jacobian-ik-a33939639ab2)

4. [Book - Robotics Modelling, Planning and Control](https://link.springer.com/book/10.1007/978-1-84628-642-1) CH3: Differential Kinematics and Statics

## Lab 3: Path Planning

### Description

In [solution.py](./assignment_3_planning/solution.py), you're tasked to implement a robotic planning method in order to drive a Dubins car from an initial position (x0,y0) to a target position (xt, yt), while avoiding both collisions with obstacles and venturing out of bounds.

The state variables are:

- x: horizontal position
- y: vertical position
- theta: heading angle (direction of travel)

And, the sole control variable is the steering angle phi ∈ [-pi/4, pi/4] (with respect to the direction of travel).

### Usage

```bash
make assignment3
python3 main.py -p
```

### A star algorithm

Implement A* algorithm.

```python
:weight: g() + h()
  - g(): the distance which have walked
  - h(): straight line distance between that node and end node
  
  - End the loop when the queue is empty or the end node is found.
  - In the loop:
    - Get the node with the smallest weight(g + h) from the queue.
    - If the node is the end node, end the loop.
    - Get the neighbors of the node.
    - For each neighbor, Calculate the weight from the start node to the neighbor.
    - If the neighbor is not visited or the weight(g) is smaller than the previous one,
    - Add the neighbor to the visited set.
    - Record the parent node and the distance in the parent dictionary.
    - Add the neighbor to the queue.
```

### References

1. [Video - A* Pathfinding (E01: algorithm explanation)](https://www.youtube.com/watch?v=-L-WgKMFuhE&t=285s)

2. [Github - PythonRobotics/PathPlanning/AStar/a_star.py](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/AStar/a_star.py)

3. [Instruction - Dubin's Car](./assignment_3_planning/README.md)

## Lab 4: Mapping

### Description

### Usage

```bash
make assignment4
. ./devel/setup.zsh
```

### References
