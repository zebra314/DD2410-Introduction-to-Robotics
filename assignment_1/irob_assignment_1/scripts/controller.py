#!/usr/bin/env python3
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, hypot

# Use to transform between frames
tf_buffer = None
listener = None

# The exploration simple action client
goal_client = None

# The collision avoidance service client
control_client = None

# The velocity command publisher
pub = None

# The robots frame
robot_frame_id = "base_link"

# Max linear velocity (m/s)
max_linear_velocity = 0.5

# Max angular velocity (rad/s)
max_angular_velocity = 1.0

def move(path):
    global control_client, robot_frame_id, pub

    # Call service client with path
    response = control_client(path)

    # Transform Setpoint from service client
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    transform = tf_buffer.lookup_transform(
        robot_frame_id,
        response.setpoint.header.frame_id,
        rospy.Time(0),
    )

    transformed_point = tf2_geometry_msgs.do_transform_point(
        response.setpoint,
        transform
    )

    # Create Twist message from the transformed Setpoint
    cmd_vel = Twist()
    cmd_vel.linear.x = transformed_point.x
    cmd_vel.angular.z = transformed_point.z

    # Publish Twist
    pub.publish(cmd_vel)

    # Call service client again if the returned path is not empty and do stuff again
    if len(response.path.poses) > 0:
        move(response.path)

    # Send 0 control Twist to stop robot
    else:
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
        pub.publish(cmd_vel)

    # Get new path from action server
    get_path()

def get_path():
    global goal_client

    # Init action client
    goal_client = actionlib.SimpleActionClient(
        "get_next_goal",
        irob_assignment_1.msg.GetNextGoalAction
    )
    goal_client.wait_for_server()

    # Call action server
    goal = irob_assignment_1.msg.GetNextGoalGoal()
    goal_client.send_goal(goal)
    goal_client.wait_for_result()

    # Get result from action server
    result = goal_client.get_result()

    # Call move with path from action server
    move(result.path)

if __name__ == "__main__":
    # Init node
    rospy.init_node('controller')

    # Init publisher
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 5)

    # Init simple action server
    action_server = actionlib.SimpleActionServer(
        "get_next_goal", 
        irob_assignment_1.msg.GetNextGoalAction, 
        auto_start=False
    )
    action_server.start()

    # Init service client
    rospy.wait_for_service('get_setpoint')
    control_client = rospy.ServiceProxy('get_setpoint', GetSetpoint)

    # Call get path
    get_path()

    # Spin
    rospy.spin()
