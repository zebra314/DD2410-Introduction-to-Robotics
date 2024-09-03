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

    while path.poses:
        # Call service client with path to get setpoint and new path
        response = control_client(path)
        path = response.new_path

        # Transform Setpoint from service client
        transform = tf_buffer.lookup_transform(
            robot_frame_id,
            response.setpoint.header.frame_id,
            rospy.Time(),
        )

        transformed_point = tf2_geometry_msgs.do_transform_point(
            response.setpoint,
            transform
        )

        # Create Twist message from the transformed Setpoint
        cmd_vel = Twist()

        cmd_vel.linear.x = 0.5 * hypot(transformed_point.point.x, transformed_point.point.y)
        cmd_vel.linear.x = min(cmd_vel.linear.x, max_linear_velocity)

        cmd_vel.angular.z = 4 * atan2(transformed_point.point.y, transformed_point.point.x)
        cmd_vel.angular.z = min(cmd_vel.angular.z, max_angular_velocity)

        # If angular velocity is too high, set linear velocity to 0
        # prevents robot from moving forward while turning then stuck in the wall
        if(cmd_vel.angular.z / max_angular_velocity) > 0.5:
            cmd_vel.linear.x = 0

        # Publish Twist
        pub.publish(cmd_vel)
        rospy.sleep(0.1)

    # Send 0 control Twist to stop robot
    cmd_vel.linear.x = 0
    cmd_vel.angular.z = 0
    pub.publish(cmd_vel)

def get_path():
    global goal_client

    while not rospy.is_shutdown():
        # Call action server
        goal_client.wait_for_server()
        goal = irob_assignment_1.msg.GetNextGoalGoal()
        goal_client.send_goal(goal)
        goal_client.wait_for_result()

        # Get result from action server
        result = goal_client.get_result()

        # If path is empty, break
        if not result.path.poses:
            break

        # Call move with path from action server
        move(result.path)

if __name__ == "__main__":
    # Init node
    rospy.init_node('controller')

    # Init publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Init simple action client   
    goal_client = actionlib.SimpleActionClient(
        "get_next_goal",
        irob_assignment_1.msg.GetNextGoalAction
    )
    goal_client.wait_for_server()

    # Init service client
    rospy.wait_for_service('get_setpoint')
    control_client = rospy.ServiceProxy('get_setpoint', GetSetpoint)

    # Init tf2
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Call get path
    get_path()

    # Spin
    rospy.spin()
