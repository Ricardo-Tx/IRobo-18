#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray

# List of goals (x, y, z, orientation_w) that will be sent to the robot

# For Lab Map
goal_list = [
    (2.0, -0.5, 0.0, 1.0),  # First goal
    (2.0, 0.5, 0.0, 1.0),  # Second goal
    (-2.0, 0.5, 0.0, 1.0),  # Third goal
    (-2.0, -0.5, 0.0, 1.0)  # Fourth goal
]


# For Gazebo
# goal_list = [
#     (2.0, -0.5, 0.0, 1.0),  # First goal
#     (2.0, 0.5, 0.0, 1.0),  # Second goal
#     (-2.0, 0.5, 0.0, 1.0),  # Third goal
#     (-2.0, -0.5, 0.0, 1.0)  # Fourth goal
# ]

# Global index for the current goal
current_goal_index = 0

# Total number of goals
total_goals = len(goal_list)

# Callback function for the /move_base/status subscriber
def status_callback(status_msg):
    global current_goal_index

    # Check if there is at least one status in the list
    if status_msg.status_list:
        last_status = status_msg.status_list[-1]  # Get the last status

        # If the last goal succeeded (status code 3), send the next goal
        if last_status.status == 3:  # SUCCEEDED
            current_goal_index += 1

            if current_goal_index < total_goals:
                rospy.loginfo(f"Goal succeeded, sending next goal ({current_goal_index + 1}/{total_goals}).")
                send_new_goal(current_goal_index)
            else:
                rospy.loginfo("All goals have been sent and reached successfully.")

# Function to send a new goal to the /move_base/goal topic
def send_new_goal(goal_index):
    goal = MoveBaseGoal()

    # Define the goal position and orientation (coordinates from goal_list)
    x, y, z, w = goal_list[goal_index]
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = z
    goal.target_pose.pose.orientation.w = w

    # Send goal
    client.send_goal(goal)
    rospy.loginfo(f"New goal sent: x = {x}, y = {y} ({goal_index + 1}/{total_goals})")

if __name__ == "__main__":
    rospy.init_node('goal_publisher')

    # Create a SimpleActionClient for the move_base action server
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Wait for the action server to become available
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()

    rospy.loginfo("Connected to move_base action server")

    # Subscribe to the /move_base/status topic
    rospy.Subscriber('/move_base/status', GoalStatusArray, status_callback)

    # Send the first goal initially
    rospy.loginfo(f"Sending the first goal (1/{total_goals})")
    send_new_goal(current_goal_index)

    # Keep the node running
    rospy.spin()
