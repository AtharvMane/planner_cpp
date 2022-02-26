#! /usr/bin/env python
import rospy
import actionlib
import planner.msg
from geometry_msgs.msg import Point

def move_fb(fb_data):
    print("Current Position: ", fb_data.current_pos)

def path_follow_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (InitialMoveAction) to the constructor.
    client = actionlib.SimpleActionClient('path_follow', planner.msg.PathFollowAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # values for the waypoints have been hardcoded here. This is to be changed in the final planner.
    rx = [1.0, 2.0, 3.0, 4.0, 5.0]
    ry = [1.0, 2.0, 3.0, 5.0, 5.0]

    # Creates a goal to send to the action server.
    goal = planner.msg.PathFollowGoal(trajectory_x = rx, trajectory_y = ry)

    # Sends the goal to the action server.
    client.send_goal(goal, feedback_cb = move_fb)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # ack

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the ActionClient can publish and subscribe over ROS.
        rospy.init_node('path_follow_client')
        result = path_follow_client()
        print("Result:", result.ack)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
