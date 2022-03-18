#! /usr/bin/env python3
import rospy
import sys
import os
import math
import numpy as np

from nav_msgs.msg import Odometry

import actionlib
import planner_urc.msg

class Controller():

    # current object of interest -> tag or gate
    object = 'tag'

    # status 
    # false -> some process failed
    # true -> was able to follow and reach the tags
    status = False

    # action vars
    found = False
    cx = 0
    cy = 0

    def __init__(self):
        
        # initialize actions
        self.follow_gate_client = actionlib.SimpleActionClient('follow_gate', planner_urc.msg.FollowGateAction)
        self.follow_gate_client.wait_for_server()

        print("Done initializing pubs/subs/servers!")

    def main(self):
        
        # follow the tag till within 2m
        self.follow_goal = planner_urc.msg.FollowGateGoal(cx = self.cx, cy = self.cy)
        self.follow_gate_client.send_goal(self.follow_goal)
        self.follow_gate_client.wait_for_result()

        print(self.follow_gate_client.get_result())
        if self.follow_gate_client.get_result().reached == False:
            print("Failed to reach tag")
            return self.status
        else:
            print("Tag within 2m")

if __name__ == "__main__":
    rospy.init_node("follow_gate_client")

    root = Controller()
    root.main()

    rospy.spin()
    rospy.logwarn("Killing!")
