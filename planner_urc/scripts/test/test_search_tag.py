#! /usr/bin/env python3
import rospy
import sys
import os
import math
import numpy as np

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
        self.search_tag_client = actionlib.SimpleActionClient('search_tag', planner_urc.msg.SearchTagAction)
        self.search_tag_client.wait_for_server()

        print("Done initializing pubs/subs/servers!")

    def search_cb(self, fb_msg):
        print("Spiral Iterations till now: ", fb_msg.iterations)

    def main(self):
                              
        # search for the arrow
        self.search_goal = planner_urc.msg.SearchTagGoal(class_name="tag")
        self.search_tag_client.send_goal(self.search_goal, feedback_cb=self.search_cb)
        self.search_tag_client.wait_for_result()

        self.found = self.search_tag_client.get_result().found
        self.cx = self.search_tag_client.get_result().cx
        self.cy = self.search_tag_client.get_result().cy
        
        print(self.search_tag_client.get_result())
        if self.found == False:
            print("Failed to find the arrow.")
            return self.status
        else:
            print("Found AR Tag.")

if __name__ == "__main__":
    rospy.init_node("search_tag_client")

    root = Controller()
    root.main()

    rospy.spin()
    rospy.logwarn("Killing!")
