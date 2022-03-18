#! /usr/bin/env python3
import rospy
import sys
import os
import math
import numpy as np
from std_msgs.msg import String

from nav_msgs.msg import Odometry

import actionlib
import planner_urc.msg

class Controller():
    """
    Main controller class
    """

    # number of tags detected till now
    tag_cnt = 0


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

        self.follow_tag_client = actionlib.SimpleActionClient('follow_tag', planner_urc.msg.FollowTagAction)
        self.follow_tag_client.wait_for_server()

        # self.follow_gate_client = actionlib.SimpleActionClient('follow_gate', planner_urc.msg.FollowGateAction)
        # self.follow_gate_client.wait_for_server()
        r = rospy.Rate(10)
        self.pub_led = rospy.Publisher('/led', String, queue_size = 10)
        for i in range(30):
            msg = String()
            msg.data = 'red'
            self.pub_led.publish(msg)
            r.sleep()
        print("Done initializing pubs/subs/servers!")

    def search_cb(self, fb_msg):
        print("Spiral Iterations till now: ", fb_msg.iterations)

    def main(self):
        # run the code until rover reaches the cone
        while self.tag_cnt<1:
                      
            # search for the arrow
            self.search_goal = planner_urc.msg.SearchTagGoal(class_name="tag")
            self.search_tag_client.send_goal(self.search_goal, feedback_cb=self.search_cb)
            self.search_tag_client.wait_for_result()

            self.found = self.search_tag_client.get_result().found
            self.cx = self.search_tag_client.get_result().cx
            self.cy = self.search_tag_client.get_result().cy

            if self.found == False:
                print("Failed to find the arrow.")
                return self.status
            else:
                print("Found AR Tag.")
                        
            # follow the tag till within 2m
            self.follow_goal = planner_urc.msg.FollowTagGoal(cx = self.cx, cy = self.cy)
            self.follow_tag_client.send_goal(self.follow_goal)
            self.follow_tag_client.wait_for_result()

            if self.follow_tag_client.get_result().reached == False:
                print("Failed to reach tag within 2m")
                return self.status
            else:
                print("tag within 2m")
                self.tag_cnt+=1

        print("testing over, go home")
        self.status = True
        return self.status

if __name__ == "__main__":
    rospy.init_node("controller_client")

    root = Controller()
    result = root.main()

    rospy.spin()
    rospy.logwarn("Killing!")
