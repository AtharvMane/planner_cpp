#!/usr/bin/env python3
import rospy
from aruco_22.srv import *
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point



            


if __name__ == "__main__":     
    rospy.init_node("astar_test")
    rospy.wait_for_service('spiral_search')
    
    try:
        spiral_search_service = rospy.ServiceProxy('spiral_search', Spiral_search)
        response = spiral_search_service(True)
        if response.searched:
            print("spirally searching")
            print("searched", response.searched)
        else:
            print("spiral_search failed to search!!")
    except rospy.ServiceException as e:
        print("Service call failed:" + str(e))

    rospy.spin()