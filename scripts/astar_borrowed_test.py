#!/usr/bin/env python
import rospy
from planner.srv import *
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point

class Subscriber():
    def __init__(self):
        self.count = 0
        self.sub = rospy.Subscriber('/map', OccupancyGrid, self.callback)        
    
    def callback(self, msg):
        if self.count == 0:
            # initialization
            obstacle_map = OccupancyGrid()
            start_pos = Point()
            goal_pos = Point()

            # assignment
            # here values have been hardcoded. we can change this in the main planner.
            obstacle_map = msg
            start_pos.x = -2.0
            start_pos.y = 0.0
            goal_pos.x = 2.0
            goal_pos.y = -0.5
            
            #service call
            rospy.wait_for_service('astar_borrowed_planner')
    
            try:
                astar_service = rospy.ServiceProxy('astar_borrowed_planner', astar_borrowed)
                response = astar_service(obstacle_map, start_pos, goal_pos)
                if response.ack:
                    print("A* planner successfully generated the path to goal!!")
                    print("Trajectory:", response.trajectory_x, response.trajectory_y)
                else:
                    print("A* planner failed to generate the path to goal!!")
            except rospy.ServiceException as e:
                print("Service call failed:" + str(e))

            self.count+=1
        else:
            pass


if __name__ == "__main__":     
    rospy.init_node("astar_borrowed_test")
    sub_map = Subscriber()
    rospy.spin()