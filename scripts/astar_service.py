#!/usr/bin/env python3
import heapq
import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy
from math import sqrt
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from planner.srv import astar, astarResponse

show_animation = True
class Astar:
    def __init__(self, resolution, world_min_x, world_min_y, obstacle_map, x_width, y_width):
        # world_min_x, world_min_y are the minimum x and y co-ordinates of the world.
        # x_width and y_width are the dimensions of the map.
        self.resolution = resolution
        self.world_min_x, self.world_min_y = world_min_x, world_min_y
        self.obstacle_map = obstacle_map
        self.x_width, self.y_width = x_width, y_width

    class Node:
        def __init__(self, x, y, cost, parent_node, processed):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent = parent_node
            self.processed = processed
        
        @classmethod
        def from_tuple(cls, t):
            x, y = t
            return cls(x, y, float("inf"), None, False)

        def __str__(self):
            return (
                str(self.x)
                + ","
                + str(self.y)
                + ","
                + str(self.cost)
                + ","
                + str(self.parent)
            )

    def calc_grid_position(self, index, min_position):
        """
        Given an index, it calculates the position of the grid in the world coordinate frame

        index: this maybe the x or y coordinate
        min_position: pass the min_x or min_y accordingly
        return: actual position of the grid in the world frame
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        """
        calculates the index of the grid cell where the given position lies
        position: position in the world frame (again maybe x or y coordinate)
        min_pos: pass the min_x or min_y accordingly
        """
        return round((position - min_pos) / self.resolution)

    def return_cost(self, t1, t2):
        x1, y1 = t1
        x2, y2 = t2

        # Taking max since the grids with uncertainity are -1, so avoiding negative weights
        t = max(self.obstacle_map[int(y1)][int(x1)], 0) + max(self.obstacle_map[int(y2)][int(x2)], 0)
        if t >= 80:
            t = float("inf")
            return t
        else:
            return self.distance(t1, t2)

    def distance(self, current_node, node2):
        """
        This returns the euclidian distance between two nodes.
        """
        x, y = current_node
        x1, y1 = node2

        # euclidian distance
        return sqrt((x-x1)**2 + (y-y1)**2)

    def calc_heuristic(self, current_node, node2):
        """
        This is the heuristic function for A*.
        Diagonal Distance Heuristic is used.
        """
        x, y = current_node
        x1, y1 = node2

        dx = abs(x - x1)
        dy = abs(y - y1)

        h = (dx + dy) + (sqrt(2) - 2) * min(dx, dy)
        return h

    def return_neighbours(self, current_node):
        """
        Returns : List containing the neighbours of the current node (as tuples).
        """
        l = []
        x, y = current_node
        x_minus = x - 1
        y_minus = y - 1
        x_plus = x + 1
        y_plus = y + 1

        max_x = self.x_width
        min_x = 0
        min_y = 0
        max_y = self.y_width

        if x_plus < max_x:
            l.append((x_plus, y))

            if y_plus < max_y:
                l.append((x_plus, y_plus))

            if y_minus >= min_y:
                l.append((x_plus, y_minus))

        if x_minus >= min_x:
            l.append((x_minus, y))

            if y_plus < max_y:
                l.append((x_minus, y_plus))

            if y_minus >= min_y:
                l.append((x_minus, y_minus))

        if y_plus < max_y:
            l.append((x, y_plus))

        if y_minus >= min_y:
            l.append((x, y_minus))

        return l

    def plan(self, sx, sy, gx, gy):
        pq = []  # this is the list which we are going to use as a priority queue
        rx = []
        ry = []
        reached = {}
        # converting the positions into indices of the grid
        sx, sy = self.calc_xy_index(sx, self.world_min_x), self.calc_xy_index(sy, self.world_min_y)
        gx, gy = self.calc_xy_index(gx, self.world_min_x), self.calc_xy_index(gy, self.world_min_y)
        start = (sx, sy)
        goal = (gx, gy)

        heapq.heappush(
            pq, (0.0 + self.distance(start, goal), start)
        )  # similar to pq.push() in c++

        start_node = self.Node(sx, sy, 0.0, None, False)
        reached[start] = start_node
        iterations = 0  # to keep track of total number of iterations.

        while len(pq) > 0:
            iterations += 1
            d, t = heapq.heappop(pq)
            if (
                reached[t].processed == True
            ):  # if node is processed no need to iterate this node
                continue

            reached[t].processed = True
            # if show_animation:  # pragma: no cover
            #     plt.plot(
            #         self.calc_grid_position(reached[t].x, self.world_min_x),
            #         self.calc_grid_position(reached[t].y, self.world_min_y),
            #         "xc",
            #     )
            #     plt.pause(0.001)

            if (
                t == goal
            ):  # if goal is reached, backtrack all nodes from goal upto parent
                print("goal reached!")
                node = reached[goal]
                while not (node == None):
                    rx.append(self.calc_grid_position(node.x, self.world_min_x))
                    ry.append(self.calc_grid_position(node.y, self.world_min_y))
                    node = node.parent
                rx.reverse()
                ry.reverse()
                return True, rx, ry

            l = self.return_neighbours(t)
            for tup in l:
                if (
                    reached.get(tup) == None
                ):  # object does not exist, so creating an object here
                    n = self.Node.from_tuple(tup)
                    n.cost = reached[t].cost + self.return_cost(t, tup)
                    n.parent = reached[t]
                    reached[tup] = n
                    heapq.heappush(pq, (n.cost + self.calc_heuristic(tup, goal), tup))
                    
                elif (
                    reached[t].cost + self.return_cost(t, tup)< reached[tup].cost
                ):  # In this case object already exists, so just update the values
                    n = reached[tup]
                    n.cost = reached[t].cost + self.return_cost(t, tup)
                    n.parent = reached[t]
                    reached[tup] = n
                    heapq.heappush(pq, (n.cost + self.calc_heuristic(tup, goal), tup))

            if iterations > 5000:  # Just to ensure we don't go in an infinite loop
                print("goal not reached")
                return False, [], []




def main_planner(request):
    global show_animation
    print("in main planner")
    # initialize variables
    obstacle_map = OccupancyGrid()
    start_point = Point()
    goal_point = Point()

    # extract data from request
    obstacle_map = request.obstacle_map
    start_point = request.start_pos
    goal_point = request.goal_pos

    # extract map parameters
    map_width = obstacle_map.info.width
    map_height = obstacle_map.info.height
    map_data = obstacle_map.data
    map_resolution = obstacle_map.info.resolution
    # x and y coordinates of the cell (0,0) of map in the real world
    map_origin_x = obstacle_map.info.origin.position.x
    map_origin_y = obstacle_map.info.origin.position.y

    # set start and goal position
    sx = start_point.x
    sy = start_point.y
    gx = goal_point.x
    gy = goal_point.y

    ox, oy = [], []
    print(1)
    map_data = np.array(map_data, dtype=np.int32)

    map_data = map_data.reshape((map_height, map_width))
    temp = np.where(map_data==100)
    tempx = temp[0]
    tempy = temp[1]
    # l = [
    #     (i, j)
    #     for i in range(map_height)
    #     for j in range(map_width)
    #     if map_data[i, j] == 100
    # ]

    # for i in range(map_width):
    #     for j in range(map_height):
    #         if map_data[j][i] >= 50:
    #             oy.append((float(i) * map_resolution) + map_origin_x)#ox
    #             ox.append((float(j) * map_resolution) + map_origin_y)#oy
    #     print("still forring"+str(i))
    # if show_animation:
    #     plt.plot(oy, ox, ".k")#ox,oy
    #     plt.plot(sx, sy, "og")# sx,sy
    #     plt.plot(gx, gy, "xb")# gx,gy
    #     plt.grid(True)
    #     plt.axis("equal")
    #     plt.pause(0.001)
    '''
    The nxt part of the code is meant for easier path tracking. Each obstacle is enhanced to the 
    size of the bot_radius. This is done to ensure that if the bot travels on the path returned by the
    planner it does not collide with the obstacles.
    '''

    # radius of the bot in world frame.
    bot_radius = 0.225
    # radius of the bot in map frame. 
    map_bot_radius = int(round(bot_radius / map_resolution))
    #print(l)

    # #enhance obstacles 
    # #gox = 1.7
    # #goy = 0
    # #sx = -2
    # #sy = 0
    # #m = ((gy-sy)/(gx-sx))
    # #c = gy-m*gx
    # #c1 = c - (map_bot_radius)*((sqrt(1+m*m)))
    # #c2 = c + (map_bot_radius)*((sqrt(1+m*m)))
    # #print('start')
    # #print(l)

    print(2)
    for cx, cy in list(zip(tempx, tempy)): 
        #c0 = cy - m*cx
        #if  ((cx>-2) and (cx<1.7) and (y<0.75) and (cy>-0.75)):
        #    print('hi')
        map_data = cv2.circle(map_data, (cy, cx), map_bot_radius, 100, -1) 
    print('end')
    print(map_data)
    '''
    This part makes space around the goal empty for the planner. It is possible that the goal is very near
    to an obstacle it lies within bot_radius [m] of the obstacle. In this case A* wil fail to return a path 
    although one exists. Hence, the obstacles within bot_radius [m] of the goal are removed.
    '''
    print(3)
    map_data = cv2.circle(
        map_data, 
        (int(round((gy - map_origin_y) / map_resolution)), # map co-ordinate of goal_x
        int(round((gy - map_origin_y) / map_resolution))), # map co-ordinate of goal_y
        map_bot_radius,
        0, 
        -1
    )
    print("tryig a* now")
    # try:
    a_star = Astar(
        map_resolution, 
        map_origin_x, 
        map_origin_y,
        map_data, 
        map_width, 
        map_height
    )
    status, trajectory_x, trajectory_y = a_star.plan(sx, sy, gx, gy)
    print("a* done")
    # if show_animation and status:
    #     plt.plot(trajectory_x, trajectory_y, "-r")
    #     plt.show()
    # plt.plot(trajectory_x, trajectory_y, "-r")
    # plt.show()
    print("returning")
    return astarResponse(status, trajectory_x, trajectory_y)        
        
    # except Exception as e:
    #     trajectory_x, trajectory_y = [], []
    #     print("Planner failed to find a path!:", str(e))
    #     return astarResponse(False, trajectory_x, trajectory_y)
    
def astar_server():
    rospy.init_node('astar_server')
    s = rospy.Service('astar_planner', astar, main_planner)
    print("Ready to plan using A*.")
    rospy.spin()

if __name__ == "__main__":
    while not rospy.is_shutdown():
        astar_server()
    