#!/usr/bin/env python3


# *tbd=to be done


# imports

import rospy #importing rospy cuz ROS
from rospy.rostime import Time, Duration                            # Required for syncronized Odometry and LaserScan message filtering
import tf2_ros                                                      # Required for RViz visualization (tbd*)
import tf2_msgs.msg                                                 # Same as above
import matplotlib.pyplot as plt                                     # Required for plot visualization of map (not the actual map but a copy of it)
from nav_msgs.msg import Odometry, OccupancyGrid                    # Odometry to get Odometry :) and Occupancy Grid to make map
from tf.transformations import euler_from_quaternion                # Needed for Orientation calculations
from geometry_msgs.msg import Point, Twist, PoseStamped, Pose       # Point required to store world stage info storage, PoseStamped for ?, Pose to get Position
from geometry_msgs.msg import TransformStamped, Quaternion          # TranformStamped for ?, Quaternion for ?
from sensor_msgs.msg import LaserScan                               # LaserScan for getting LaserScan messages either from lidar or from Transformed point/depthcloud
from math import atan2 , pi, cos, sin, radians, floor, isnan, isinf,sqrt,acos # some general terms in for some Meth ;)

# initializations

x = 0.0                 # current x-coordinate of the bot as seen in bot's co-ordinate frame (Read ReadMe_mapping.md)
y = 0.0                 # current y-coordinate of the bot as seen from bot's co-ordinate frame
x_glob = 0.0            # current x-coordinate of bot as seen in world frame 
y_glob = 0.0            # current x-coordinate of bot as seen in world frame
theta = 0.0             # orientation of the bot in radians
ranges = []             # current distances[] in laserScan message (if unfamiliar go read up on LaserScan message Docs: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html)
sub_ranges = []         # required for restricting the ranges (might not be relevant with newer tech so if new tech: Yay :D)
position = ()           # Point() -> world position
orientation = ()        # Quaternion -> world orientation
is_moving = False       # boolean variable that denotes whether the bot is currently moving or not, was required for discontinuous mapping, irrelevant as of now
show_animation = True #False  #if true, matplotlib viz is displayed, else no (for viz purposes, update to false on Jetson)

odom_timestamp=Time()          # timestamp for comparison purposes for synchronized message filtering
laserscan_timestamp=Time()     # timestamp for comparison purposes for synchronized message filtering

first_odom_callback=False      # initial callback bool (details in the end)
first_laserscan_callback=False # initial callback bool



# class Setup

class Subscriber():

    def __init__(self):

        # Setting up subscribers
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.callback_odom)         # Setting up odom SUbscriber 
        #self.sub_rover = rospy.Subscriber('/rover', Twist, self.callback_cmd_vel)      # For Rover (details in the end "IMU problem" section)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.callback_scan)        # Setting uo LaserScan subscriber
        
        
        
        # Some parameters for restrict ranges
        self.start_angle = -45
        self.end_angle =  45
        self.min_distance = 0.7
        self.max_distance = 4

        
    def callback_odom(self, msg):
        # getting the previously defined global variables (confused? here you go: https://www.programiz.com/python-programming/global-keyword)
        global x
        global y
        global theta
        global orientation
        global position
        global is_moving
        global odom_timestamp
        global first_odom_callback


        odom_timestamp=msg.header.stamp # getting the timestamp
        
        
        # for ZED:
        # message x coordinate = world y coordinate
        # message y coordinate = world x coordinate
        x = msg.pose.pose.position.x # getting the position of the bot in its own coordinate system
        y = msg.pose.pose.position.y # getting the position of the bot in its own coordinate system




        rot_q = msg.pose.pose.orientation # getting the orientation of the bot in its own coordinate system in Quaternions... ehh pls dont ask what it is, even most of us here dont know, just that its a convinient way for measuring angles in 3D, we not gonna use it anyways so LITE


        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w]) # Converting quaternions to more understadable x,y,z euler angles this is same as 12th Physics sytem of measuring rotatonal motion, a Right handed system 

        orientation = msg.pose.pose.orientation # unneded in my opinion
        position = msg.pose.pose.position   # this too



        # Again dis-continuous mapping stuff abhi ke liye lite lo :
        # vel_x = msg.twist.twist.linear.x 
        # vel_y = msg.twist.twist.linear.y
        # speed = vel_x**2 + vel_y**2
        # ang_z = msg.twist.twist.angular.z

        # if speed>0.001 or ang_z>0.001:
        #     is_moving = True
        # else:
        #     is_moving = False

            
        
        first_odom_callback=True # first callback bool

    
    
# Again dis-continuous mapping stuff abhi ke liye lite lo :    
    #def callback_cmd_vel(self,msg):
    #   vel_x = msg.linear.x
    #   vel_y = msg.linear.y
    #   speed = vel_x**2 + vel_y**2
    #   ang_z = msg.angular.z
    #   if speed>0 or ang_z>0:
    #       is_moving = True
    #   else:
    #       is_moving = False

    

    # Used  in restricting ranges (the first ZED we had gave inaccuracies beyond certain ranges, so we restricted them)
    # logic: make a list (restricted_ranges_li[]) of same length as ranges and initialize all its members to inf, then copy only required list members from ranges[]
    # to this list (restricted_ranges_li[])
    def restrict_ranges(self, start_angle, end_angle, min_distance, max_distance, ranges_li):

        restricted_ranges_li = [float('inf')] * len(ranges_li) # initializing restricted_ranges_li[] to all members infinity

        for i in range(start_angle, end_angle + 1): # a restriction on view span of the messages you
            if ranges_li[i] >= min_distance and ranges_li[i] <= max_distance: # condition to check and copy values restricted within min_distance and max_distance
                restricted_ranges_li[i] = ranges_li[i] # copying values
        return restricted_ranges_li

    
    
    
    
    
    
    
# the LaserScan callback
# This one is vv easy to understand so ye mai (the commentor) lite lunga    
    def callback_scan(self, msg):
        global laserscan_timestamp
        global ranges
        global sub_ranges
        global first_laserscan_callback

        sub_ranges = list(msg.ranges)
        laserscan_timestamp=msg.header.stamp
        ranges = self.restrict_ranges(self.start_angle, self.end_angle, self.min_distance,  self.max_distance, sub_ranges)
        first_laserscan_callback=True
        

if __name__ == "__main__":
    rospy.init_node("mapping_node")

    
    
    
    # intialize the pose of the origin of the map i.e. the world frame pose of the cell (0,0) in the map.
    pose = Pose()
    pose.position.x = -10.0
    pose.position.y = -10.0
    pose.position.z = 0.0
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0

    
    
    
    #intialize the map
    map = OccupancyGrid()

    map.header.frame_id = "map"
    map.info.resolution = 0.05  # (1/value) cells in 1m
    map.info.width = 4000
    map.info.height = 4000
    map.info.origin = pose      #the point (-10,-10,0) is the cell (0,0) of the map
    data = [-1]*map.info.width*map.info.height

    

    
    
    # instantiates class
    sub_pose = Subscriber()


    
    
    
    # initalize publisher to /map and /tf ? (idek what tf is) 
    pub = rospy.Publisher("/map", OccupancyGrid, queue_size = 10)
    tf_pub = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=10)

    
    
    ox, oy = [], []     #ox keeps a list of x-coordinates of the obstacles, oy keeps a list of the y-coordinates of the obstacles. (used for viz in matplotlib)
    
    
    sync_prob_num=0     # Don't be afraid its just a number to visualize how wasteful synchronized message filter is (This one isnt very expensive but the waste can be reduced, good readup topic if anyone interested, I am not :P)
    


    rate = rospy.Rate(10) # Defining rate to publish map at



    while not rospy.is_shutdown(): # i really dont know why they put it

        if not(first_laserscan_callback or first_odom_callback): # this his how we use first callback bools, currently since we are already inside a loop if statement is sufficient, but if not in a loop use while (condition): continue,
            continue                                             # Also putting them inside the constructor of class is better (no good reason as such, just that needed stuff is found at single place in th code)
        
        
        
        
        
        # Synchronized message filtering, basically filtering messages which were published at more or less the same time.
        # notice the use and comparison of timestamps with the instance of Duration() class
        # time stamp come as instances of Time, whose subtraction is transformed to an instance of Duration class, which can be compared only with another instance of duration class
        # read to understand: http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html
        # if abs(laserscan_timestamp-odom_timestamp)>Duration(0.025): 
        #     sync_prob_num+=1
        #     print("Sync probs"+str(sync_prob_num), end="\r")
        #     continue

        
        
        
        for i in range(len(ranges)):

            
            
            # # idek what this for tbh ?
            # map obstacles only when the current range is less than 'inf' i.e max range of lidar
            # if isnan(ranges[i]):
            #     ranges[i] = 0.3

            
            
            
            
            
            
            
            # yay some coordinate meth, check out ReadMe_mapping.md for further details
            
            # For Zed
            # x_glob=-y
            # y_glob=x       
            # obs_x = x_glob - ranges[i]*sin(theta + radians(i))   #world xcoordinate of obstacle
            # obs_y = y_glob + ranges[i]*cos(theta + radians(i))   #world ycoordinate of obstacle
            # cur_x = int((x_glob - map.info.origin.position.x)*(1/map.info.resolution))                          #map xcoordinate of robot
            # cur_y = int((y_glob - map.info.origin.position.y)*(1/map.info.resolution))                          #map ycoordinate of robot
            
            if not isinf(ranges[i]) and not isnan(ranges[i]) and not isinf(-ranges[i]) :
                if (ranges[i] == float('inf')) or (ranges[i] == -float('inf')):
                    continue
                x_glob=-y
                y_glob=x       
                obs_x = x_glob - ranges[i]*sin(theta + radians(i))   #world xcoordinate of obstacle
                obs_y = y_glob + ranges[i]*cos(theta + radians(i))   #world ycoordinate of obstacle
                cur_x = int((x_glob - map.info.origin.position.x)*(1/map.info.resolution)) # map xcoordinate of robot
                cur_y = int((y_glob - map.info.origin.position.y)*(1/map.info.resolution)) # map ycoordinate of robot                
                map_x = int((obs_x - map.info.origin.position.x)*(1/map.info.resolution))  # map xcoordinate of obstacle
                map_y = int((obs_y - map.info.origin.position.y)*(1/map.info.resolution))  # map ycoordinate of obstacle
                
                '''
                # for visualizing the map in map_frame co-ordinates
                ox.append(map_x)
                oy.append(map_y)
                '''
                # for visualizing the map in world_frame co-ordinates and also marking on matplotlib

                ox.append(obs_x)
                oy.append(obs_y)


                #marking the gridcels with obstacles on the actual Occupancy Grid
                data[int(map_y)*map.info.width + map_x] = 100 

                
                #mark (map_x,map_y) as an obstacle only if the range is not 'inf'

                
                # # Ahh some more experimentational BS XD
                
                # map_data = data.reshape((map.info.height, map.info.width))
                # temp = np.where(map_data==100)
                # tempx = temp[0]
                # tempy = temp[1]
                # bot_radius = 0.225
                # # radius of the bot in map frame. 
                # map_bot_radius = int(round(bot_radius / map_resolution))
                # #print(l)

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

                # print(2)
                # for cx, cy in list(zip(tempx, tempy)): 
                #     #c0 = cy - m*cx
                #     #if  ((cx>-2) and (cx<1.7) and (y<0.75) and (cy>-0.75)):
                #     #    print('hi')
                #     map_data = cv2.circle(map_data, (cy, cx), map_bot_radius, 100, -1)
                
                # data=map_data.reshape(1,map.info.height*map.info.width)
        
            
        '''
            THIS PART OF THE CODE IS FOR RVIZ VIZUALIZATION. NOT WORKING PROPERLY. FIX(TO-DO)
            #debug;-;
            print("map_x = " + str(map_x) + "\n" + "map_y = " + str(map_y) + "\n" + "---------------------------------\n" + "cur_x = " + str(cur_x) + "\n" + "cur_y = " + str(cur_y) + "\n" + "---------------------------------\n")
            
            #when the line joining bot and obstacle is straight, parallel to y-axis
            if map_x == cur_x:
                for j in range(min(cur_y,map_y),max(cur_y,map_y)):
                    obs_tmp_y = j
                    data[obs_tmp_y*map.info.width + cur_x] = 0
            
            #other cases
            else:
                slope = (map_y - cur_y)/(map_x - cur_x)
                #intercept = (map_x*cur_y - cur_x*map_y)/(map_x - cur_x)

                for j in range(min(cur_x,map_x),max(cur_x,map_x)):
                    obs_tmp_y = int(slope*(j-cur_x) + cur_y)
                    data[obs_tmp_y*map.info.width + j] = 0
            '''          

        
        
        # setting map.data nad publishing
        map.data = data
        pub.publish(map)

        
        '''
        THIS PART OF THE CODE IS FOR RVIZ VIZUALIZATION. NOT WORKING PROPERLY. FIX(TO-DO)
        #main bt part. Need to figure this out
        #map->odom tf is created by ROS when we run gmapping(live) or amcl on a mapped environment(bag file or saved map).
        #this map->odom tf needs to be manually created in this script. Need to figure what the map->odom link exactly does. 
        t = TransformStamped()
        t.header.frame_id = "map"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "odom"
        t.transform.translation = Point(0.0,0.0,0.0) #Point(cur_x, cur_y, 0.0) #
        t.transform.rotation = Quaternion(0.0,0.0,0.0,1.0)

        tfm = tf2_msgs.msg.TFMessage([t])
        tf_pub.publish(tfm)
        '''
        
        
        
        #vizualization in matplotlib
        if show_animation:   
            plt.plot(ox,oy,".k")
            plt.plot(x_glob,y_glob,"og")
            plt.grid(True)
            plt.axis("equal")
            plt.pause(0.01)
        
        
        
        rate.sleep()

#Run this command for RViZ visualization:
#rosrun robot_state_publisher robot_state_publisher 










# Extended explanations 




# Initial callbak bool:
# initial callback bool: On running the program, reception of first few required messages is delayed, which are set to default values when this happens. So its better to use these
# how they work: They turn to true only after the first callback, thus we can ensure that the required variables are initialized properly before use, by putting loops which cant be exited unless the initial callback variables become true 



# IMU problem:
# This one was mostly for non-continuous mapping (we stopped and mapped)
# The rover  currently (6 Jan 2022 11:27 pm) :) does not have an IMU so there is no way of getting velocities (other than differentiation position (which believe it you dont wanna do :))
# So a work-around was suggested to subscribe to the /rover topic (/cmd_vel but for rover) and check when we publish 0 velocity, assuming you havent stopped cuz of crashing XD.
# This is currently (7 Jan 2022 11.:36 am) :) irrelevant