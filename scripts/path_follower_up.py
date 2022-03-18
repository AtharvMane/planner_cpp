#! /usr/bin/env python3
import rospy                                        
import actionlib                            # for writing action clients
import planner.msg                          # generates messages for sending goal, receiving feedback, etc; these automatically gets created when you do catkin_make. refer http://wiki.ros.org/actionlib_tutorials/Tutorials
from nav_msgs.msg import Odometry, OccupancyGrid        # getting the odometry; map as occupancy grid 
from geometry_msgs.msg import Twist                     # to express linear and angular velocities; refer http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html 
from sensor_msgs.msg import LaserScan                   # to get the laserscan data; refer http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html
from tf.transformations import euler_from_quaternion    # to convert from quaternion form to euler form
from math import atan2, pi, sqrt                        # importing math fuctions
import matplotlib.pyplot as plt                         # to plot the path and obstacles in the OccupancyGrid
import math
from math import atan2 , pi, cos, sin, radians, floor, isnan, isinf
class PathFollower(object):
    ranges = [100.0]*360        # laserscan gives 360 points for turtlebot, hence the '360'. 100.0 here is an arbitary value here in place of 'inf'                                             
    cur_x = 0.0                 # current x position in map
    cur_y = 0.0                 # current y position in map 
    _feedback = planner.msg.PathFollowFeedback()            # creates messages to publish feedback
    _result = planner.msg.PathFollowResult()                # creates messages to publish result            


    total_distance = 0.0         # initialising distance
    previous_x = 0               
    previous_y = 0               
    first_run = True

    # assigning the velocities 
    linear_vel = 0.07             
    angular_vel = 0.1              

    # threshold values
    angle_err_threshold = 1*math.pi/180
    continuous_movement_angle_threshold=5*math.pi/180         # if difference between goal angle and current angle is less than this threshold, we stop angular correction
    dist_err_threshold = 0.05                   # if bot is within a radius of this threshold value, goal has been reached.
    obstacle_dist_threshold = 2                 # if bot is within this threshold, we can assume that there's an obstacle in front
    stopping_dist = 10.0                        # after travelling stopping_dist m, the bot stops to update obstacles in the OccuancyGrid

    #initializing map
    map=OccupancyGrid()
    map_first_cb=False                          # initializing the first map callback as 'False', since we don't have our first message(map) yet

    #initializations
    def __init__(self, name):  
        #subscribers and publisher initialize                 
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.clbk_odom)             
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.clbk_scan)
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.clbk_map)
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)                

        #action initialize
        self._action_name = name                  
        self._as = actionlib.SimpleActionServer(self._action_name, planner.msg.PathFollowAction, execute_cb=self.execute_cb, auto_start = False)    #creating our action server
        self._as.start() 
        self.command = Twist()                  # an instance of Twist() class to store linear and angular velocities that are published to the bot    
        self.kP = 0.3
        self.kI = 0.01
        self.kD = 0.01
        self.kP_r = 0.5
        self.kI_r = 0.0
        self.kD_r = 0.0  
        self.a_i = 0
        self.a_prev = 0   
        self.e_i = 0
        self.e_prev = 0   
        self.max_linear_speed = 0.2
        self.min_linear_speed = 0.001
        self.max_angular_speed = 1.0
        self.min_angular_speed = 0.001 
        self.linear_tolerance = 0.1
        self.angular_tolerance = 0.1               


    def clbk_map(self,msg):                                            
        self.map=msg                            # getting the map and storing and updating it in map variable
        self.map_first_cb=True                  # since we got our first message, callback bool changes back to 'True'


    def clbk_odom(self, msg):   

        #initialised as such to match the mapping coordinate system in gazebo (refer readme file for meth)
        if(self.first_run):
            #when the bot is running for the first time, we set the current x and y position to be the 'previous' position, since we dont have any previous position as of now
            self.previous_x = -msg.pose.pose.position.y     
            self.previous_y = msg.pose.pose.position.x 

        #updating the current x and y position
        self.cur_x = -msg.pose.pose.position.y              
        self.cur_y = msg.pose.pose.position.x
        rot_q = msg.pose.pose.orientation               # as the name suggests, its the orientation of the bot in x, y, z, w Quaternion form: refer http://docs.ros.org/en/diamondback/api/geometry_msgs/html/msg/Pose.html
        (self.roll, self.pitch, theta1) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])       # converting the Quarternion orientation to euler angles; idk why we do this conversion
        self.theta=theta1+math.pi/2                     # math from readme.md

        d_increment = sqrt((self.cur_x - self.previous_x)**2 + (self.cur_y - self.previous_y)**2)           # measuring the distance between current and previous point 
        self.total_distance = self.total_distance + d_increment                                             # our total distance variable is not storing the total distance continuosly, we gotta add the increase in distance      
        self.previous_x = self.cur_x                    # updating the previous x coordinate
        self.previous_y = self.cur_y                    # updating the previous y coordinate
        self.first_run = False                          # pretty self-explanatory


    # Used in restricting ranges (the earlier ZED we had was giving inaccurate data beyond a certain range, so we restricted them)
    # logic applied: make a list (restricted_ranges_li[]) of same length as ranges_li[] and initialize all its members as inf. We then copy only required list members from ranges[] to this list (restricted_ranges_li[])

    def restrict_ranges(self, start_angle, end_angle, min_distance, max_distance, ranges_li):

        restricted_ranges_li = [float('inf')] * len(ranges_li)                 # initializing all the values of restricted_ranges_li[] as inf

        for i in range(start_angle, end_angle + 1):                            # restricting angle range of the ZED/bot
            if ranges_li[i] >= min_distance and ranges_li[i] <= max_distance:  # restricting the values within min_distance and max_distance
                restricted_ranges_li[i] = ranges_li[i]                         # copying those restricted values to the restricted_ranges_li[] list
        return restricted_ranges_li

    def clbk_scan(self, msg):
        sub_ranges = list(msg.ranges)                                       
        self.ranges = self.restrict_ranges(-45, 45, 0.7,  4, sub_ranges)       # calling the func 
        
    #stopping the bot
    def stop_bot(self):
        self.command.linear.x = 0.0                                 
        self.command.linear.y = 0.0
        self.command.linear.z = 0.0
        self.command.angular.x = 0.0
        self.command.angular.y = 0.0
        self.command.angular.z = 0.0
        
    
    # calculating the distance error between current position and goal position
    def dist_err(self):
        inc_x = self.goal_x - self.cur_x
        inc_y = self.goal_y - self.cur_y
        return sqrt(inc_x**2 + inc_y**2)
    
    # Angle correction; basically calculating what angle the bot or ZED will have to turn to realign itself, so that it is facing the next goal point on the trajectory
    def angle_to_turn(self):
        # if the current goal point is the last goal point of the trajectory, we dont need to the correction
        if self.current_goal_index>=len(self.trajectory_x)-1:                       
            return 0


        # if the distance error is greater than the threshold value and the bot has not reached the goal point 
        if abs(self.dist_err())>self.dist_err_threshold:                         
            angle_to_goal = atan2(self.goal_y-self.cur_y, self.goal_x-self.cur_x)   # tan of (increment of y coordinate/increment of x coordinate): refer the dist_err(self); basic meth


        # when the bot has reached a goal point in trajectory and gotta move to the next one. inc_y = next goal point_y - current goal point_y; similarly for x. we'll then do tan(inc_y/inc_x)
        else:
            angle_to_goal = atan2(self.trajectory_y[self.current_goal_index+1]-self.goal_y, self.trajectory_x[self.current_goal_index+1]-self.goal_x)
             
        angle=angle_to_goal-self.theta                      # angle to rotate = angle_to_goal - current angle orientation of bot
        
        # basic 12th grade meth
        if abs(angle)<math.pi:                              
            return angle
        else:
            if angle<0:
                return 2*math.pi+angle              # refer path_follower_ReadMe.md for meth
            else:
                return -2*math.pi+angle

    def hard_turn(self):
        while abs(self.angle_to_turn()) > self.angle_err_threshold:
            self.command.linear.x = 0.0
            self.turn_determine()
            self.pub_vel.publish(self.command)
                                          #publishing the velocities
                #print("turning till 0: "+str(self.angle_to_turn()*180/math.pi), end="\r")       #will constantly print
    def turn_determine(self):
        self.a = self.angle_to_turn()
        print("angle"+str(self.a*180/math.pi),end="\r")
        self.a_i += self.a
        self.a_d = self.a - self.a_prev
        self.command.angular.z = (self.kP_r * self.a + self.kI_r * self.a_i + self.kD_r * self.a_d)
        self.a_prev = self.a
        if abs(self.command.angular.z) > self.max_angular_speed:
            self.command.angular.z = self.max_angular_speed*self.command.angular.z/abs(self.command.angular.z)
        elif abs(self.command.angular.z) > 0 and abs(self.command.angular.z) < self.min_angular_speed:
            self.command.angular.z = self.min_angular_speed*self.command.angular.z/abs(self.command.angular.z)
        



    def execute_cb(self, goal):
        print("in execute cb")
        # initalizing the goal co-ordinates
        self.trajectory_x = goal.trajectory_x
        self.trajectory_y = goal.trajectory_y
        
        self.flag = True

        self.goal_x = self.trajectory_x[0]                  #the trajectory_x list was reversed in the code; so the first element of trajectory_x[] means the final point i.e. the goal point
        self.goal_y = self.trajectory_y[0]
        self.current_goal_index = 0
        broken=False                                                    # status element for breaking out of multiple loop

        while(self.flag):
            #condition to check whether we have recieved the first map
            if not(self.map_first_cb):                      # btw since we are already inside a loop, 'if' statement is sufficient for using first callbacks, but if not in a loop, use while (condition): continue,
                continue
            
            #ox keeps a list of x-coordinates of the obstacles, oy keeps a list of the y-coordinates of the obstacles
            ox=[]                                           
            oy=[]
            for i in range(len(self.ranges)):
                if not isinf(self.ranges[i]) and not isnan(self.ranges[i]) and not isinf(-self.ranges[i]) :
                    if (self.ranges[i] == float('inf')) or (self.ranges[i] == -float('inf')):
                        continue       
                    obs_x = self.cur_x + self.ranges[i]*cos(self.theta + radians(i))        #world xcoordinate of obstacle
                    obs_y = self.cur_y + self.ranges[i]*sin(self.theta + radians(i))        #world ycoordinate of obstacle
                    map_x = int((obs_x +10)*(1/0.05))                                       #map xcoordinate of obstacle
                    map_y = int((obs_y +10)*(20))                                           #dervied from: int((obs_y - map.info.origin.position.y)*(1/map.info.resolution))
                    
                    # for visualizing the map in world_frame co-ordinates and also marking the obstacles on map
                    ox.append(obs_x)                                                        #adding all x obstacle points in one list
                    oy.append(obs_y)                                                        

            # plotting on matplotlib
            plt.clf()
            plt.plot(ox, oy, ".k")
            plt.plot(self.trajectory_x, self.trajectory_y, "-r")

            plt.grid(True)
            plt.axis("equal")
            plt.pause(0.01)

            obj_dist = min(self.ranges[:])                                  #calculating the nearest obstacle detected in the field of view of the range set(20 degree here)

            #angle correction
            print("hard_turning: "+str(self.angle_to_turn()))
            if abs(self.angle_to_turn())>self.continuous_movement_angle_threshold:
                
                self.hard_turn()

            print("\n")

            #now that the bot is aligned, it needs to move forward
            if abs(self.dist_err()) > self.dist_err_threshold:
                
                if obj_dist<self.obstacle_dist_threshold:
                    for i in range(len(self.trajectory_x)):
                        
                        for x_buff in range(-2,2):                      #for creating buffer gridcells in x direction
                            for y_buff in range(-2,2):                  #for creating buffer gridcells in y direction 
                                
                                '''

                                we are putting a condition to check if the trajectory we have rn, has aany obstacle in its path.
                                To check that, we are comparing the trajectory in map coordinates to 100 (100 denotes that the gridcell has an obstacles )
                                
                                data[(int(map_y) + y_buff )*map.info.width + map_x] == 100
                                here, map_y = (trajectory[i] - map.info.origin.position.y)*(1/map.info.resolution) and similarly for map_x


                                '''

                                if(self.map.data[(int((self.trajectory_y[i]+10)*20)+y_buff)*self.map.info.width+(int((self.trajectory_x[i]+10)*20)+x_buff)]==100):
                                    self.stop_bot()                                     #obstacle detected in the trajectory, stop the bot
                                    self.flag = False                                   
                                    self.pub_vel.publish(self.command)                  #publishing the velocities
                                    print("stopped bot cuz obj detected")
                                    # result is set to True                
                                    self._result.ack = True                             #ack = True, since obstacle detected
                                    self._as.set_succeeded(self._result, 'Obstacle detected in front')      #result
                                    broken=True                                         # status element for breaking out of multiple loop
                                    break
                            if(broken):
                                break
                        if(broken):
                            break
                            # print("Obstacle neglected")
                    if(broken):
                        break
                
                #assigning velocities                                                                  
                self.command.angular.z = 0.0
                if self.angle_to_turn()>self.angle_err_threshold and self.angle_to_turn()<self.continuous_movement_angle_threshold:
                    self.turn_determine()
                self.e = self.dist_err()
                print("e: {}".format(self.e))
                self.e_i += self.e
                self.e_d = self.e - self.e_prev
                self.command.linear.x = (self.kP * self.e + self.kI * self.e_i + self.kD * self.e_d)

                if abs(self.command.linear.x) > self.max_linear_speed:
                    self.command.linear.x = self.max_linear_speed
                elif abs(self.command.linear.x) > 0 and abs(self.command.linear.x) < self.min_linear_speed:
                    self.command.linear.x = self.min_linear_speed
                self.e_prev = self.e
                self.pub_vel.publish(self.command)

                rospy.sleep(0.1)
                
            # if self.total_distance>self.stopping_dist:
            #     self.stop_bot()
            #     self.pub_vel.publish(self.command)
            #     self.total_distance= 0.0
            #     rospy.sleep(2)
            if(self.dist_err()<self.dist_err_threshold):
                self.current_goal_index = self.current_goal_index+1                         #updating the next trajectory goal point
                print("updated traj_goalpoint")

            if self.current_goal_index >= len(self.trajectory_x):                       #if current goal index is the last goal index, we'll stop the bot since goal reached
                self.stop_bot()
                self.flag = False   
                self.pub_vel.publish(self.command)                                      #publishing velocities
                self.pub_vel.publish(self.command)
                self.pub_vel.publish(self.command)
                self.pub_vel.publish(self.command)
                print("goal reached")
                # result is set to False                
                self._result.ack = False
                
                self._as.set_succeeded(self._result, 'Reached the Goal')
            else:
                self.goal_x = self.trajectory_x[self.current_goal_index]                #updating the trajectory goal
                self.goal_y = self.trajectory_y[self.current_goal_index]      

        if self.flag:
            # publish the feedback   
            self._feedback.current_pos.x = self.cur_x
            self._feedback.current_pos.y = self.cur_y            
            self._as.publish_feedback(self._feedback)

        '''
        # this step is not necessary, the sequence is computed at 1 Hz. But this can be useful later.
        r = rospy.Rate(1)
        r.sleep()

        '''
        
if __name__ == '__main__':
    rospy.init_node('path_follow')
    server = PathFollower(rospy.get_name())
    rospy.spin()

#Use rospy.loginfo() in the future for debugging
