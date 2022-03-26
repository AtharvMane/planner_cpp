#! /usr/bin/env python3
import rospy
import actionlib
import planner.msg
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import atan2, pi, sqrt
import matplotlib.pyplot as plt
import math
from math import atan2 , pi, cos, sin, radians, floor, isnan, isinf
class PathFollower(object):
    # message variables initialization
    ranges = [100.0]*360
    cur_x = 0.0
    cur_y = 0.0

    # create messages that are used to publish feedback/result
    _feedback = planner.msg.PathFollowFeedback()
    _result = planner.msg.PathFollowResult()

    # variables for calculating total distance travelled till now
    total_distance = 0.0
    previous_x = 0
    previous_y = 0
    first_run = True

    # velocity values
    linear_vel = 0.08
    angular_vel = 0.08

    # threshold values
    angle_err_threshold = 1*math.pi/180  # if difference between goal angle and current angle is less than this threshold we stop angular correction
    dist_err_threshold = 0.05 # if bot is within a radius of this threshold value goal has been reached.
    obstacle_dist_threshold = 2 #1.0 #if there is an obstacle within this threshold, stop the bot
    stopping_dist = 10.0  # after travelling stopping_dist m, the bot stops to update obstacles

    #initializing map
    map=OccupancyGrid()
    map_first_cb=False
    # Initializing the Subscriber(Odom), Publisher(cmd_vel), ActionServer(Initial_Move)
    def __init__(self, name):
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.clbk_odom)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.clbk_scan)
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.clbk_map)
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, planner.msg.PathFollowAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()


    def clbk_map(self,msg):
        self.map_first_cb=True
        self.map=msg


    def clbk_odom(self, msg):   
        if(self.first_run):
            #initialized this way to match co-ordinates in mapping
            self.previous_x = -msg.pose.pose.position.y
            self.previous_y = msg.pose.pose.position.x 
        #initialized this way to match co-ordinates in mapping
        self.cur_x = -msg.pose.pose.position.y
        self.cur_y = msg.pose.pose.position.x
        rot_q = msg.pose.pose.orientation
        (self.roll, self.pitch, theta1) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        self.theta=theta1+math.pi/2
        # calculate and update distance travelled till now
        d_increment = sqrt((self.cur_x - self.previous_x)**2 + (self.cur_y - self.previous_y)**2)
        self.total_distance = self.total_distance + d_increment        
        self.previous_x = self.cur_x
        self.previous_y = self.cur_y
        self.first_run = False

    def restrict_ranges(self, start_angle, end_angle, min_distance, max_distance, ranges_li):

        restricted_ranges_li = [float('inf')] * len(ranges_li)

        for i in range(start_angle, end_angle + 1):
            if ranges_li[i] >= min_distance and ranges_li[i] <= max_distance:
                restricted_ranges_li[i] = ranges_li[i]
        return restricted_ranges_li

    def clbk_scan(self, msg):
        sub_ranges = list(msg.ranges)
        # print(len(ranges))
        
        self.ranges = self.restrict_ranges(-45, 45, 0.7,  4, sub_ranges)
        

    def stop_bot(self):
        self.command.linear.x = 0.0
        self.command.linear.y = 0.0
        self.command.linear.z = 0.0
        self.command.angular.x = 0.0
        self.command.angular.y = 0.0
        self.command.angular.z = 0.0

    def dist_err(self):
        inc_x = self.goal_x - self.cur_x
        inc_y = self.goal_y - self.cur_y
        return sqrt(inc_x**2 + inc_y**2)
    
    def angle_to_turn(self):
        if self.current_goal_index+1>=len(self.trajectory_x):
            return 0
        angle_to_goal = atan2(self.trajectory_y[self.current_goal_index+1]-self.goal_y, self.trajectory_x[self.current_goal_index+1]-self.goal_x)
        angle=angle_to_goal-self.theta
        if abs(angle)<math.pi:
            return angle
        else:
            if angle<0:
                return 2*math.pi+angle
            else:
                return -2*math.pi+angle

    def execute_cb(self, goal):
        # initialize the goal co-ordinates
        print("in execute cb")
        self.trajectory_x = goal.trajectory_x
        self.trajectory_y = goal.trajectory_y
        # plt.plot(self.cur_x, self.cur_y, "og")
        # plt.plot(self.trajectory_x, self.trajectory_y, "-r")
        # plt.show()

        self.command = Twist()
        self.flag = True

        self.goal_x = self.trajectory_x[0]
        self.goal_y = self.trajectory_y[0]
        self.current_goal_index = 0

        while(self.flag):
            if not(self.map_first_cb):
                continue
            # print(self.trajectory_x)
            ox=[]
            oy=[]
            for i in range(len(self.ranges)):
                if not isinf(self.ranges[i]) and not isnan(self.ranges[i]) and not isinf(-self.ranges[i]) :
                    if (self.ranges[i] == float('inf')) or (self.ranges[i] == -float('inf')):
                        continue       
                    obs_x = self.cur_x + self.ranges[i]*cos(self.theta + radians(i))   #world xcoordinate of obstacle
                    obs_y = self.cur_y + self.ranges[i]*sin(self.theta + radians(i))   #world ycoordinate of obstacle  
                    map_x = int((obs_x +10)*(1/0.05))                      #map xcoordinate of obstacle
                    map_y = int((obs_y +10)*(20))                      #map ycoordinate of obstacle
                    ox.append(obs_x)
                    oy.append(obs_y)
            # calculations for adjusting bot position and velocity
            plt.clf()
            plt.plot(ox, oy, ".k")
            plt.plot(self.trajectory_x, self.trajectory_y, "-r")
            # # plt.plot(x, y, "og")
            plt.grid(True)
            plt.axis("equal")
            plt.pause(0.01)


            broken=False # status element for breaking out of multiple loops
            
            # calculate the nearest obstacle detected in the field of view of 20 degrees
            obj_dist = min(self.ranges[:])
            # if the nearest obstacle is within 2m stop the bot            
            # if obj_dist<self.obstacle_dist_threshold:
            #     for i in range(len(self.ranges)):
            #         if not isinf(self.ranges[i]) and not isnan(self.ranges[i]) and not isinf(-self.ranges[i]) :
            #             if (self.ranges[i] == float('inf')) or (self.ranges[i] == -float('inf')):
            #                 continue       
            #             obs_x = self.cur_x + self.ranges[i]*cos(self.theta + radians(i))   #world xcoordinate of obstacle
            #             obs_y = self.cur_y + self.ranges[i]*sin(self.theta + radians(i))   #world ycoordinate of obstacle  
            #             map_x = int((obs_x +10)*(1/0.05))                      #map xcoordinate of obstacle
            #             map_y = int((obs_y +10)*(20))                      #map ycoordinate of obstacle
            #             ox.append(obs_x)
            #             oy.append(obs_y)
            #     #Bharaths comments
            #     # stop bot
            #     # plan path again (using the updated mapping)
            #     # while(abs(angle_to_goal - self.theta) > self.angle_err_threshold):
            #     #  turn the bot
            #     for i in range(len(self.trajectory_x)):
            #         for j in range(len(ox)):
            #             if(int(self.trajectory_x[i]*20)==int(ox[j]*20) and int(self.trajectory_y[i]*20)==int(oy[j])*20):
            #                 self.stop_bot()
            #                 self.flag = False
            #                 self.pub_vel.publish(self.command)
            #                 print("stopped bot cuz obj detected")
            #                 # result is set to True                
            #                 self._result.ack = True
            #                 self._as.set_succeeded(self._result, 'Obstacle detected in front')
            #                 broken=True
            #                 break
            #         if(broken):
            #             break
            #         # print("Obstacle neglected")
            #     if(broken):
            #         break


            # # if bot has travelled more than 2m, stop the bot mapping.py to map obstacles
            # elif self.total_distance>self.stopping_dist:
            #     self.stop_bot()
            #     self.pub_vel.publish(self.command)
            #     self.total_distance = 0.0
            #     #delay of 2 secs for updation of obstacles
            #     rospy.sleep(2)
            
            # bot needs angle correction
            if abs(self.angle_to_turn()) > self.angle_err_threshold:
                self.command.linear.x = 0.0
                if self.angle_to_turn() > 0:
                   self.command.angular.z = self.angular_vel
                else:
                    self.command.angular.z = -self.angular_vel
                self.pub_vel.publish(self.command)
                print("turning for "+str(self.angle_to_turn()/self.angular_vel)+" seconds")
                rospy.sleep(abs(self.angle_to_turn()/self.angular_vel))

            
                

            # bot is aligned to the goal but needs to move forward 

            if abs(self.dist_err()) > self.dist_err_threshold:

                if obj_dist<self.obstacle_dist_threshold:
                    print("Nearer than thresh")
                    # for i in range(len(self.ranges)):
                    #     if not isinf(self.ranges[i]) and not isnan(self.ranges[i]) and not isinf(-self.ranges[i]) :
                    #         if (self.ranges[i] == float('inf')) or (self.ranges[i] == -float('inf')):
                    #             continue       
                    #         obs_x = self.cur_x + self.ranges[i]*cos(self.theta + radians(i))   #world xcoordinate of obstacle
                    #         obs_y = self.cur_y + self.ranges[i]*sin(self.theta + radians(i))   #world ycoordinate of obstacle  
                    #         map_x = int((obs_x +10)*(1/0.05))                      #map xcoordinate of obstacle
                    #         map_y = int((obs_y +10)*(20))                      #map ycoordinate of obstacle
                    #         ox.append(obs_x)
                    #         oy.append(obs_y)
                #Bharaths comments
                # stop bot
                # plan path again (using the updated mapping)
                # while(abs(angle_to_goal - self.theta) > self.angle_err_threshold):
                #  turn the bot
                    for i in range(len(self.trajectory_x)):
                            if(self.map.data[int((self.trajectory_y[i]+10)*20)*self.map.info.width+int((self.trajectory_x[i]+10)*20)]==100):
                                self.stop_bot()
                                self.flag = False
                                self.pub_vel.publish(self.command)
                                print("stopped bot cuz obj detected")
                                # result is set to True                
                                self._result.ack = True
                                self._as.set_succeeded(self._result, 'Obstacle detected in front')
                                broken=True
                                break
                        # print("Obstacle neglected")
                    if(broken):
                        break

                self.command.linear.x = self.linear_vel
                self.command.angular.z = 0.0
                self.pub_vel.publish(self.command)
                print(self.dist_err())
                rospy.sleep(self.dist_err()/self.linear_vel)

                


            # if bot has travelled more than 2m, stop the bot mapping.py to map obstacles
            elif self.total_distance>self.stopping_dist:
                self.stop_bot()
                self.pub_vel.publish(self.command)
                self.total_distance = 0.0
                #delay of 2 secs for updation of obstacles
                rospy.sleep(2)
            # bot has reached the current goal
            
                # update goal index
            self.current_goal_index = self.current_goal_index+1

            if self.current_goal_index >= len(self.trajectory_x):
                self.stop_bot()
                self.flag = False
                self.pub_vel.publish(self.command)
                self.pub_vel.publish(self.command)
                self.pub_vel.publish(self.command)
                self.pub_vel.publish(self.command)
                print("goal reached")
                # result is set to False                
                self._result.ack = False
                
                self._as.set_succeeded(self._result, 'Reached the Goal')
            else:
                self.goal_x = self.trajectory_x[self.current_goal_index]
                self.goal_y = self.trajectory_y[self.current_goal_index]      
        print("finished turning")
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