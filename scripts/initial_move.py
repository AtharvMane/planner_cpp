#! /usr/bin/env python3
import rospy
import actionlib
import planner.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import atan2, pi, sqrt

class InitialMover(object):
    # create messages that are used to publish feedback/result
    _feedback = planner.msg.InitialMoveFeedback()
    _result = planner.msg.InitialMoveResult()

    # variables for calculating total distance travelled till now
    total_distance = 0.0
    previous_x = 0
    previous_y = 0
    first_run = True

    # velocity values
    linear_vel = 0.5
    angular_vel = 0.3

    # threshold values
    angle_err_threshold = 0.05  # if difference between goal angle and current angle is less than this threshold we stop angular correction
    dist_err_threshold = 0.3 # if bot is within a radius of this threshold value goal has been reached.
    obstacle_dist_threshold = 1.0 #if there is an obstacle within this threshold, stop the bot
    stopping_dist =2.0  # after travelling stopping_dist m, the bot stops to update obstacles

    # Initializing the Subscriber(Odom), Publisher(cmd_vel), ActionServer(Initial_Move)
    def __init__(self, name):
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.clbk_odom)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.clbk_scan)
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, planner.msg.InitialMoveAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
    
    def clbk_odom(self, msg):   
        if(self.first_run):
            self.previous_x = msg.pose.pose.position.x
            self.previous_y = msg.pose.pose.position.y 

        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        (self.roll, self.pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

        # calculate and update distance travelled till now
        d_increment = sqrt((self.cur_x - self.previous_x)**2 + (self.cur_y - self.previous_y)**2)
        self.total_distance = self.total_distance + d_increment        
        self.previous_x = self.cur_x
        self.previous_y = self.cur_y
        self.first_run = False

    def clbk_scan(self, msg):
        self.ranges = msg.ranges

    def stop_bot(self):
        self.command.linear.x = 0.0
        self.command.linear.y = 0.0
        self.command.linear.z = 0.0
        self.command.angular.x = 0.0
        self.command.angular.y = 0.0
        self.command.angular.z = 0.0

    def execute_cb(self, goal):
        # initialize the goal co-ordinates
        self.goal_x = goal.goal.x
        self.goal_y = goal.goal.y

        self.command = Twist()
        self.flag = True
        while(self.flag):
            # calculations for adjusting bot position and velocity
            inc_x = self.goal_x - self.cur_x
            inc_y = self.goal_y - self.cur_y
            angle_to_goal = atan2(inc_y, inc_x)
            dist_err = sqrt(inc_x**2 + inc_y**2)

            # calculate the nearest obstacle detected in the field of view of 20 degrees
            obj_dist = min(self.ranges[:10] + self.ranges[-10:])

            # if the nearest obstacle is within 3m stop the bot            
            if obj_dist<self.obstacle_dist_threshold:
                self.stop_bot()
                self.flag = False
                self.pub_vel.publish(self.command)
                # result is set to True                
                self._result.ack = True
                self._as.set_succeeded(self._result, 'Obstacle detected in front')         

            # if bot has travelled more than 2m, stop the bot mapping.py to map obstacles
            elif self.total_distance>self.stopping_dist:
                self.stop_bot()
                self.pub_vel.publish(self.command)
                self.total_distance = 0.0
                #delay of 2 secs for updation of obstacles
                rospy.sleep(2)

            # bot needs angle correction
            elif abs(angle_to_goal - self.theta) > self.angle_err_threshold:
                self.command.linear.x = 0.0
                if (angle_to_goal - self.theta) > 0:
                   self.command.angular.z = self.angular_vel
                else:
                    self.command.angular.z = -self.angular_vel
                self.pub_vel.publish(self.command)

            # bot is aligned to the goal but needs to move forward 
            elif dist_err > self.dist_err_threshold:
                self.command.linear.x = self.linear_vel
                self.command.angular.z = 0.0
                self.pub_vel.publish(self.command)

            # bot has reached the goal
            else:
                self.stop_bot()
                self.flag = False
                self.pub_vel.publish(self.command)
                # result is set to False                
                self._result.ack = False
                self._as.set_succeeded(self._result, 'Obstacle detected in front')      

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
    rospy.init_node('initial_move')
    server = InitialMover(rospy.get_name())
    rospy.spin()

#Use rospy.loginfo() in the future for debugging