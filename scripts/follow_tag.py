#! /usr/bin/env python3
import rospy
import cv2
import cv_bridge
import numpy as np
import planner.msg
import actionlib
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from planner.msg import BoundingBoxes,BoundingBox
from tf import TransformListener


class Follow():
    # create messages that are used to publish feedback/result
    _feedback = planner.msg.FollowTagFeedback()
    _result = planner.msg.FollowTagResult()
    
    # depth info
    # depth_data = []

    # image
    image = np.zeros((480,640))

    # center of the image
    center = 320 #320

    # bounding box params
    cx, cy, xmin, ymin, xmax, ymax = 0, 0, 0, 0, 0, 0

    # bounding box data
    bounding_boxes = []

    # threshold to stop rotating
    yaw_threshold = 80

    # distance from the object to stop (in meters)
    depth_threshold = 1.0

    # linear velocity and angular velocity
    linear_vel = 0.3  # 0.3 previosuly
    angular_vel = 0.1 # 0.15 previously

    area = 0

    area_threshold = 200 # 5000 previously

    def __init__(self, name):
        self.bridge = cv_bridge.CvBridge()

        # initialize subscribers
        self.box_sub = rospy.Subscriber('/bb_aruco', BoundingBoxes, self.box_callback)

        # initialize publishers
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #topic name for running of rover is : /rover

        # start the server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, planner.msg.FollowTagAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
    
        #first callback
        self.first = False
        while not self.first:
            pass 

        print('Done initializing server, pubs, subs')

    def box_callback(self, msg):
        self.bounding_boxes = msg.bounding_boxes
        self.first = True

    def get_cx_cy(self):
        bb = self.bounding_boxes

        mx_area = - float('inf')
        cx = 0
        cy = 0

        if len(bb)>0:
            for box in bb:
                #take cx and cy from topic. Consider msg=[x_min,y_min,x_max,y_max]
                xmin=box.xmin 
                xmax=box.xmax
                ymin=box.ymin    
                ymax=box.ymax

                area = abs((xmax-xmin)*(ymax-ymin))

                if area>mx_area:
                    self.xmin = xmin
                    self.xmax = xmax
                    self.ymin = ymin
                    self.ymax = ymax
                    self.cx = (int)((xmin+xmax)/2)
                    self.cy = (int)((ymin+ymax)/2)
                    mx_area = area
            self.area = mx_area
    
    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.pub_vel.publish(msg)
    
    def move_forward(self):
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        # self.pub_vel.publish(msg)
        return msg
    
#We don't actually use this block, we use rotate_with_linear instead

    def rotate(self, rotation):
        msg = Twist()
        # msg.linear.x = self.linear_vel
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        # msg.angular.z = (rotation/320) * self.angular_vel
        msg.angular.z = rotation* self.angular_vel

        self.pub_vel.publish(msg)

    def rotate_with_linear(self, forward, proportion):
        msg = Twist()
        msg.linear.x = self.linear_vel if forward else 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = proportion * self.angular_vel
        # self.pub_vel.publish(msg)
        return msg

    def execute_cb(self,goal):
        self.req_class_name = goal.class_name
        self.area = 0
        r = rospy.Rate(10)
        self.stop()

        

        while (self.area < self.area_threshold) and (not rospy.is_shutdown()):
            self.stop()
            self.get_cx_cy()
            print(self.area)
            # print(abs(self.cx - self.center))
            if(abs(self.cx - self.center) > 30): #Threshold to turn | Value Before : 40
                error = float(self.center - self.cx)
                msg = self.rotate_with_linear(True, float(error/320))
                #self.rotate_with_linear(True, float(error/320)) # Was 200
            
            else:
                msg = self.move_forward()
            #self.move_forward()
            self.pub_vel.publish(msg)   
            r.sleep()

            
        if self.area > self.area_threshold:
            self.stop()
            self._result.reached = True
            self._as.set_succeeded(self._result, 'Reached within 2m of the ar tag.')

            

        else:
            self.stop()
            print("error")
            self._result.reached = False
            self._as.set_succeeded(self._result, 'Not reached')

if __name__ == '__main__':
    rospy.init_node('follow_tag')
    server = Follow(rospy.get_name())
    rospy.spin()  
