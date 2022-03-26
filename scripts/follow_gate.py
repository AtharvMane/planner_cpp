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
from planner.msg import BoundingBoxes
from tf import TransformListener


class Follow():
    # create messages that are used to publish feedback/result
    _feedback = planner.msg.FollowGateFeedback()
    _result = planner.msg.FollowGateResult()
    
    # depth info
    # depth_data = []

    # image
    image = np.zeros((480,640))

    # center of the image
    center = 0

    # bounding box params
    cx, cy = 0, 0

    # bounding box data
    bounding_boxes = []

    # threshold to stop rotating
    yaw_threshold = 80

    # distance from the object to stop (in meters)
    depth_threshold = 1.0

    # linear velocity and angular velocity
    linear_vel = 0.5  # 0.3 previosuly
    angular_vel = 0.3 # 0.3 previously

    area = 0

    area_threshold = 6000

    gate = True

    def __init__(self, name):
        self.bridge = cv_bridge.CvBridge()

        # initialize subscribers
        self.box_sub = rospy.Subscriber('/bb_aruco', BoundingBoxes, self.box_callback)
        self.image_sub = rospy.Subscriber('/camera1/usb_cam1/image_raw/', Image, self.image_callback)

        # initialize publishers
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # start the server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, planner.msg.FollowGateAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def box_callback(self, msg):
        self.bounding_boxes = msg.bounding_boxes
        self.center=self.bounding_boxes[0].x/2

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
        # print(self.image.shape)
        # cv2.imshow('window',img)
        # cv2.waitKey(1)

    def get_cx_cy(self):
        bb = self.bounding_boxes

        mx_area = 0
        cx = []
        cy = []

        if len(bb) == 2:
            for i in range(2):
                #take cx and cy from topic. Consider msg=[x_min,y_min,x_max,y_max]
                box = bb[i]
                xmin=box.xmin 
                xmax=box.xmax
                ymin=box.ymin    
                ymax=box.ymax
                
                cx.append((int)((xmin+xmax)/2))
                cy.append((int)((ymin+ymax)/2))

            self.cx = (int)((cx[0] + cx[1])/2)
            self.cy = (int)((cy[0] + cy[1])/2)
        
        if len(bb) == 0:
            self.gate = False

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

        self.pub_vel.publish(msg)
    
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
        print("forwarding")
        msg = Twist()
        msg.linear.x = self.linear_vel if forward else 0.0
        print(msg.linear.x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = proportion * self.angular_vel
        self.pub_vel.publish(msg)

    def execute_cb(self,goal):
        self.req_class_name = goal.class_name
        self.area = 0
        r = rospy.Rate(10)
        while self.gate and not rospy.is_shutdown():
            self.get_cx_cy()
            print("cx: "+str(self.cx)+"cy: "+str(self.cy))
            if(abs(self.cx - self.center) > 10):
                error = self.center - self.cx
                print("error: "+str(error))
                self.rotate_with_linear(True, float(error/200))
                r.sleep()
            
            else:
                self.move_forward()
                r.sleep()

            self.move_forward()
            r.sleep()

        rospy.sleep(2)
        
        if not self.gate:
            self.stop()
            self._result.reached = True
            self._as.set_succeeded(self._result, 'Passed the Gate.')

        else:
            self._result.reached = False
            self._as.set_succeeded(self._result, 'Not passed')

if __name__ == '__main__':
    rospy.init_node('follow_gate')
    server = Follow(rospy.get_name())
    rospy.spin()  
