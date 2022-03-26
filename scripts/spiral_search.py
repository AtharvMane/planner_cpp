#!/usr/bin/env python3
import rospy
from planner.srv import Spiral_search,Spiral_searchResponse
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from planner.msg import BoundingBoxes
from tf.transformations import euler_from_quaternion
from math import sqrt,pi
class Spiral:
    position=Point()
    first_odom_clbk=False
    first_bb_clbk=False

    twist=Twist()
    searching=True
    count=0

    cx, cy, xmin, ymin, xmax, ymax, area,center = 0, 0, 0, 0, 0, 0, 0, 0
    angular_vel = 0.1 # 0.15 previously


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

        self.vel_pub.publish(msg)


    
    def get_cx_cy(self):
        bb = self.bb.bounding_boxes

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
    

    def see_flag(self):
        r= rospy.Rate(20)
        print("stop1")
        self.stop()
        while (not rospy.is_shutdown()):
            self.get_cx_cy()
            print(abs(self.cx - self.center))
            if(abs(self.cx - self.center) > 60): #Threshold to turn | Value Before : 40
                error = float(self.center - self.cx)
                msg = self.rotate(float(error/320))
            else:
                break
                #self.rotate_with_linear(True, float(error/320)) # Was 200
            
            r.sleep()
    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.vel_pub.publish(msg)


    def spiral_calculator(self,x,y):
        k=2/(pi)
        glob_ang_vel=pi/30
        r_square= (self.position.x-x)**2+(self.position.y-y)**2
        vel=sqrt(r_square*(glob_ang_vel**2)+(k**2)*(glob_ang_vel)**2)
        bot_ang_vel=glob_ang_vel+(k**2/(k**2+r_square))*glob_ang_vel
        return [vel,bot_ang_vel,sqrt(r_square)]

    def spiral_vel_pub(self,x,y):
        v,a,r=self.spiral_calculator(x,y)
        print(r)
        self.twist.linear.x=v
        self.twist.angular.z=a
        rate=rospy.Rate(10)
        print("am publishin")
        self.vel_pub.publish(self.twist)
        rate.sleep()
        
        
    def aruco_cb(self, msg):
    # initialized as opposites to match co-ordinates in mapping (get your meth right using ReadMe_mapping.md)
        self.bb=msg
        
        if len(self.bb.bounding_boxes)>0:
            self.center=self.bb.bounding_boxes[0].x/2

            self.searching=False

        else:
            self.searching=True
        print("msg: "+str(msg.bounding_boxes))
        self.count+=1
        self.first_bb_clbk=True      

    def __init__(self,searching):
        self.vel_pub=rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.pos_sub=rospy.Subscriber("/odom", Odometry,self.odom_clbk)
        self.aruco_sub=rospy.Subscriber("/bb_aruco", BoundingBoxes,self.aruco_cb,queue_size=10)
        self.bb=BoundingBoxes()
        self.searching=searching
        while not self.first_odom_clbk or not self.first_bb_clbk:
            print("sub not ready odom_sub: "+str(self.first_bb_clbk)+" bb_sub: "+str(self.first_bb_clbk),end="\r")
        print("\n")

    def main(self,x,y):
        while self.searching and not rospy.is_shutdown():
            self.spiral_vel_pub(x,y)
        if not self.searching:
            self.stop()
            self.see_flag()
            
        
    
    
    def odom_clbk(self,msg):
        self.position=msg.pose.pose.position
        self.first_odom_clbk=True

def spiral_search_func(req):
    x=req.x
    y=req.y
    searching=req.search
    spiral=Spiral(searching)
    spiral.main(x,y)
    

    print("returning") 
    return Spiral_searchResponse(spiral.searching)
def spiral_search_server():
    rospy.init_node('spiral_search_server')
    s = rospy.Service('spiral_search', Spiral_search, spiral_search_func)

    print("Ready to plan using A*.")


if __name__ == "__main__":
    while not rospy.is_shutdown():
        spiral_search_server()
        rospy.spin()