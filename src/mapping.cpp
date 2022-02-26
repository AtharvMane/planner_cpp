#include "ros/ros.h"
#include "ros/duration.h"
#include "tf/transform_datatypes.h"
#include "ros/time.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "matplotlib.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Int8.h"
#include "math.h"
#include <vector>

namespace plt=matplotlibcpp;

class Mapper{
  private:

  ros::Publisher map_pub;
  ros::Subscriber ls_sub;
  ros::Subscriber odom_sub;


  sensor_msgs::LaserScan ls;
  double x_glob=0.0,y_glob=0.0,theta=0.0;
  std::vector<float>ranges={};


  bool odom_clbk_bool=false;
  bool ls_clbk_bool=false;


  ros::Time ls_stamp_time;
  ros::Time odom_stamp_time;


  ros::Rate rate=ros::Rate(10);


  public:


  Mapper(ros::NodeHandle *n){

    this->ls_sub=n->subscribe("/scan",10,&Mapper::scan_callback,this);

    this->odom_sub=n->subscribe("/odom",10,&Mapper::odom_callback,this);

    this->map_pub=n->advertise<nav_msgs::OccupancyGrid>("map",10);


  }
  void scan_callback(sensor_msgs::LaserScan msg){
    this->ls_stamp_time=msg.header.stamp;
    this->ranges=msg.ranges;
    // this->restrict_ranges(&this->ranges,30,-30,0.7,8.0);
    this->ls_clbk_bool=true;
    
  }
  void restrict_ranges(std::vector<float> *ranges, int start_angle,int end_angle,float min_distance,float max_distance){
    for(int i=start_angle;i<360+end_angle;i++){
      (*ranges)[i]= std::numeric_limits<float>::infinity();
    }
    
  }
  void odom_callback(nav_msgs::Odometry msg){
    this->odom_stamp_time=msg.header.stamp;
    this->theta=tf::getYaw(msg.pose.pose.orientation);
    this->x_glob=-msg.pose.pose.position.y;
    this->y_glob=msg.pose.pose.position.x;
    this->odom_clbk_bool=true;
  }
  void map(){
    geometry_msgs::Pose map_origin=geometry_msgs::Pose();
          map_origin.position.x = -10.0;
          map_origin.position.y = -10.0;
          map_origin.position.z = 0.0;
          map_origin.orientation.x = 0.0;
          map_origin.orientation.y = 0.0;
          map_origin.orientation.z = 0.0;
          map_origin.orientation.w = 1.0;
    nav_msgs::OccupancyGrid obstacle_map= nav_msgs::OccupancyGrid();
      obstacle_map.info.width=4000;
      obstacle_map.info.height=4000;
      obstacle_map.header.frame_id = "map";
      obstacle_map.info.origin = map_origin;
      obstacle_map.info.resolution = 0.05;

      
      std::vector<int8_t> data(16000000,-1);
      
      double obs_x,obs_y,cur_x,cur_y,map_x,map_y;
      std::vector<double>ox;
      std::vector<double>oy;
      std::vector<double>bx;
      std::vector<double>by;
    while(!(ros::isShuttingDown()) ){
      ros::spinOnce();

      if((abs((this->odom_stamp_time-this->ls_stamp_time).toSec())>ros::Duration(0.025).toSec()) || !(this->ls_clbk_bool) ||!(this->odom_clbk_bool)){
        continue;
      }
      
      
      
      bx.push_back(x_glob);
      by.push_back(y_glob);

      for(int i=0;i<this->ranges.size();i++){
        if (this->ranges[i]==std::numeric_limits<float>::infinity()){
          continue;
        }
       obs_x = x_glob - this->ranges[i]*sin(theta + (i*M_PI/180.0));
        obs_y = y_glob + this->ranges[i]*cos(theta + (i*M_PI/180.0));
        cur_x = int((x_glob - obstacle_map.info.origin.position.x)*(1/obstacle_map.info.resolution));
        cur_y = int((y_glob - obstacle_map.info.origin.position.y)*(1/obstacle_map.info.resolution));         
        map_x = int((obs_x - obstacle_map.info.origin.position.x)*(1/obstacle_map.info.resolution));
        map_y = int((obs_y - obstacle_map.info.origin.position.y)*(1/obstacle_map.info.resolution));

      ox.push_back(obs_x);
      oy.push_back(obs_y);
      data[map_y*obstacle_map.info.width + map_x] = 100; 

                
}
      obstacle_map.data=data;
      plt::plot(ox, oy, ".k");
      plt::plot(bx,by,"og");
      plt::grid(true);
      plt::axis("equal");
      plt::pause(0.01);
      map_pub.publish(obstacle_map);

      this->rate.sleep();
      
      
    }

  }

}
;

int main(int argc, char **argv)
{ 
  ros::init(argc,argv,"mapper");

  ros::NodeHandle n;

  Mapper mapper=Mapper(&n);
  mapper.map();

  
  

  
  return 0;
}