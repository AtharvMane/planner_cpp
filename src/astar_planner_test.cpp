#include "ros/ros.h"
#include "planner_cpp/astar.h"
#include <cstdlib>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>

class MapSubscriber{
    private:
    ros::Subscriber map_sub;
    ros::NodeHandle n;
    ros::ServiceClient client;
    
    int count;
    public:
    MapSubscriber(){
        this->map_sub=this->n.subscribe("/map",10,&MapSubscriber::callback,this);
        this->client = this->n.serviceClient<planner_cpp::astar>("astar_service");
        this->count=0;
    }
    void callback(nav_msgs::OccupancyGrid msg){
        if(!(this->count++))
            
        {nav_msgs::OccupancyGrid map=msg;
        geometry_msgs::Point start_pos=geometry_msgs::Point();
        geometry_msgs::Point goal_pos=geometry_msgs::Point();
        start_pos.x = 0.0;
        start_pos.y = 0.0;
        goal_pos.x = -0.0;
        goal_pos.y =5.0;
        planner_cpp::astar srv;
        srv.request.start_pos=start_pos;
        srv.request.goal_pos=goal_pos;
        srv.request.obstacle_map=map;
        std::cout<<"calling service"<<std::endl;
        if (client.call(srv))
        {
            ROS_INFO("Successfully found");
            
        }
        else
        {
            ROS_ERROR("Failed to call service add_two_ints");
        }}
        }
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "astar_service_client");

  MapSubscriber plan_call=MapSubscriber();
  ros::spin();
  
}