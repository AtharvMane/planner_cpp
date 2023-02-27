#include "ros/ros.h"
#include <bits/stdc++.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
using namespace std;
class Converter{
    private:
    ros::Subscriber grid_sub;
    grid_map_msgs::GridMap grid_msg;
    grid_map::GridMap grid;
    public:
    Converter(ros::NodeHandle *nh){
        grid_sub = nh->subscribe("/clouder",10, &Converter::grid_sub_cb, this);
    }

    void grid_sub_cb(grid_map_msgs::GridMap::ConstPtr msg){
        grid_msg = *msg;
	std::cout<<"subbed"<<std::endl;
	convert();
	show();
    }
    void convert(){
        grid_map::GridMapRosConverter conv_class;
        conv_class.fromMessage(grid_msg, grid);
    }
    void show(){
        grid_map::Index idx(0,0);
        grid_map::Position coordinate;
	grid.getPosition(idx,coordinate);
        cout<<coordinate<<endl;
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "play");
    ros::NodeHandle n;
    cout<<"Playing\n";
    ros::AsyncSpinner spinner(0);
    Converter con(&n);
    con.convert();
    con.show();
    spinner.start();
    // ros::spin();
    return 0;
}
