#ifndef ASTAR_NODELET_H
#define ASTAR_NODELET_H
#include <nodelet/nodelet.h>
#include <ros/ros.h>

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/point_cloud.h>

// #include <pointcloud_cuda_mapping/pointcloud_cuda.cuh>

#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>
#include <queue>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GridMap.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

#include <nav_msgs/Odometry.h>

// typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

namespace AstarNS{
    class Astar_Nodelet: public nodelet::Nodelet{
        public:
            Astar_Nodelet();
        private:
            virtual void onInit();
            double frequency;
            grid_map_msgs::GridMapConstPtr main_map_ptr;
            ros::NodeHandle nh;
            ros::NodeHandle private_nh;
            std::shared_ptr<ros::Subscriber> map_sub;
            std::shared_ptr<ros::SubscribeOptions> map_sub_opts;
            std::shared_ptr<ros::Subscriber> odom_sub;
            std::shared_ptr<ros::SubscribeOptions> odom_sub_opts;
            ros::Publisher path_pub;
            grid_map::Position map_pos;
            void createMsg();
            grid_map::Length map_length;
            void map_clbk(const grid_map_msgs::GridMapConstPtr& msg_ptr);
            void odom_clbk(nav_msgs::Odometry msg_ptr);
            nav_msgs::Odometry odom;
            float goal_x=25;
            float goal_y=15;
            bool first_odom_clbk=false;
            bool first_map_clbk=false;
            double returnCost(grid_map::Index t1,grid_map::Index t2);
            double calcHeuristic(grid_map::Index current_node,grid_map::Index node2);
            nav_msgs::PathPtr path_msg_ptr;
            class Astar_Graph_Node{
                private:
                grid_map::Index indx;
                grid_map::Position posn;
                bool processed;
                double cost;
                Astar_Nodelet::Astar_Graph_Node *parent;
                public:
                Astar_Graph_Node(grid_map::Index indx, bool processed,double cost, Astar_Nodelet::Astar_Graph_Node *parent);
                Astar_Graph_Node(grid_map::Index indx);
                void show(){
                    std::cout<<"x="<<this->indx.x()<<" y="<<this->indx.y()<<std::endl;
                }
                Astar_Graph_Node* getParent(){
                    return this->parent;
                }
                bool isProcessed();
                void setProcessed(bool a);
                void setParent(Astar_Nodelet::Astar_Graph_Node *a);
                grid_map::Index returnIndex();
                double getCost();
                void setCost(double a);
                



            };
            std::vector<grid_map::Index> getNeighbour(grid_map::Index indx);


            nav_msgs::PathPtr plan(grid_map::Position start, grid_map::Position goal);
            nav_msgs::PathPtr backtrack(Astar_Graph_Node *goalNode);
        };
}
#endif
