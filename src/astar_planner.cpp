#include "matplotlib.h"
#include <eigen3/Eigen/Dense>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <queue>
#include <unordered_map>
#include "planner_cpp/astar.h"
#include <armadillo>
#include "opencv4/opencv2/highgui.hpp"
#include "opencv4/opencv2/imgcodecs.hpp"
#include "opencv4/opencv2/core/core.hpp"
#include "ros/ros.h"
#include "opencv4/opencv2/imgproc.hpp"
#include "matplotlib.h"

namespace plt=matplotlibcpp;
bool foo(const cv::Mat& img, int& x, int& y);
class compare{
    public:
    bool operator()(const std::pair<double,std::pair<int,int>>&a,const std::pair<double,std::pair<int,int>>&b){
        return a.first> b.first;
    }
};

class Astar_Planner{
    private:
    int map_width=4000;
    int map_height=4000;
    double map_resolution=0.05;
    int world_min_x;
    int world_min_y;

    cv::Mat obstacle_map;

    public:
    Astar_Planner(double resolution,int world_min_x,int world_min_y,cv::Mat *obstacle_map,int x_width,int y_width){
        this->obstacle_map=(*obstacle_map);
        this->world_min_x=world_min_x;
        this->world_min_y=world_min_y;

    }
    class Astar_Node{
        private:
        int x,y;
        bool processed;
        double cost;
        Astar_Planner::Astar_Node *parent;
        std::priority_queue<std::pair<int,int>> pq;
        public:
        // Astar_Node(){
        //     this->processed=false;

        // }
        Astar_Node(int x,int y, bool processed,double cost, Astar_Planner::Astar_Node *parent){
            this->x=x;
            this->y=y;
            this->processed=processed;
            this->parent=parent;
            this->cost=cost;
        }
        Astar_Node(std::pair<int,int>tup){
            std::tie(x,y)=tup;
            this->x=x;
            this->y=y;
            this->processed=false;
            this->parent=nullptr;
            this->cost=std::numeric_limits<double>::infinity();
        }
        void show(){
            std::cout<<"x="<<this->x<<" y="<<this->y<<std::endl;
        }
        Astar_Node* getParent(){
            return this->parent;
        }
        bool isProcessed(){
            return this->processed;
        }
        void setProcessed(bool a){
            this->processed=a;
        }
        void setParent(Astar_Node *a){
            this->parent=a;
        }
        int returnX(){
            return this->x;
        }
        int returnY(){
            return this->y;
        }
        double getCost(){
            return this->cost;
        }
        void setCost(double a){
            this->cost=a;
        }


    };







    std::vector<std::pair<int,int>> getNeighbour(std::pair<int,int> node){
            int x,y;
            std::vector<std::pair<int,int>> neighbours;
            std::tie(x,y)=node;
            if((x+1)<map_width){
                neighbours.push_back({x+1,y});
                if((y+1)<map_height){
                neighbours.push_back({x+1,y+1});
                }
                if((y-1)>0){
                neighbours.push_back({x+1,y-1});

                }
            }
            if((x-1)>0){
                neighbours.push_back({x-1,y});
                if((y+1)<map_height){
                neighbours.push_back({x-1,y+1});
                }
                if((y-1)>0){
                neighbours.push_back({x-1,y-1});
                }
            }
            if((y+1)<map_height){
                neighbours.push_back({x,y+1});
            }
            if((y-1)>0){
                neighbours.push_back({x,y-1});
            }

            return neighbours;
        }

    std::pair<std::vector<double>,std::vector<double>> backtrack(Astar_Node *goalNode){
        std::vector<double> rx,ry;
        while(goalNode && !(ros::isShuttingDown())){
            rx.push_back(this->calc_grid_position(goalNode->returnX(),this->world_min_x));
            ry.push_back(this->calc_grid_position(goalNode->returnY(),this->world_min_y));
            goalNode=goalNode->getParent();
        }
        std::reverse(rx.begin(),rx.end());
        std::reverse(ry.begin(),ry.end());
        return {rx,ry};    
    }


    double returnCost(std::pair<int,int> t1,std::pair<int,int> t2){
        int x1,y1,x2,y2;
        std::tie(x1,y1)=t1;
        std::tie(x2,y2)=t2;

        double t=std::max((int)(this->obstacle_map.at<int8_t>(y1,x1)), 0)+std::max((int)(this->obstacle_map.at<int>(y2,x1)), 0);

        if (t>80){
            
            t=std::numeric_limits<double>::infinity();
            return t;
        }
        else{
            return this->distance(t1,t2);
        }
    }

    double turningCost(Astar_Node a, Astar_Node b){
        int x1=a.returnX();
        int x2=b.returnX();
        int y1=a.returnY();
        int y2=b.returnY();
        if(b.getParent()){
        int x3=b.getParent()->returnX();
        int y3=b.getParent()->returnY();

        double angle1=std::atan2(y1-y2,x1-x2);
        double angle2=std::atan2(y2-y3,x2-x3);

        if(angle1==angle2){
            return 0;
        }
        else{
            std::cout<<0.05<<",";
            return 1.5;
        }}else{
            return 0;
        }


    }


    double distance(std::pair<int,int> current_node,std::pair<int,int> node2){
        int x,y,x1,y1;
        std::tie(x,y)=current_node;
        std::tie(x1,y1)=node2;
        return std::sqrt(std::pow((x-x1),2) + std::pow((y-y1),2));
    }


    double calcHeuristic(std::pair<int,int> current_node,std::pair<int,int> node2){
        int x,y,x1,y1,dx,dy;
        std::tie(x,y)=current_node;
        std::tie(x1,y1)=node2;
        dx = std::abs(x - x1);
        dy = std::abs(y - y1);
        double h = (dx + dy) + (std::sqrt(2) - 2) * std::min(dx, dy);
        return h;
    }


    int calc_xy_index(double coordinate, double min_pos){
        return std::round((coordinate-min_pos)/this->map_resolution);
    } 


    double calc_grid_position(int index,double min_pos){
        return index*this->map_resolution+min_pos;
    }


    std::tuple<bool,std::vector<double>,std::vector<double>> plan(double sx,double sy, double gx, double gy){
        std::cout<<sx<<sy<<gx<<gy<<std::endl;
        std::priority_queue<std::pair<double,std::pair<int,int>>,std::vector<std::pair<double,std::pair<int,int>>>,compare> pq;
        std::map<std::pair<int,int>,Astar_Planner::Astar_Node> reached;
        int iteration=0;
        std::pair<int,int>start={calc_xy_index(sx,this->world_min_x),calc_xy_index(sy,this->world_min_y)};
        std::pair<int,int>goal={calc_xy_index(gx,this->world_min_x),calc_xy_index(gy,this->world_min_y)};
        pq.push({0.0+this->distance(start,goal),start});
        Astar_Node start_node=Astar_Node(start);
        start_node.setCost(0);
        reached.insert({start,start_node});
        std::pair<int,int> t;
        double d;
        
        while (!(pq.empty()) && !(ros::isShuttingDown())){
            iteration++;
            
            
            std::tie(d,t)=pq.top();
            std::vector<double>rx,ry;
            pq.pop();


            if (reached.at(t).isProcessed())
                {continue;}
            reached.at(t).setProcessed(true);
            if(t==goal){

                Astar_Node node= reached.at(t);
                std::tie(rx,ry)=this->backtrack(&node);
                std::cout<<"Goal reached"<<std::endl;
                return {true,rx,ry};
            }
            std::vector<std::pair<int,int>> l=this->getNeighbour(t);
            for(std::pair<int,int> tup:l){
                if(!(reached.count(tup))){
                    Astar_Node n=Astar_Node(tup);
                    n.setCost(reached.at(t).getCost()+this->returnCost(t,tup)+this->turningCost(n,reached.at(t)));
                    n.setParent(&(reached.at(t)));
                    reached.insert({tup,n});

                    pq.push({n.getCost()+this->calcHeuristic(goal,tup),tup});
                }
                else if ((reached.at(tup).getCost()+this->returnCost(t,tup))<reached.at(tup).getCost())
                {
                    Astar_Node n=reached.at(tup);
                    n.setCost(reached.at(t).getCost()+this->returnCost(t,tup));
                    n.setParent(&(reached.at(t)));
                    pq.push({n.getCost()+this->calcHeuristic(goal,tup),tup});
                }
                
            }

            if (iteration > 5000){
                std::cout<<"couldn't reach goal"<<std::endl;
            }
        }
        return {false,{},{}};  

 

    }


};

bool main_planner(planner_cpp::astar::Request &req, planner_cpp::astar::Response &res){
    std::cout<<"started planning"<<std::endl;

    geometry_msgs::Point start_point=req.start_pos;
    geometry_msgs::Point goal_point=req.goal_pos;
    nav_msgs::OccupancyGrid obstacle_map=req.obstacle_map;
    double sx,sy,gx,gy;
    sx=start_point.x;
    sy=start_point.y;
    gx=goal_point.x;
    gy=goal_point.y;
    int map_width=obstacle_map.info.width;
    int map_height=obstacle_map.info.height;
    std::vector<int8_t> map_data=obstacle_map.data;
    float map_resolution = obstacle_map.info.resolution;
    double map_origin_x = obstacle_map.info.origin.position.x;
    double map_origin_y = obstacle_map.info.origin.position.y;
    std::vector<int> obs_coords;
    std::vector<int8_t>::iterator it=map_data.begin();
    cv::Mat cv_1d_map(map_data);
    cv::Mat cv_2d_map=cv_1d_map.reshape(1,map_height);
    int bot_radius=0.225;
    int map_bot_radius=bot_radius/obstacle_map.info.resolution;
    int indx;
    int x;
    int y;
    while(it<=map_data.end() && !(ros::isShuttingDown())){


        it=std::find(it,map_data.end(),100);
        indx=std::distance(map_data.begin(),it);
        obs_coords.push_back(indx);
        
        it++;
    }
    std::vector<float> obs_pos_x;
    std::vector<float> obs_pos_y;

    for(auto i:obs_coords){

        y=i/map_height;
        x=i-y*map_height;
        // std::cout<<x<<","<<y<<" ";
        obs_pos_x.push_back(x*obstacle_map.info.resolution+obstacle_map.info.origin.position.x);
        obs_pos_y.push_back(y*obstacle_map.info.resolution+obstacle_map.info.origin.position.y);

        cv::circle(cv_2d_map,cv::Point(x,y),4,cv::Scalar(100),-1);
    }



    Astar_Planner planner=Astar_Planner(map_resolution,map_origin_x,map_origin_y,&cv_2d_map,map_width,map_height);



    std::tie(res.ack,res.trajectory_x,res.trajectory_y)=planner.plan(sx,sy,gx,gy);

    plt::plot(res.trajectory_x,res.trajectory_y,"-r");
    plt::plot(obs_pos_x,obs_pos_y,".k");
    plt::show();
    while(!(ros::isShuttingDown())){
        cv::imshow("test",cv_2d_map);
        cv::waitKey(2);
    }
    return true;

    

}

int main(int argc, char **argv){
    ros::init(argc, argv, "astar_service");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("astar_service", main_planner);
    std::cout<<"spinning"<<std::endl;
    ros::spin();

    


    return 0;
}
