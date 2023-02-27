
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_list_macros.hpp>
#include <planner_cpp/astar_nodelet.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <geometry_msgs/PoseStamped.h>
bool foo(const cv::Mat& img, int& x, int& y);
class compare{
    public:
    bool operator()(const std::pair<double,grid_map::Index>&a,const std::pair<double,grid_map::Index>&b){
        return a.first> b.first;
    }
};
namespace AstarNS
{


    Astar_Nodelet::Astar_Nodelet(){}
    void Astar_Nodelet::onInit(){

        // createMsg();

        //* get node handles
        private_nh=getPrivateNodeHandle();
        nh=getMTNodeHandle();
        
        //* get params
        private_nh.param<double>("frequency",frequency,0.05);
        

        //* init subscibers
        map_sub_opts.reset(new ros::SubscribeOptions());
        map_sub_opts->allow_concurrent_callbacks=false;
        // std::cout<<"DID SOMETHING"<<std::endl;

        map_sub_opts->initByFullCallbackType<const grid_map_msgs::GridMapConstPtr&>("map_in",1,boost::bind(&Astar_Nodelet::map_clbk,this,_1));
        // std::cout<<"DIDNT SOMETHING"<<std::endl;
        
        map_sub.reset(new ros::Subscriber());
        // std::cout<<"DIDNT SOMETHING1"<<std::endl;

        (*map_sub)=nh.subscribe((*map_sub_opts));

        odom_sub_opts.reset(new ros::SubscribeOptions());
        odom_sub_opts->allow_concurrent_callbacks=false;
        odom_sub_opts->initByFullCallbackType<nav_msgs::Odometry>("odom_in",10,boost::bind(&Astar_Nodelet::odom_clbk,this,_1));
        odom_sub.reset(new ros::Subscriber());
        (*odom_sub)=nh.subscribe((*odom_sub_opts));

        path_pub=nh.advertise<nav_msgs::Path>("/planned_path",10);
        map_pos=grid_map::Position::Zero();
        // map_length=;
        path_msg_ptr.reset(new nav_msgs::Path);
        NODELET_INFO_STREAM("INIT DONE");
        // ros::AsyncSpinner asyn(4);
        // asyn.start();
        // ros::waitForShutdown();



        
    }

    void Astar_Nodelet::map_clbk(const grid_map_msgs::GridMapConstPtr& msg_ptr){
        NODELET_INFO_STREAM("GOT MAP");

        // ros::Rate r(0.5);
        // std::cout<<"DID SOMETHING"<<std::endl;

        while(!first_odom_clbk){
        NODELET_INFO_STREAM("LOOP_STUCK");
            
        };

        this->main_map_ptr=msg_ptr;
        std::cout<<"DIDNT SOMETHING"<<std::endl;

        first_map_clbk=true;
        this->path_msg_ptr= this->plan({this->odom.pose.pose.position.x,this->odom.pose.pose.position.y},{this->goal_x,this->goal_y});
        if(this->path_msg_ptr) path_pub.publish(this->path_msg_ptr);
        std::cout<<"DIDNT SOMETHING"<<std::endl;
        
        // r.sleep();
    }
    Astar_Nodelet::Astar_Graph_Node::Astar_Graph_Node(grid_map::Index indx, bool processed,double cost, Astar_Nodelet::Astar_Graph_Node *parent){
        this->indx=indx;
        this->processed=processed;
        this->parent=parent;
        this->cost=cost;
    }

    Astar_Nodelet::Astar_Graph_Node::Astar_Graph_Node(grid_map::Index indx){
        this->indx=indx;
        this->processed=false;
        this->parent=nullptr;
        this->cost=std::numeric_limits<double>::infinity();
    }


    std::vector<grid_map::Index> Astar_Nodelet::getNeighbour(grid_map::Index indx){
        std::vector<grid_map::Index> neighbours;
        int map_width=main_map_ptr->info.length_x;
        int map_height=main_map_ptr->info.length_y;
        int x=indx.x();
        int y=indx.y();
        if((x+1)<map_width){
            neighbours.push_back(grid_map::Index({x+1,y}));
            if((y+1)<map_height){
            neighbours.push_back(grid_map::Index({x+1,y+1}));
            }
            if((y-1)>=0){
            neighbours.push_back(grid_map::Index({x+1,y-1}));

            }
        }
        if((x-1)>=0){
            neighbours.push_back(grid_map::Index({x-1,y}));
            if((y+1)<map_height){
            neighbours.push_back(grid_map::Index({x-1,y+1}));
            }
            if((y-1)>=0){
            neighbours.push_back(grid_map::Index({x-1,y-1}));
            }
        }
        if((y+1)<map_height){
            neighbours.push_back(grid_map::Index({x,y+1}));
        }
        if((y-1)>=0){
            neighbours.push_back(grid_map::Index({x,y-1}));
        }
        return neighbours;

        
        
    }
    


double Astar_Nodelet::returnCost(grid_map::Index t1,grid_map::Index t2){
        
        float dist = sqrt(((t1-t2).x())^2+((t1-t2).y())^2);
        
        float z1=this->main_map_ptr->data[0].data[t1.x()+t1.y()*this->main_map_ptr->info.length_x/this->main_map_ptr->info.resolution];
        float z2=this->main_map_ptr->data[0].data[t2.x()+t2.y()*this->main_map_ptr->info.length_x/this->main_map_ptr->info.resolution];        
        if(std::isnan(z1)){
            z1=0;
        }
        if(std::isnan(z2)){
            z2=0;
        }
        float gradient=(z2-z1)/dist;
        return abs(gradient);
    }


double Astar_Nodelet::calcHeuristic(grid_map::Index current_node,grid_map::Index node2){
        int dx,dy;        
        dx = std::abs((current_node-node2).x());
        dy = std::abs((current_node-node2).y());
        double h = (dx + dy) + (std::sqrt(2) - 2) * std::min(dx, dy);
        return h;
    }


nav_msgs::PathPtr Astar_Nodelet::backtrack(Astar_Graph_Node *goalNode){
        geometry_msgs::PoseStamped pose;
        grid_map::Position pos;
        
        pose.header.frame_id="map";
        pose.header.stamp=ros::Time::now();
        pose.pose.orientation.w=1;
        float z1_old=this->odom.pose.pose.position.z;
        while(goalNode && !(ros::isShuttingDown())){
            grid_map::Index t1=goalNode->returnIndex();
            grid_map::getPositionFromIndex(pos,t1,grid_map::Length(this->main_map_ptr->info.length_x,this->main_map_ptr->info.length_y),this->map_pos,this->main_map_ptr->info.resolution,grid_map::Size(this->main_map_ptr->info.length_x/this->main_map_ptr->info.resolution,this->main_map_ptr->info.length_y/this->main_map_ptr->info.resolution));
            pose.pose.position.x=pos.x();
            pose.pose.position.y=pos.y();
            
            float z1_new=this->main_map_ptr->data[0].data[t1.x()+t1.y()*this->main_map_ptr->info.length_x/this->main_map_ptr->info.resolution];
            if(isnan(z1_new)){
                z1_new=z1_old;
            }
            z1_old=z1_new;
            pose.pose.position.z=z1_new+0.3;
            
            this->path_msg_ptr->poses.push_back(pose);
            goalNode=goalNode->getParent();
        }
        std::reverse(this->path_msg_ptr->poses.begin(),this->path_msg_ptr->poses.end());
        return this->path_msg_ptr;    
    }










nav_msgs::PathPtr Astar_Nodelet::plan(grid_map::Position start, grid_map::Position goal){
        path_msg_ptr.reset(new nav_msgs::Path);

    NODELET_INFO_STREAM("STARTED_PLANNING");
    std::map<std::pair<int,int>,Astar_Nodelet::Astar_Graph_Node> reached;
    std::priority_queue<std::pair<double,grid_map::Index>,std::vector<std::pair<double,grid_map::Index>>,compare> pq;

    grid_map::Index start_indx;
    grid_map::Index goal_indx;

    grid_map::getIndexFromPosition(start_indx,start,grid_map::Length(this->main_map_ptr->info.length_x,this->main_map_ptr->info.length_y),this->map_pos,this->main_map_ptr->info.resolution,grid_map::Size(this->main_map_ptr->info.length_x/this->main_map_ptr->info.resolution,this->main_map_ptr->info.length_y/this->main_map_ptr->info.resolution));
    grid_map::getIndexFromPosition(goal_indx,goal,grid_map::Length(this->main_map_ptr->info.length_x,this->main_map_ptr->info.length_y),this->map_pos,this->main_map_ptr->info.resolution,grid_map::Size(this->main_map_ptr->info.length_x/this->main_map_ptr->info.resolution,this->main_map_ptr->info.length_y/this->main_map_ptr->info.resolution));
    NODELET_INFO_STREAM("GOT GOAL+START INDICES");

    
    pq.push({0.0+this->calcHeuristic(start_indx,goal_indx),start_indx});

    Astar_Nodelet::Astar_Graph_Node start_node(start_indx);
    NODELET_INFO_STREAM("START: {"<<start_indx.x()<<","<<start_indx.y()<<" processed: "<<start_node.isProcessed());
    start_node.setCost(0);
    reached.insert({{start_indx.x(),start_indx.y()},start_node});
    grid_map::Index t;
    double d;
    double doubleCheck=reached.at({start_indx.x(),start_indx.y()}).getCost()+this->calcHeuristic(start_indx,goal_indx);
    int iterations=0;
    this->path_msg_ptr->header.frame_id="map";
    this->path_msg_ptr->header.stamp=ros::Time::now();


    NODELET_INFO_STREAM("SEARCH LOOP");

    while(!pq.empty() && ros::ok()){
        iterations++;
        std::tie(d,t)=pq.top();
        pq.pop();
        if(reached.at({t.x(),t.y()}).isProcessed()){
            // NODELET_INFO_STREAM("REACHED_POINT, "<<iterations<<" "<<t.x()<<" "<<t.y());

            continue;
        }
        reached.at({t.x(),t.y()}).setProcessed(true);
        if(t.x()==goal_indx.x() && t.y()==goal_indx.y()){
                NODELET_INFO_STREAM("REACHED_GOAL");

                Astar_Graph_Node node= reached.at({t.x(),t.y()});
                this->backtrack(&node);
                std::cout<<"Goal reached"<<std::endl;
                return this->path_msg_ptr;
        }
        std::vector<grid_map::Index> l=this->getNeighbour(t);
        // NODELET_INFO_STREAM("GOT_NEIGHBOUR");

        for(grid_map::Index ind: l){
            if(!reached.count({ind.x(),ind.y()})){
                Astar_Graph_Node n(ind);
                n.setParent(&(reached.at({t.x(),t.y()})));
                n.setCost(reached.at({t.x(),t.y()}).getCost()+this->returnCost(t,ind));
                reached.insert({{ind.x(),ind.y()},n});
                pq.push({n.getCost()+this->calcHeuristic(ind,goal_indx),ind});

            }else if (reached.at({t.x(),t.y()}).getCost()+this->returnCost(t,ind)<reached.at({ind.x(),ind.y()}).getCost())
                {
                    Astar_Nodelet::Astar_Graph_Node n=reached.at({ind.x(),ind.y()});
                    
                    n.setParent(&(reached.at({t.x(),t.y()})));
                    n.setCost(reached.at({t.x(),t.y()}).getCost()+this->returnCost(t,ind));
                    reached.at({ind.x(),ind.y()})=n;
                    pq.push({reached.at({ind.x(),ind.y()}).getCost()+this->calcHeuristic(ind,goal_indx),ind});

                }

        }
        if (iterations > 100000){
            std::cout<<"couldn't reach goal"<<std::endl;
            return this->path_msg_ptr;;  
        }
    }
}


void Astar_Nodelet::odom_clbk(nav_msgs::Odometry msg){
    // NODELET_INFO_STREAM("GOT ODOM");

    this->odom=msg;
    this->first_odom_clbk=true;
}


bool Astar_Nodelet::Astar_Graph_Node::isProcessed(){
                std::cout<<"IF_OROCESSED "<<this->processed<<std::endl;
                

                    return this->processed;
                }
                void Astar_Nodelet::Astar_Graph_Node::setProcessed(bool a){
                    // NODELET_INFO_STREAM("PROCESSES");
                    this->processed=a;
                }
                void Astar_Nodelet::Astar_Graph_Node::setParent(Astar_Nodelet::Astar_Graph_Node *a){
                    this->parent=a;
                }
                grid_map::Index Astar_Nodelet::Astar_Graph_Node::returnIndex(){
                    return this->indx;
                }
                double Astar_Nodelet::Astar_Graph_Node::getCost(){
                    return this->cost;
                }
                void Astar_Nodelet::Astar_Graph_Node::setCost(double a){
                    this->cost=a;
                }
                
/******************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
    * * Subscriber Callback
    * todo: ADD GridMap Publishing
********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/




    


    
}; // namespace 
PLUGINLIB_EXPORT_CLASS(AstarNS::Astar_Nodelet, nodelet::Nodelet);



