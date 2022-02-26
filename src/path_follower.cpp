// #include <ros/ros.h>
#include <stdio.h>
#include <boost/range/combine.hpp>
#include <actionlib/server/simple_action_server.h>
#include <planner_cpp/PathFollowAction.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <math.h>

class PathFollower
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<planner_cpp::PathFollowAction> as_; 
  ros::Subscriber odom_sub;
  ros::Subscriber laser_scan_sub;
  std::string action_name_;
  planner_cpp::PathFollowFeedback path_follow_feedback_;
  planner_cpp::PathFollowResult path_follow_result_;
  geometry_msgs::Point curr_point;




  bool first_run=true;
  double prev_pos_x;
  double prev_pos_y;
  double cur_pos_x;
  double cur_pos_y;
  double cur_yaw;


  double goal_pos_x;
  double goal_pos_y;


  std::vector<double> trajectory_x;
  std::vector<double> trajectory_y;
  std::vector<double> trajectory_endpoints_x;
  std::vector<double> trajectory_endpoints_y;



public:

  PathFollower(std::string name) :
    as_(nh_, name, boost::bind(&PathFollower::executeCB, this, _1), false),
    action_name_(name)
  {
    this->odom_sub=this->nh_.subscribe("/odom",10,&PathFollower::odom_clbk,this);
    as_.start();
  }

  void odom_clbk(nav_msgs::Odometry msg){
    this->curr_point=msg.pose.pose.position;
    this->cur_pos_x=-msg.pose.pose.position.y;
    this->cur_pos_y=msg.pose.pose.position.x;
    this->cur_yaw=tf::getYaw(msg.pose.pose.orientation)+M_PI_2f64;
  }


  void path_breaker(){
    std::vector<double>::iterator traj_x_iter=this->trajectory_x.begin();
    std::vector<double> x_diff;
    std::vector<double>::iterator traj_y_iter=this->trajectory_y.begin();
    std::vector<double> y_diff;
    std::vector<double> angles;
    
    while((traj_x_iter+1)!=this->trajectory_x.end()){
      x_diff.push_back(*(traj_x_iter)-*(++traj_x_iter));
    }
    while((traj_y_iter+1)!=this->trajectory_y.end()){
      y_diff.push_back(*(traj_y_iter)-*(++traj_y_iter));
    }
    traj_x_iter=x_diff.begin();
    traj_y_iter=y_diff.begin();
    while((traj_x_iter)!=x_diff.end() && (traj_y_iter)!=y_diff.end()){
      angles.push_back(std::atan2(*(traj_y_iter),*(traj_x_iter)));
    }

    for(int i;i<angles.size();i++){
      if(angles[i]>0){
        this->trajectory_endpoints_x.push_back(this->trajectory_x[i]);
        this->trajectory_endpoints_y.push_back(this->trajectory_y[i]);
  
      }
    }
    this->trajectory_endpoints_x.push_back(*(this->trajectory_x.rbegin()));
    this->trajectory_endpoints_y.push_back(*(this->trajectory_y.rbegin()));
    angles.clear();
    angles.shrink_to_fit();
    x_diff.clear();
    x_diff.shrink_to_fit();
    y_diff.clear();
    y_diff.shrink_to_fit();
  }

  double angle_to_turn(){
    double inc_x=this->goal_pos_x-this->cur_pos_x;
    double inc_y=this->goal_pos_y-this->cur_pos_y;       
    double angle_of_goal =std::atan2(inc_y,inc_x);
    double angle_to_turn=angle_of_goal-this->cur_yaw;
    if(std::abs(angle_to_turn)>M_PI){
      return angle_to_turn;
    }else{
      if (angle_to_turn<0){
        return angle_to_turn+2*M_PI;
      }else{
        return angle_to_turn-2*M_PI;

      }
    }
  }
  
  void executeCB(const planner_cpp::PathFollowGoalConstPtr &goal){
    this->trajectory_x=goal->trajectory_x;
    this->trajectory_y=goal->trajectory_y;
    this->path_breaker();
    std::vector<double>::iterator x_iter=this->trajectory_endpoints_x.begin();
    std::vector<double>::iterator y_iter=this->trajectory_endpoints_y.begin();  


    }
//   {
//     // helper variables
//     

//     // push_back the seeds for the fibonacci sequence
//     feedback_.sequence.clear();
//     feedback_.sequence.push_back(0);
//     feedback_.sequence.push_back(1);

//     // publish info to the console for the user
//     ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

//     // start executing the action
//     for(int i=1; i<=goal->order; i++)
//     {
//       // check that preempt has not been requested by the client
//       if (as_.isPreemptRequested() || !ros::ok())
//       {
//         ROS_INFO("%s: Preempted", action_name_.c_str());
//         // set the action state to preempted
//         as_.setPreempted();
//         success = false;
//         break;
//       }
//       feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
//       // publish the feedback
//       as_.publishFeedback(feedback_);
//       // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
//       r.sleep();
//     }

//     if(success)
//     {
//       result_.sequence = feedback_.sequence;
//       ROS_INFO("%s: Succeeded", action_name_.c_str());
//       // set the action state to succeeded
//       as_.setSucceeded(result_);
//     }
//   }


};


int main(int argc, char** argv)
{
//   ros::init(argc, argv, "fibonacci");

//   FibonacciAction fibonacci("fibonacci");
//   ros::spin();

  return 0;
}