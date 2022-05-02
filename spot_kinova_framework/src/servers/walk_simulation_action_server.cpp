#include <spot_kinova_framework/servers/walk_simulation_action_server.hpp>

WalkSimulationActionServer::WalkSimulationActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SpotKinovaWrapper> &mu)
: ActionServerBase(name,nh,mu), as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&WalkSimulationActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&WalkSimulationActionServer::preemptCallback, this));
  as_.start();

  move_base_subscriber_ = nh.subscribe("move_base/result", 1, &WalkSimulationActionServer::movebaseCallback, this);
  move_base_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 100);   
  isrelative_ = true;
}

void WalkSimulationActionServer::goalCallback()
{
    feedback_header_stamp_ = 0;
    goal_ = as_.acceptNewGoal();
    
    mode_change_ = false;
    walk_done_ = false;
    
    isrelative_ = goal_->relative;
    if (!mu_->state().spot.body_tilted){
      if (!isrelative_)
        move_base_publisher_.publish(goal_->target_pose);
      else{
        msg_ = goal_->target_pose;
        msg_.pose.position.x += mu_->state().q(0);
        msg_.pose.position.y += mu_->state().q(1);
        msg_.pose.position.z += mu_->state().q(2);
        tf::Quaternion recieve_quat = mu_->state().spot.nav;
        tf::Quaternion current_quat(msg_.pose.orientation.x, msg_.pose.orientation.y, msg_.pose.orientation.z, msg_.pose.orientation.w);
        tf::Quaternion resultant_quat = recieve_quat * current_quat;
        msg_.pose.orientation.x = resultant_quat.getX();
        msg_.pose.orientation.y = resultant_quat.getY();
        msg_.pose.orientation.z = resultant_quat.getZ();
        msg_.pose.orientation.w = resultant_quat.getW();
        
        move_base_publisher_.publish(msg_);
      }
    }
    start_time_ = ros::Time::now();

    control_running_ = true;  
}

void WalkSimulationActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool WalkSimulationActionServer::compute(ros::Time ctime)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 

  if (!mu_->state().spot.body_tilted){    
    ROS_WARN_STREAM("Body is tilted. Cannot WalkSimulation for safety.");
    setAborted();
    return false;
  }
  
  if (walk_done_){
    setSucceeded();
    return true;
  }

  if (ctime.toSec() - start_time_.toSec() > 20.0){
    setAborted();
    return false;
  }

  return false;
}


void WalkSimulationActionServer::signalAbort(bool is_aborted)
{
  setAborted();  
}

void WalkSimulationActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  control_running_ = false;
}
void WalkSimulationActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}
void WalkSimulationActionServer::movebaseCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
    if (msg->status.status == 3)
      walk_done_ = true;
    else
      walk_done_ = false;
}