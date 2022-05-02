#include <spot_kinova_framework/servers/body_posture_action_server.hpp>

BodyPostureActionServer::BodyPostureActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SpotKinovaWrapper> &mu)
: ActionServerBase(name,nh,mu), as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&BodyPostureActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&BodyPostureActionServer::preemptCallback, this));
  as_.start(); 
}

void BodyPostureActionServer::goalCallback()
{
    feedback_header_stamp_ = 0;
    goal_ = as_.acceptNewGoal();
    
    mu_->state().spot.duration = goal_->duration;
    mu_->state().spot.orientation_ref = tf::Quaternion(goal_->target_pose.orientation.x, goal_->target_pose.orientation.y, goal_->target_pose.orientation.z, goal_->target_pose.orientation.w) ;
    
    start_time_ = ros::Time::now();
    mu_->init_body_posture_ctrl(start_time_);
    
    if (goal_->target_pose.orientation.w < 0.99)
      mu_->state().spot.body_tilted = true;
    else
      mu_->state().spot.body_tilted = false;
    
    mu_->state().spot.orientation_publish = true;

    control_running_ = true;  
}

void BodyPostureActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool BodyPostureActionServer::compute(ros::Time ctime)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  mu_->compute_body_posture_ctrl(ctime);
  
  if (ctime.toSec() - start_time_.toSec() > goal_->duration){
    setSucceeded();
    mu_->state().spot.orientation_publish = false;
    return true;
  }

  if (ctime.toSec() - start_time_.toSec() > 10.0){
    mu_->state().spot.orientation_publish = false;
    setAborted();
    return false;
  }

  return false;
}


void BodyPostureActionServer::signalAbort(bool is_aborted)
{
  setAborted();  
}

void BodyPostureActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  control_running_ = false;
}
void BodyPostureActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}
