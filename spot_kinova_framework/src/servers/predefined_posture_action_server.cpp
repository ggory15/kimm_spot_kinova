#include <spot_kinova_framework/servers/predefined_posture_action_server.hpp>

PredefinedPostureActionServer::PredefinedPostureActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SpotKinovaWrapper> &mu)
: ActionServerBase(name,nh,mu), as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&PredefinedPostureActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&PredefinedPostureActionServer::preemptCallback, this));
  as_.start();  
}

void PredefinedPostureActionServer::goalCallback()
{
    feedback_header_stamp_ = 0;
    goal_ = as_.acceptNewGoal();
    
    mu_->state().kinova.lowlevel_ctrl = false;
    string posture_name = goal_->posture_name;
    //ROS_WARN_STREAM(posture_name);
    mu_->init_predefined_posture_ctrl(posture_name);

    start_time_ = ros::Time::now();
    control_running_ = true;  
}

void PredefinedPostureActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool PredefinedPostureActionServer::compute(ros::Time ctime)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  if (ctime.toSec() - start_time_.toSec() > 1.0){
    setSucceeded();
    return true;
  }

  return false;
}


void PredefinedPostureActionServer::signalAbort(bool is_aborted)
{
  setAborted();  
}

void PredefinedPostureActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  mu_->done_se3_ctrl();
  control_running_ = false;
}
void PredefinedPostureActionServer::setAborted()
{
  as_.setAborted();
  mu_->done_se3_ctrl();
  control_running_ = false;
}