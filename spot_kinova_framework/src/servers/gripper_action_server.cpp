#include <spot_kinova_framework/servers/gripper_action_server.hpp>

GripperActionServer::GripperActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SpotKinovaWrapper> &mu)
: ActionServerBase(name,nh,mu), as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&GripperActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&GripperActionServer::preemptCallback, this));
  as_.start();  

  if (!mu_->simulation())
    mu_->done_se3_ctrl();
}

void GripperActionServer::goalCallback()
{
    feedback_header_stamp_ = 0;
    goal_ = as_.acceptNewGoal();

    // if (goal_->open){
    //   mu_->init_open_gripper();
    // }
    // else{
    //   mu_->init_close_girpper();
    // }
    
    if (goal_->position == 0.0){
      mu_->init_open_gripper();
    }
    else if (goal_->position == 1.0){
      mu_->init_close_girpper();
    }
    else{
      mu_->init_position_girpper(goal_->position);
    }

    start_time_ = ros::Time::now();
    control_running_ = true;  
}

void GripperActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool GripperActionServer::compute(ros::Time ctime)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  if (ctime.toSec() - start_time_.toSec() > 2.0){
    setSucceeded();
    return true;
  }

  return false;
}


void GripperActionServer::signalAbort(bool is_aborted)
{
  setAborted();  
}

void GripperActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  if (!mu_->simulation())
    mu_->done_se3_ctrl();
  control_running_ = false;
}
void GripperActionServer::setAborted()
{
  as_.setAborted();
  if (!mu_->simulation())
    mu_->done_se3_ctrl();
  control_running_ = false;
}