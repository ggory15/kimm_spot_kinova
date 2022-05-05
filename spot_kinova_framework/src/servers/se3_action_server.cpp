#include <spot_kinova_framework/servers/se3_action_server.hpp>
#include <pinocchio/fwd.hpp>

using namespace pinocchio;

SE3ActionServer::SE3ActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SpotKinovaWrapper> &mu)
: ActionServerBase(name,nh,mu), as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&SE3ActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&SE3ActionServer::preemptCallback, this));
  as_.start();
  isrelative_ = true;
}

void SE3ActionServer::goalCallback()
{
    feedback_header_stamp_ = 0;
    goal_ = as_.acceptNewGoal();
    
    mode_change_ = false;
    isrelative_ = goal_->relative;
    mu_->state().kinova.isrelative = isrelative_;

    Eigen::Vector3d pos(goal_->target_pose.position.x, goal_->target_pose.position.y, goal_->target_pose.position.z);
    Eigen::Quaterniond quat(goal_->target_pose.orientation.w, goal_->target_pose.orientation.x, goal_->target_pose.orientation.y, goal_->target_pose.orientation.z);
    SE3 oMi_ref(quat.toRotationMatrix(), pos);
    mu_->state().kinova.H_ee_ref = oMi_ref;

    mu_->state().kinova.duration = goal_->duration;

    if (!mu_->simulation())
      mu_->state().kinova.lowlevel_ctrl = true;
        
    start_time_ = ros::Time::now();
    mu_->init_se3_ctrl(start_time_);
    control_running_ = true;  
}

void SE3ActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool SE3ActionServer::compute(ros::Time ctime)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  mu_->compute_se3_ctrl(ctime);
  
  //ROS_WARN_STREAM((mu_->state().kinova.H_ee.translation()-mu_->state().kinova.H_ee_ref.translation()).norm());
  if (ctime.toSec() - start_time_.toSec() > goal_->duration +1.0 && (mu_->state().kinova.H_ee.translation()-mu_->state().kinova.H_ee_ref.translation()).norm() < 5e-3){
    setSucceeded();
    return true;
  }

  if (ctime.toSec() - start_time_.toSec()  > goal_->duration+ 5.0){
    setAborted();
    return false;
  }

  return false;
}


void SE3ActionServer::signalAbort(bool is_aborted)
{
  setAborted();  
}

void SE3ActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  if (!mu_->simulation())
    mu_->done_se3_ctrl();
  control_running_ = false;
}
void SE3ActionServer::setAborted()
{
  as_.setAborted();
  if (!mu_->simulation())
    mu_->done_se3_ctrl();
  control_running_ = false;
}