#include <spot_kinova_framework/servers/wholebody_action_server.hpp>
#include <pinocchio/fwd.hpp>

using namespace pinocchio;

WholebodyActionServer::WholebodyActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SpotKinovaWrapper> &mu)
: ActionServerBase(name,nh,mu), as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&WholebodyActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&WholebodyActionServer::preemptCallback, this));
  as_.start();
  isrelative_ = true;
}

void WholebodyActionServer::goalCallback()
{
    feedback_header_stamp_ = 0;
    goal_ = as_.acceptNewGoal();
    
    mode_change_ = false;
    isrelative_ = goal_->relative;

    if (!mu_->simulation())
      mu_->state().kinova.lowlevel_ctrl = true;
      

    //for arm
    if (!isrelative_){
      Eigen::Vector3d pos(goal_->target_ee_pose.position.x, goal_->target_ee_pose.position.y, goal_->target_ee_pose.position.z);
      Eigen::Quaterniond quat(goal_->target_ee_pose.orientation.w, goal_->target_ee_pose.orientation.x, goal_->target_ee_pose.orientation.y, goal_->target_ee_pose.orientation.z);
      SE3 oMi_ref(quat.toRotationMatrix(), pos);
      mu_->state().kinova.H_ee_ref = oMi_ref;
    }
    else{
      Eigen::Quaterniond recieve_quat_eigen(goal_->target_ee_pose.orientation.w, goal_->target_ee_pose.orientation.x, goal_->target_ee_pose.orientation.y,goal_->target_ee_pose.orientation.z);
      Eigen::Vector3d pos(goal_->target_ee_pose.position.x + mu_->state().kinova.H_ee.translation()(0),
                         goal_->target_ee_pose.position.y + mu_->state().kinova.H_ee.translation()(1),
                         goal_->target_ee_pose.position.z + + mu_->state().kinova.H_ee.translation()(2));
      Eigen::Quaterniond current_quat_eigen(mu_->state().kinova.H_ee.rotation());

      SE3 oMi_ref( (recieve_quat_eigen * current_quat_eigen ).toRotationMatrix(), pos);
      mu_->state().kinova.H_ee_ref = oMi_ref;
    }
    mu_->state().kinova.duration = goal_->duration;

    
    // for spot
    mu_->state().spot.duration = goal_->duration;
    mu_->state().spot.orientation_ref = tf::Quaternion(goal_->target_body_pose.orientation.x, goal_->target_body_pose.orientation.y, goal_->target_body_pose.orientation.z, goal_->target_body_pose.orientation.w) ;
        
    if (goal_->target_body_pose.orientation.w < 0.98)
      mu_->state().spot.body_tilted = true;
    else
      mu_->state().spot.body_tilted = false;
    
    mu_->state().spot.orientation_publish = true;
    
    start_time_ = ros::Time::now();
    mu_->init_body_posture_ctrl(start_time_);
    mu_->init_se3_ctrl(start_time_);
    control_running_ = true;  
}

void WholebodyActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool WholebodyActionServer::compute(ros::Time ctime)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  mu_->compute_se3_ctrl(ctime);
  mu_->compute_body_posture_ctrl(ctime);
  

  if (ctime.toSec() - start_time_.toSec() > goal_->duration && (mu_->state().kinova.H_ee.translation()-mu_->state().kinova.H_ee_ref.translation()).norm() < 5e-3){
    setSucceeded();
    mu_->state().spot.orientation_publish = false;
    return true;
  }

  if (ctime.toSec() - start_time_.toSec()  > goal_->duration+ 5.0){
    setAborted();
    mu_->state().spot.orientation_publish = false;
    return false;
  }

  return false;
}


void WholebodyActionServer::signalAbort(bool is_aborted)
{
  setAborted();  
}

void WholebodyActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  if (!mu_->simulation())
    mu_->done_se3_ctrl();
  control_running_ = false;
}
void WholebodyActionServer::setAborted()
{
  as_.setAborted();
  if (!mu_->simulation())
    mu_->done_se3_ctrl();
  control_running_ = false;
}