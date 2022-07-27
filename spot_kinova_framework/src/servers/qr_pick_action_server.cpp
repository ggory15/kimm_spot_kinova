#include <spot_kinova_framework/servers/qr_pick_action_server.hpp>
#include <Eigen/Core>

using namespace Eigen;

QRPickActionServer::QRPickActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SpotKinovaWrapper> &mu)
: ActionServerBase(name,nh,mu), as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&QRPickActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&QRPickActionServer::preemptCallback, this));
  as_.start();

}

void QRPickActionServer::goalCallback()
{
    feedback_header_stamp_ = 0;
    goal_ = as_.acceptNewGoal();
    
    mode_change_ = true;
    if (!mu_->simulation())
      mu_->state().kinova.lowlevel_ctrl = true;
    
    mu_->state().kinova.isrelative = false;
    start_time_ = ros::Time::now();   

    // ROS_WARN_STREAM(mu_->state().kinova.H_ee);

    control_running_ = true;  
}

void QRPickActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool QRPickActionServer::compute(ros::Time ctime)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 

  if (ctime.toSec() - start_time_.toSec() > 1.0){
    if (mode_change_){
      SE3 target_tf;
      mu_->state().kinova.H_ee_ref_array.resize(2);
      mu_->state().kinova.duration_array.resize(2);

      qr_tf_ = SE3(Eigen::Quaterniond(goal_->qr_pose.orientation.w, goal_->qr_pose.orientation.x, goal_->qr_pose.orientation.y, goal_->qr_pose.orientation.z).toRotationMatrix(),
                      Eigen::Vector3d(goal_->qr_pose.position.x, goal_->qr_pose.position.y, goal_->qr_pose.position.z));
      target_tf = SE3(Eigen::Quaterniond(goal_->target_pose.orientation.w, goal_->target_pose.orientation.x, goal_->target_pose.orientation.y, goal_->target_pose.orientation.z).toRotationMatrix(),
                      Eigen::Vector3d(goal_->target_pose.position.x, goal_->target_pose.position.y, goal_->target_pose.position.z + goal_->approach_offset));
      mu_->state().kinova.H_ee_ref_array[0] = qr_tf_ * target_tf;
      
      // mu_->state().kinova.H_ee_ref_array[0].translation()(2) = mu_->state().kinova.H_ee.translation()(2);                  

      //align qr marker tf to joint7 tf
      Eigen::Matrix3d rot_offset; //[0 -1 0; 1 0 0; 0 0 1]
      rot_offset.setZero();
      rot_offset(0, 1) = -1.0;
      rot_offset(1, 0) = 1.0;
      rot_offset(2, 2) = 1.0;      
      mu_->state().kinova.H_ee_ref_array[0].rotation() = mu_->state().kinova.H_ee_ref_array[0].rotation()*rot_offset;
           
      target_tf = SE3(Eigen::Quaterniond(goal_->target_pose.orientation.w, goal_->target_pose.orientation.x, goal_->target_pose.orientation.y, goal_->target_pose.orientation.z).toRotationMatrix(),
                      Eigen::Vector3d(goal_->target_pose.position.x, goal_->target_pose.position.y, goal_->target_pose.position.z));
      mu_->state().kinova.H_ee_ref_array[1] = qr_tf_ * target_tf;      
      mu_->state().kinova.H_ee_ref_array[1].rotation() = mu_->state().kinova.H_ee_ref_array[1].rotation()*rot_offset;

      for (int i=0; i<2; i++){
        mu_->state().kinova.duration_array[i] = goal_->duration;
      }
      
      mu_->init_se3_array_ctrl(ctime);
      mode_change_ = false;
      
    }
    mu_->compute_se3_array_ctrl(ctime);
 
    if (ctime.toSec() - start_time_.toSec() > goal_->duration * 2.0 +3.0){
      setSucceeded();
      return true;
    }
  }
 
  return false;
}


void QRPickActionServer::signalAbort(bool is_aborted)
{
  setAborted();  
}

void QRPickActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  if (!mu_->simulation())
    mu_->done_se3_ctrl();
  control_running_ = false;
}
void QRPickActionServer::setAborted()
{
  as_.setAborted();
  if (!mu_->simulation())
    mu_->done_se3_ctrl();
  control_running_ = false;
}
