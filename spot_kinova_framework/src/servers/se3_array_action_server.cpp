#include <spot_kinova_framework/servers/se3_array_action_server.hpp>
#include <Eigen/Core>

using namespace Eigen;

SE3ArrayActionServer::SE3ArrayActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SpotKinovaWrapper> &mu)
: ActionServerBase(name,nh,mu), as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&SE3ArrayActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&SE3ArrayActionServer::preemptCallback, this));
  as_.start();
}

void SE3ArrayActionServer::goalCallback()
{
    feedback_header_stamp_ = 0;
    goal_ = as_.acceptNewGoal();
       
    mode_change_ = true;
    if (!mu_->simulation())
      mu_->state().kinova.lowlevel_ctrl = true;
    
    isrelative_ = goal_->relative;
    mu_->state().kinova.isrelative = isrelative_;
      
    start_time_ = ros::Time::now();   

    if (goal_->target_poses.poses.size() != goal_->durations.data.size()){
      ROS_WARN_STREAM("The size of durations differs from the size of target poses");
      setAborted();
    }
    else{
      int target_size = goal_->target_poses.poses.size();
      mu_->state().kinova.H_ee_ref_array.resize(goal_->target_poses.poses.size());
      mu_->state().kinova.duration_array.resize(goal_->durations.data.size());

      for (int i=0; i<target_size; i++){
        Eigen::Vector3d pos(goal_->target_poses.poses[i].position.x, goal_->target_poses.poses[i].position.y, goal_->target_poses.poses[i].position.z);
        Eigen::Quaterniond quat(goal_->target_poses.poses[i].orientation.w, goal_->target_poses.poses[i].orientation.x, goal_->target_poses.poses[i].orientation.y, goal_->target_poses.poses[i].orientation.z);
        SE3 oMi_ref(quat.toRotationMatrix(), pos);
        mu_->state().kinova.H_ee_ref_array[i] = oMi_ref;
        mu_->state().kinova.duration_array[i] = goal_->durations.data[i];
      }
    }
   
    mu_->init_se3_array_ctrl(start_time_);

    control_running_ = true;  
}

void SE3ArrayActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool SE3ArrayActionServer::compute(ros::Time ctime)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 

  mu_->compute_se3_array_ctrl(ctime);

  if (ctime.toSec() - start_time_.toSec() > 1.0 + std::accumulate(mu_->state().kinova.duration_array.begin(), mu_->state().kinova.duration_array.end(), 0)){
      setSucceeded();
      return true;
    }

  if (ctime.toSec() - start_time_.toSec() > 5.0 + std::accumulate(mu_->state().kinova.duration_array.begin(), mu_->state().kinova.duration_array.end(), 0)){
    setAborted();
    return false;
  }
 
  return false;
}


void SE3ArrayActionServer::signalAbort(bool is_aborted)
{
  setAborted();  
}

void SE3ArrayActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  if (!mu_->simulation())
    mu_->done_se3_ctrl();
  control_running_ = false;
}
void SE3ArrayActionServer::setAborted()
{
  as_.setAborted();
  if (!mu_->simulation())
    mu_->done_se3_ctrl();
  control_running_ = false;
}
