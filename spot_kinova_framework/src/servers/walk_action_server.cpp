#include <spot_kinova_framework/servers/walk_action_server.hpp>

WalkActionServer::WalkActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SpotKinovaWrapper> &mu)
: ActionServerBase(name,nh,mu), as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&WalkActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&WalkActionServer::preemptCallback, this));
  as_.start();

  ac_ = new actionlib::SimpleActionClient<spot_msgs::TrajectoryAction>("/spot/trajectory", false);

  isrelative_ = true;
}

void WalkActionServer::goalCallback()
{
    feedback_header_stamp_ = 0;
    goal_ = as_.acceptNewGoal();
    
    mode_change_ = false;
    walk_done_ = false;
    
    isrelative_ = goal_->relative;
    // if (!mu_->state().spot.body_tilted)
    {
      spot_msgs::TrajectoryGoal goal;
      goal.target_pose.header.frame_id = "body";

      if (!isrelative_){        
        Quaterniond goal_quat, odom_quat;
        Vector3d goal_pos, odom_pos;
        goal_quat.x() = goal_->target_pose.orientation.x;
        goal_quat.y() = goal_->target_pose.orientation.y;
        goal_quat.z() = goal_->target_pose.orientation.z;
        goal_quat.w() = goal_->target_pose.orientation.w;
        
        goal_pos(0) = goal_->target_pose.position.x;
        goal_pos(1) = goal_->target_pose.position.y;
        goal_pos(2) = 0.0;

        SE3 goal_tf(goal_quat, goal_pos);

        odom_quat.x() = mu_->state().q(3);
        odom_quat.y() = mu_->state().q(4);
        odom_quat.z() = mu_->state().q(5);
        odom_quat.w() = mu_->state().q(6);
        
        odom_pos(0) =  mu_->state().q(0);
        odom_pos(1) =  mu_->state().q(1);
        odom_pos(2) =  0.0;//q_(2);

        SE3 odom_tf(odom_quat, odom_pos);
        SE3 action_tf_ = odom_tf.inverse() * goal_tf;
        
        goal.target_pose.pose.position.x = action_tf_.translation()(0);
        goal.target_pose.pose.position.y = action_tf_.translation()(1);
        goal.target_pose.pose.position.z = action_tf_.translation()(2);

        Quaterniond quat_tmp = Eigen::Quaterniond(action_tf_.rotation());
        goal.target_pose.pose.orientation.x = quat_tmp.x();
        goal.target_pose.pose.orientation.y = quat_tmp.y();
        goal.target_pose.pose.orientation.z = quat_tmp.z();
        goal.target_pose.pose.orientation.w = quat_tmp.w();

        goal.duration.data.sec = 10.0;
        goal.precise_positioning = true;
        ac_->sendGoal(goal);
      }
      else{
       
        goal.target_pose.pose.position.x = goal_->target_pose.position.x;
        goal.target_pose.pose.position.y = goal_->target_pose.position.y;
        goal.target_pose.pose.position.z = goal_->target_pose.position.z;


        goal.target_pose.pose.orientation.x = goal_->target_pose.orientation.x;
        goal.target_pose.pose.orientation.y = goal_->target_pose.orientation.y;
        goal.target_pose.pose.orientation.z = goal_->target_pose.orientation.z;
        goal.target_pose.pose.orientation.w = goal_->target_pose.orientation.w;

        goal.duration.data.sec = 10.0;
        goal.precise_positioning = true;
        ac_->sendGoal(goal);
      }
    }
    start_time_ = ros::Time::now();

    control_running_ = true;  
}

void WalkActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool WalkActionServer::compute(ros::Time ctime)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 

  // if (!mu_->state().spot.body_tilted){    
  //   ROS_WARN_STREAM("Body is tilted. Cannot walk for safety.");
  //   setAborted();
  //   return false;
  // }
  
  if (ac_->getResult() ){
    setSucceeded();
    return true;
  }

  if (ctime.toSec() - start_time_.toSec() > 20.0){
    setAborted();
    return false;
  }

  return false;
}


void WalkActionServer::signalAbort(bool is_aborted)
{
  setAborted();  
}

void WalkActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  if (!mu_->simulation())
    mu_->done_se3_ctrl();
  control_running_ = false;
}
void WalkActionServer::setAborted()
{
  as_.setAborted();
  if (!mu_->simulation())
    mu_->done_se3_ctrl();
  control_running_ = false;
}
void WalkActionServer::movebaseCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
    if (msg->status.status == 3)
      walk_done_ = true;
    else
      walk_done_ = false;
}
