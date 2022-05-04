#include <spot_kinova_framework/servers/qr_action_server.hpp>
#include <Eigen/Core>

using namespace Eigen;

QRActionServer::QRActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SpotKinovaWrapper> &mu)
: ActionServerBase(name,nh,mu), as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&QRActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&QRActionServer::preemptCallback, this));
  as_.start();

  ac_ = new actionlib::SimpleActionClient<spot_msgs::TrajectoryAction>("/spot/trajectory", false);
 
  qr_recieved_ = false;
}

void QRActionServer::goalCallback()
{
    feedback_header_stamp_ = 0;
    goal_ = as_.acceptNewGoal();
    
    string topic_name = goal_->topic_name;
    qr_subscriber_ = nh_.subscribe("/" + topic_name, 1, &QRActionServer::qrCallback, this);
    
    start_time_ = ros::Time::now();
    mode_change_ = true;
    control_running_ = true;  
}

void QRActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool QRActionServer::compute(ros::Time ctime)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 

  if (ctime.toSec() - start_time_.toSec() > 1.0 && !qr_recieved_){
    ROS_WARN_STREAM("QR is not recieved");
    setAborted();
    return false;
  }

  if (ctime.toSec() - start_time_.toSec() > 1.0 && qr_recieved_){
    if (mode_change_){
      spot_msgs::TrajectoryGoal goal;
      goal.target_pose.header.frame_id = "body";

      goal.duration.data.sec = 10.0;
      goal.precise_positioning = true;

      Quaterniond goal_quat, odom_quat;
      Vector3d goal_pos, odom_pos;

      tf::Quaternion qr_quat(qr_msg_.orientation.x,  qr_msg_.orientation.y, qr_msg_.orientation.z, qr_msg_.orientation.w);
      tf::Matrix3x3 m(qr_quat);
      double r, p, y;
      m.getRPY(r, p, y);
      tf::Quaternion res_quat;
      res_quat.setRPY(0, 0, y);

      goal_quat.x() = res_quat.getX();
      goal_quat.y() = res_quat.getY();
      goal_quat.z() = res_quat.getZ();
      goal_quat.w() = res_quat.getW();;
      
      goal_pos(0) = qr_msg_.position.x;
      goal_pos(1) = qr_msg_.position.y;
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
    
      ROS_WARN_STREAM(goal);  
      ac_->sendGoal(goal);
      mode_change_ = false;
    }
    if (ac_->getResult() ){
      setSucceeded();
    return true;
    }
  }
    

  if (ctime.toSec() - start_time_.toSec() > 20.0){
    setAborted();
    return false;
  }

  return false;
}


void QRActionServer::signalAbort(bool is_aborted)
{
  setAborted();  
}

void QRActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  mu_->done_se3_ctrl();
  control_running_ = false;
}
void QRActionServer::setAborted()
{
  as_.setAborted();
  mu_->done_se3_ctrl();
  control_running_ = false;
}
void QRActionServer::qrCallback(const geometry_msgs::Pose::ConstPtr& msg){
    qr_recieved_ = true;
    qr_msg_ = geometry_msgs::Pose();

    qr_msg_.position.x = msg->position.x;
    qr_msg_.position.y = msg->position.y;
    qr_msg_.position.z = msg->position.z;
    
    qr_msg_.orientation.x = msg->orientation.x;
    qr_msg_.orientation.y = msg->orientation.y;
    qr_msg_.orientation.z = msg->orientation.z;
    qr_msg_.orientation.w = msg->orientation.w;
    
}