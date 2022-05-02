// Controller
#include "spot_kinova_framework/controller/controller.hpp"

// Ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Transform.h"
#include "tf/transform_datatypes.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <champ_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

//SYSTEM Header
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

//SpotKinovaAction
#include <spot_kinova_framework/servers/joint_posture_action_server.hpp>
#include <spot_kinova_framework/servers/se3_action_server.hpp>
#include <spot_kinova_framework/servers/walk_action_server.hpp>
#include <spot_kinova_framework/servers/body_posture_action_server.hpp>
#include <spot_kinova_framework/servers/wholebody_action_server.hpp>
#include <spot_kinova_framework/servers/predefined_posture_action_server.hpp>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "actionlib/client/simple_goal_state.h"
#include "actionlib/client/simple_client_goal_state.h"
#include "actionlib/client/terminal_state.h"
#include "spot_msgs/TrajectoryAction.h"

using namespace Eigen;

double time_;
std::shared_ptr<RobotController::SpotKinovaWrapper> ctrl_;

ros::Publisher joint_state_publisher_;
ros::Publisher body_pose_publisher_;
sensor_msgs::JointState joint_msg_;
geometry_msgs::Pose pose_msg_;
actionlib::SimpleActionClient<spot_msgs::TrajectoryAction>* ac_;

ros::Subscriber cmd_pose_subscriber_;
ros::Subscriber body_state_subscriber_;
ros::Subscriber nav_goal_subscriber_;

Vector3d odom_pos_;
tf::Quaternion quat1_, quat2_, body_quat_, quat_res_;
Vector3d x_;
SE3 action_walk_tf_;

std::unique_ptr<JointPostureActionServer> joint_posture_action_server_;
std::unique_ptr<SE3ActionServer> se3_action_server_;
std::unique_ptr<WalkActionServer> walk_action_server_;
std::unique_ptr<BodyPostureActionServer> body_posture_action_server_;
std::unique_ptr<WholebodyActionServer> wholebody_action_server_;
std::unique_ptr<PredefinedPostureActionServer> predefined_posture_action_server_;

void cmdPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void bodyStateCallback(const nav_msgs::Odometry::ConstPtr& msg);
void NavGollCallback_(const geometry_msgs::PoseStamped::ConstPtr& msg);
void publishJointState();
void publishBodyPose();



