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
#include <tf/transform_listener.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
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
#include <spot_kinova_framework/servers/qr_walk_action_server.hpp>
#include <spot_kinova_framework/servers/gripper_action_server.hpp>
#include <spot_kinova_framework/servers/qr_pick_action_server.hpp>
#include <spot_kinova_framework/servers/se3_array_action_server.hpp>
#include "spot_msgs/MobilityParams.h"

using namespace Eigen;

double time_;
std::shared_ptr<RobotController::SpotKinovaWrapper> ctrl_;

ros::Publisher wrench_publisher_;
ros::Publisher joint_state_publisher_;
ros::Publisher body_pose_publisher_;
geometry_msgs::Wrench wrench_msg_;
sensor_msgs::JointState joint_msg_;
geometry_msgs::Pose pose_msg_;

ros::Subscriber cmd_pose_subscriber_;
ros::Subscriber body_state_subscriber_;

Vector3d odom_pos_;
tf::Quaternion quat1_, quat2_, body_quat_, quat_res_;
Vector3d x_, x_prev_, x_lpf_;

SE3 action_walk_tf_;
bool start_flag_{true};

std::unique_ptr<JointPostureActionServer> joint_posture_action_server_;
std::unique_ptr<SE3ActionServer> se3_action_server_;
std::unique_ptr<WalkActionServer> walk_action_server_;
std::unique_ptr<BodyPostureActionServer> body_posture_action_server_;
std::unique_ptr<WholebodyActionServer> wholebody_action_server_;
std::unique_ptr<PredefinedPostureActionServer> predefined_posture_action_server_;
std::unique_ptr<QRWalkActionServer> qr_walk_action_server_;
std::unique_ptr<GripperActionServer> gripper_action_server_;
std::unique_ptr<QRPickActionServer> qr_pick_action_server_;
std::unique_ptr<SE3ArrayActionServer> se3_array_action_server_;


void bodyStateCallback(const nav_msgs::Odometry::ConstPtr& msg);
void cmdPoseCallback(const spot_msgs::MobilityParams::ConstPtr& msg);
void publishJointState();
void publishBodyPose();
double lpf(double time, double y, double y_last, double cutoff){
    double gain = time / (time + (1.0 / (2.0 * 3.141592 * cutoff)));
    return gain * y + (1-gain) * y_last;
};



