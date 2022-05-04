#include "spot_kinova_framework/real/real.hpp"

using namespace std;
using namespace pinocchio;
using namespace Eigen;
using namespace RobotController;

int main(int argc, char **argv)
{  
    //Ros setting
    ros::init(argc, argv, "spot_kinova_controller");
    ros::NodeHandle n_node;
    ros::Rate loop_rate(1000);

    // Robot Wapper
    string group_name;
    bool issimulation;
    n_node.getParam("/robot_group", group_name);    
    n_node.getParam("/issimulation", issimulation);    
    ctrl_ = std::make_shared<RobotController::SpotKinovaWrapper>(group_name, issimulation, n_node);
    ctrl_->initialize();

    // Ros subscribe
    body_state_subscriber_ = n_node.subscribe("/spot/odometry", 1, &bodyStateCallback);
    // nav_goal_subscriber_ = n_node.subscribe("/move_base_simple/goal", 1, &NavGollCallback_);
    cmd_pose_subscriber_ = n_node.subscribe("/spot/status/mobility_params", 1, &cmdPoseCallback);

    // Ros publish
    joint_state_publisher_ = n_node.advertise<sensor_msgs::JointState>("arm/joint_states", 100);   
    joint_msg_.name.resize(7);
    joint_msg_.position.resize(7);
    joint_msg_.velocity.resize(7);
    std::vector<std::string> joint_names;
    joint_names.push_back("joint_1");
    joint_names.push_back("joint_2");
    joint_names.push_back("joint_3");
    joint_names.push_back("joint_4");
    joint_names.push_back("joint_5");
    joint_names.push_back("joint_6");
    joint_names.push_back("joint_7");
    joint_msg_.name = joint_names;
    
    body_pose_publisher_ = n_node.advertise<geometry_msgs::Pose>("/spot/body_pose", 100);

    // Action Server
    joint_posture_action_server_ = std::make_unique<JointPostureActionServer>("/spot_kinova_action/joint_posture_control", n_node, ctrl_);
    se3_action_server_ = std::make_unique<SE3ActionServer>("/spot_kinova_action/se3_control", n_node, ctrl_);
    walk_action_server_ = std::make_unique<WalkActionServer>("/spot_kinova_action/move_base", n_node, ctrl_);
    body_posture_action_server_ = std::make_unique<BodyPostureActionServer>("/spot_kinova_action/body_posture_control", n_node, ctrl_);
    wholebody_action_server_ = std::make_unique<WholebodyActionServer>("/spot_kinova_action/wholebody_control", n_node, ctrl_);
    predefined_posture_action_server_ = std::make_unique<PredefinedPostureActionServer>("/spot_kinova_action/predefined_posture_control", n_node, ctrl_);
    qr_action_server_ = std::make_unique<QRActionServer>("/spot_kinova_action/qr_control", n_node, ctrl_);
    gripper_action_server_ = std::make_unique<GripperActionServer>("/spot_kinova_action/gripper_control", n_node, ctrl_);
    // ac_ = new actionlib::SimpleActionClient<spot_msgs::TrajectoryAction>("/spot/trajectory", false);

    // Variable Initialize
    time_ = 0.0;

    while (ros::ok()){
        ctrl_->kinova_update();

        joint_posture_action_server_->compute(ros::Time::now());
        se3_action_server_->compute(ros::Time::now());
        walk_action_server_->compute(ros::Time::now());
        body_posture_action_server_->compute(ros::Time::now());
        wholebody_action_server_->compute(ros::Time::now());
        predefined_posture_action_server_->compute(ros::Time::now());
        qr_action_server_->compute(ros::Time::now());
        gripper_action_server_->compute(ros::Time::now());
        
        publishJointState();
        publishBodyPose();

        time_ += 0.001;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void publishJointState(){
    Vector7d q = ctrl_->state().kinova.q;
    Vector7d v = ctrl_->state().kinova.v;
    joint_msg_.header.stamp = ros::Time::now();
        
    for (int i=0; i<7; i++){
        joint_msg_.position[i] = q(i);
        joint_msg_.velocity[i] = v(i);
    }
    joint_state_publisher_.publish(joint_msg_);
}

void publishBodyPose(){
    if (ctrl_->state().spot.orientation_publish){
        pose_msg_.orientation.x = ctrl_->state().spot.orientation_des.getX();
        pose_msg_.orientation.y = ctrl_->state().spot.orientation_des.getY();
        pose_msg_.orientation.z = ctrl_->state().spot.orientation_des.getZ();
        pose_msg_.orientation.w = ctrl_->state().spot.orientation_des.getW();

        body_pose_publisher_.publish(pose_msg_);
    }
}

void bodyStateCallback(const nav_msgs::Odometry::ConstPtr& msg){
    
    x_(0) = msg->pose.pose.position.x;
    x_(1) = msg->pose.pose.position.y;
    x_(2) = msg->pose.pose.position.z;
 
    quat_res_ = tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    ctrl_->spot_update(x_, quat_res_, quat1_, quat2_); // todo: quat_res (global quaternion), quat1 (global nav2d), quat2 (body pose)
}
void cmdPoseCallback(const spot_msgs::MobilityParams::ConstPtr& msg){
    quat1_ = tf::Quaternion(msg->body_control.orientation.x, msg->body_control.orientation.y, msg->body_control.orientation.z, msg->body_control.orientation.w);
}
void NavGollCallback_(const geometry_msgs::PoseStamped::ConstPtr& msg){
    // Quaterniond goal_quat, odom_quat;
    // Vector3d goal_pos, odom_pos;
    // goal_quat.x() = msg->pose.orientation.x;
    // goal_quat.y() = msg->pose.orientation.y;
    // goal_quat.z() = msg->pose.orientation.z;
    // goal_quat.w() = msg->pose.orientation.w;
    
    // goal_pos(0) =  msg->pose.position.x;
    // goal_pos(1) =  msg->pose.position.y;
    // goal_pos(2) =  msg->pose.position.z;

    // SE3 goal_tf(goal_quat, goal_pos);

    // odom_quat.x() = quat_res_.getX();
    // odom_quat.y() = quat_res_.getY();
    // odom_quat.z() = quat_res_.getZ();
    // odom_quat.w() = quat_res_.getW();
    
    // odom_pos(0) =  x_(0);
    // odom_pos(1) =  x_(1);
    // odom_pos(2) =  0.0;//q_(2);

    // SE3 odom_tf(odom_quat, odom_pos);
    // action_walk_tf_ = odom_tf.inverse() * goal_tf;
    
    // ROS_WARN_STREAM("Goal's Reletive Position is");
    // ROS_WARN_STREAM(action_walk_tf_);

    // if (ctrl_->state().spot.body_tilted)
    //     ROS_WARN_STREAM("body is tilted");
    // else{
    //     spot_msgs::TrajectoryGoal goal;
    //     goal.target_pose.header.frame_id = "body";
    //     goal.target_pose.pose.position.x = action_walk_tf_.translation()(0);
    //     goal.target_pose.pose.position.y = action_walk_tf_.translation()(1);
    //     goal.target_pose.pose.position.z = action_walk_tf_.translation()(2);

    //     Quaterniond quat_tmp = Eigen::Quaterniond(action_walk_tf_.rotation());
    //     goal.target_pose.pose.orientation.x = quat_tmp.x();
    //     goal.target_pose.pose.orientation.y = quat_tmp.y();
    //     goal.target_pose.pose.orientation.z = quat_tmp.z();
    //     goal.target_pose.pose.orientation.w = quat_tmp.w();

    //     goal.duration.data.sec = 10.0;
    //     goal.precise_positioning = true;
    //     ROS_WARN_STREAM("Action Sent");
    //     ac_->sendGoal(goal);            
    //     ROS_WARN_STREAM("Action Activated:");
    // }
}
