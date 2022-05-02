#include "spot_kinova_framework/simulation/simulation.hpp"

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
    n_node.getParam("/robot_group", group_name);    
    ctrl_ = std::make_shared<RobotController::SpotKinovaWrapper>(group_name, true, n_node);
    ctrl_->initialize();

    // Ros subscribe
    cmd_pose_subscriber_ = n_node.subscribe("base_to_footprint_pose", 1, &cmdPoseCallback);
    body_state_subscriber_ = n_node.subscribe("odom", 1, &bodyStateCallback);
    move_base_subscriber_ = n_node.subscribe("move_base/result", 1, &movebaseCallback);

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
    
    body_pose_publisher_ = n_node.advertise<geometry_msgs::Pose>("body_pose", 100);

    // Action Server
    joint_posture_action_server_ = std::make_unique<JointPostureActionServer>("/spot_kinova_action/joint_posture_control", n_node, ctrl_);
    se3_action_server_ = std::make_unique<SE3ActionServer>("/spot_kinova_action/se3_control", n_node, ctrl_);
    walk_action_server_ = std::make_unique<WalkSimulationActionServer>("/spot_kinova_action/move_base", n_node, ctrl_);
    body_posture_action_server_ = std::make_unique<BodyPostureActionServer>("/spot_kinova_action/body_posture_control", n_node, ctrl_);
    wholebody_action_server_ = std::make_unique<WholebodyActionServer>("/spot_kinova_action/wholebody_control", n_node, ctrl_);

    // Variable Initialize
    time_ = 0.0;

    while (ros::ok()){
        keyboard_event();
        ctrl_->kinova_update();

        joint_posture_action_server_->compute(ros::Time::now());
        se3_action_server_->compute(ros::Time::now());
        walk_action_server_->compute(ros::Time::now());
        body_posture_action_server_->compute(ros::Time::now());
        wholebody_action_server_->compute(ros::Time::now());

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

void cmdPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;
    
    quat1_ = tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
}

void bodyStateCallback(const nav_msgs::Odometry::ConstPtr& msg){
    Vector3d x;
    x(0) = msg->pose.pose.position.x + odom_pos_(0);
    x(1) = msg->pose.pose.position.y + odom_pos_(1);
    x(2) = msg->pose.pose.position.z + odom_pos_(2);

    quat2_ = tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Quaternion quat_res;   
    quat_res = quat2_ * quat1_;

    ctrl_->spot_update(x, quat_res, quat1_, quat2_);
}

void movebaseCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
    if (msg->status.status == 3)
        ctrl_->ctrl_update(2);
}

void keyboard_event(){
    if (_kbhit()){
        int key;
        key = getchar();
        int msg = 0;
        switch (key){
            case 'h': { //home
                msg = 0;
                spot_kinova_msgs::JointPostureGoal goal;
                goal.target_joints.position.resize(7);
                goal.target_joints.position[0] = 0.0;
                goal.target_joints.position[1] = -40.0 / 180. * M_PI;
                goal.target_joints.position[2] = 3.14;
                goal.target_joints.position[3] = -100 / 180. * M_PI;
                goal.target_joints.position[4] = 0.0;
                goal.target_joints.position[5] = 60. / 180. * M_PI;
                goal.target_joints.position[6] = 1.57;
                
                goal.duration = 2.0f;

                cout << " " << endl;
                cout << "Move to Home Posture" << endl;
                cout << " " << endl;
                break;
            }
            case 'w': //home
                msg = 1;
                ctrl_->ctrl_update(msg);
                
                cout << " " << endl;
                cout << "Move to Walking Posture" << endl;
                cout << " " << endl;
                break;
            
            // case 'g': // go
            //     msg = 2;
            //     ctrl_->ctrl_update(msg);
                
            //     cout << " " << endl;
            //     cout << "Move to Walking Posture" << endl;
            //     cout << " " << endl;
            //     break;
        }
    }
}
