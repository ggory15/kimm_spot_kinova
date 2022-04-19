#include <chicken_head.h>
#include <math_helper.h>
#include <stdio.h>
#include <Eigen/QR>  
#include <pinocchio/algorithm/joint-configuration.hpp> 

using namespace std;
using namespace pinocchio;
using namespace Eigen;

ChickenHead::ChickenHead(const ros::NodeHandle &node_handle,
                         const ros::NodeHandle &private_node_handle):
    nh_(node_handle),
    pnh_(private_node_handle)
{
    // Ros Param
    string urdf_name, urdf_path;
    nh_.getParam("urdf_path", urdf_path);
    nh_.getParam("urdf_name", urdf_name);
    start_time_ = ros::Time::now();
    

    // Pinocchio Model
    vector<string> package_dirs;
    package_dirs.push_back(urdf_path);
    std::string urdfFileName = package_dirs[0] + "/" + urdf_name;
    pinocchio::urdf::buildModel(urdfFileName, pinocchio::JointModelFreeFlyer(), model_, false);
    Data data(model_);
    data_ = data;

    q_.setZero(26);
    v_.setZero(25);
    J_.setZero(6, 7);
    x_.setZero();
    x_(2) = 0.48;

    q_(6) = 1.0;

    joint_state_publisher_ = nh_.advertise<sensor_msgs::JointState>("arm/joint_states", 100);
    cmd_pose_subscriber_ = nh_.subscribe("base_to_footprint_pose", 1, &ChickenHead::cmdPoseCallback_, this);
    joint_state_subscriber_ = nh_.subscribe("joint_states", 1,  &ChickenHead::jointStateCallback_, this);
    body_state_subscriber_ = nh_.subscribe("odom", 1, &ChickenHead::bodyStateCallback_, this);

    nh_.getParam("gait/nominal_height", nominal_height_);

    loop_timer_ = pnh_.createTimer(ros::Duration(0.001),
                                   &ChickenHead::controlLoop_,
                                   this);

    req_pose_.roll = 0.0;
    req_pose_.pitch = 0.0;
    req_pose_.yaw = 0.0;
    req_pose_.z = nominal_height_;
}

Eigen::Vector3d ChickenHead::rotate(const Vector3d pos, const float alpha, const float phi, const float beta)
{
    Eigen::Matrix3d Rx;
    Eigen::Matrix3d Ry;
    Eigen::Matrix3d Rz;
    Vector3d xformed_pos;

    Rz << cos(beta), -sin(beta), 0, sin(beta), cos(beta), 0, 0, 0, 1;
    xformed_pos = (Rz * pos).eval();

    Ry << cos(phi), 0, sin(phi), 0, 1, 0, -sin(phi), 0, cos(phi);
    xformed_pos = (Ry * xformed_pos).eval();

    Rx << 1, 0, 0, 0, cos(alpha), -sin(alpha), 0, sin(alpha),  cos(alpha);
    xformed_pos = (Rx * xformed_pos).eval();

    return xformed_pos;
}

void ChickenHead::controlLoop_(const ros::TimerEvent& event)
{
    // Vector3d target_pos = rotate(initial_pos_, -req_pose_.roll, -req_pose_.pitch, 0);
    // target_pos[2] += nominal_height_ - req_pose_.z;
    // target_pos -= base_to_lower_arm_;

    // Vector3d temp_pos = rotate(initial_pos_, -req_pose_.roll, -req_pose_.pitch, -req_pose_.yaw);
    // temp_pos -= wrist1_to_wrist2_;

    // float base_joint = atan2(temp_pos[1], temp_pos[0]);

    // float x = target_pos[0];
    // float y = target_pos[2];

    // float upper_joint = acos((pow(x, 2) + pow(y, 2) - pow(l1_, 2) - pow(l2_, 2)) / (2 * l1_ * l2_));
    // float lower_joint = -atan(y / x) - atan((l2_ * sin(upper_joint)) / (l1_ + (l2_ * cos(upper_joint))));

    // float alpha = M_PI - upper_joint;
    // float beta = M_PI - abs(alpha) - abs(lower_joint);

    // float wrist1_joint = -beta - req_pose_.pitch;
    // float wrist2_joint = -req_pose_.roll;

    std::vector<std::string> joint_names;
    joint_names.push_back("joint_1");
    joint_names.push_back("joint_2");
    joint_names.push_back("joint_3");
    joint_names.push_back("joint_4");
    joint_names.push_back("joint_5");
    joint_names.push_back("joint_6");
    joint_names.push_back("joint_7");

    sensor_msgs::JointState joint_states;
        
    
    if (ros::Time::now().toSec() - start_time_.toSec() < 3.0){
        joint_states.header.stamp = ros::Time::now();
        joint_states.name.resize(joint_names.size());
        joint_states.position.resize(joint_names.size());
        joint_states.name = joint_names;
        
        joint_states.position[0]= 0.0;
        joint_states.position[1]= -2.0944;
        joint_states.position[2]= 3.14;
        joint_states.position[3]= -2.443;
        joint_states.position[4]= 0.0;
        joint_states.position[5]= 0.3490;
        joint_states.position[6]= 1.57;
        
        for (size_t i = 0; i < joint_names.size(); ++i)
        {   
            q_(i+7) = joint_states.position[i];

            if(isnan(joint_states.position[i]))
            {
                return;
            }
        }
        
        pinocchio::computeAllTerms(model_, data_, q_, v_);
    
        joint_state_publisher_.publish(joint_states);
        oMi_ = data_.oMi[model_.getJointId("joint_7")];
    }
    else{
        pinocchio::computeAllTerms(model_, data_, q_, v_);

        Data::Matrix6x J_tmp(6, 25);
        pinocchio::getJointJacobian(model_, data_, model_.getJointId("joint_7"), pinocchio::LOCAL, J_tmp);
        J_ = J_tmp.block(0, 6, 6, 7);
        
        SE3 M_ref = oMi_;
        SE3 oMi = data_.oMi[model_.getJointId("joint_7")];
        SE3 wMl;
        wMl.setIdentity();
        Motion v_frame = data_.v[model_.getJointId("joint_7")];
        Motion p_error, v_ref, v_error;
        VectorXd p_ref, p, p_error_vec, v_error_vec, v;
        p_ref.setZero(12);
        p.setZero(12);

        errorInSE3(oMi, M_ref, p_error);
        SE3ToVector(M_ref, p_ref);
        SE3ToVector(oMi, p);
        
        wMl.rotation(oMi.rotation());
        p_error_vec = p_error.toVector();
        v_error =  wMl.actInv(v_ref) - v_frame;

        
        VectorXd xdot_des = p_error_vec * 1000.0 - v_frame.toVector();// + v_error.toVector(); 

        Eigen::MatrixXd J_inv = J_.completeOrthogonalDecomposition().pseudoInverse();
        VectorXd v_arm_des = J_inv * xdot_des;
        VectorXd v_des(25);
        v_des.segment(6,7) = v_arm_des;

        q_ = pinocchio::integrate(model_, q_, 0.001 * v_des);

        joint_states.header.stamp = ros::Time::now();
        joint_states.name.resize(joint_names.size());
        joint_states.position.resize(joint_names.size());
        joint_states.name = joint_names;      
        
        for (size_t i = 0; i < joint_names.size(); ++i)
        {   
           joint_states.position[i]= q_(i+7);
        }
        joint_state_publisher_.publish(joint_states);
    }
   
    //ROS_WARN_STREAM(q_);
    //ROS_WARN_STREAM(data_.oMi[model_.getJointId("joint_1")]);
}

void ChickenHead::cmdPoseCallback_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    quat_[0] = msg->pose.pose.orientation.x;
    quat_[1] = msg->pose.pose.orientation.y;
    quat_[2] = msg->pose.pose.orientation.z;
    quat_[3] = msg->pose.pose.orientation.w;

    x_(0) = msg->pose.pose.position.x;
    x_(1) = msg->pose.pose.position.y;
    x_(2) = msg->pose.pose.position.z;
}
void ChickenHead::jointStateCallback_ (const sensor_msgs::JointState::ConstPtr& msg){
    for (int i=0; i<12; i++)
        q_(i+14) = msg->position[i];
}
void ChickenHead::bodyStateCallback_ (const nav_msgs::Odometry::ConstPtr& msg){
    q_(0) = msg->pose.pose.position.x + x_(0);
    q_(1) = msg->pose.pose.position.y + x_(1);
    q_(2) = msg->pose.pose.position.z + x_(2);
    
    tf::Quaternion quat2(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Quaternion quat_res;

    quat_res = quat2 * quat_;
    q_(3) = quat_res.getX();
    q_(4) = quat_res.getY();
    q_(5) = quat_res.getZ();
    q_(6) = quat_res.getW();
}