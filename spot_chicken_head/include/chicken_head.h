
#ifndef CHICKEN_HEAD_H
#define CHICKEN_HEAD_H

//Pinocchio Header
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/centroidal.hpp>


#include "ros/ros.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <champ_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include "tf/transform_datatypes.h"
#include <Eigen/Dense>

using namespace pinocchio;
typedef pinocchio::Model Model;
typedef pinocchio::Data Data;

class ChickenHead
{
    using Vector3d = Eigen::Vector3d;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber cmd_pose_subscriber_;
    ros::Subscriber joint_state_subscriber_;
    ros::Subscriber body_state_subscriber_;

    ros::Publisher joint_state_publisher_;
    

    champ_msgs::Pose req_pose_;
    ros::Timer loop_timer_;
    ros::Time start_time_;

    float nominal_height_;

    Model model_;
    Data data_;
    Eigen::VectorXd q_, v_;
    Eigen::MatrixXd J_;
    SE3 oMi_;
    tf::Quaternion quat_;
    Vector3d x_;

    bool issimulation_;

    Vector3d rotate(const Vector3d, const float alpha, const float phi, const float beta);
    void cmdPoseCallback_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void controlLoop_(const ros::TimerEvent& event);
    void jointStateCallback_ (const sensor_msgs::JointState::ConstPtr& msg);
    void bodyStateCallback_ (const nav_msgs::Odometry::ConstPtr& msg);

    public:
        ChickenHead(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle);
};

#endif