// KIMM_Spot_Kinova
#include <kimm_spot_kinova/robots/robot-wrapper.hpp>
#include <kimm_spot_kinova/math/fwd.hpp>
#include <kimm_spot_kinova/formulation/inverse_dynamics_formulation_vel.hpp>
#include <kimm_spot_kinova/tasks/task_joint_posture.hpp>
#include <kimm_spot_kinova/tasks/task_se3_equality.hpp>
#include <kimm_spot_kinova/solver/solver_HQP_eiquadprog.hpp>
#include <kimm_spot_kinova/trajectories/trajectory_euclidian.hpp>
#include <kimm_spot_kinova/trajectories/trajectory_se3.hpp>
#include <kimm_spot_kinova/solver/solver_HQP_factory.hxx>
#include <kimm_spot_kinova/solver/util.hpp>

// pinocchio
#include <pinocchio/algorithm/joint-configuration.hpp>

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
#include <champ_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

//SYSTEM Header
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

using namespace spotkinova;
using namespace spotkinova::robots;
using namespace spotkinova::math;
using namespace spotkinova::trajectory;
using namespace spotkinova::solver;
using namespace spotkinova::tasks;

using namespace std;
using namespace pinocchio;
using namespace Eigen;

bool kinova_only, use_joy;
string urdf_path, urdf_name;

// hqp
std::shared_ptr<spotkinova::robots::RobotWrapper> robot;
std::shared_ptr<spotkinova::InverseDynamicsFormulationAccForce> tsid;
std::shared_ptr<spotkinova::tasks::TaskJointPosture> postureTask;
std::shared_ptr<spotkinova::tasks::TaskSE3Equality> eeTask;
std::shared_ptr<spotkinova::trajectory::TrajectoryEuclidianCubic> trajPosture;
std::shared_ptr<spotkinova::trajectory::TrajectorySE3Cubic> trajEE;

// ros
ros::Publisher joint_state_publisher;
ros::Publisher body_state_publisher;
sensor_msgs::JointState joint_msg;
geometry_msgs::Pose pose_msg;
ros::Subscriber cmd_pose_subscriber;
ros::Subscriber body_state_subscriber;

// state
VectorXd q, arm_q;
VectorXd v, arm_v;
MatrixXd J;
VectorXd q_ref, v_ref;
Vector3 odom_pos;
SE3 oMi;
Model model;
Data data;
double cur_time;
tf::Quaternion quat1, quat2, body_quat;
Vector3 body_angles, body_angles_cubic, body_angles_target;

// callback
void cmdPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void bodyStateCallback(const nav_msgs::Odometry::ConstPtr& msg);

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "spot_kinova_unittest");
    ros::NodeHandle n_node;
    ros::Rate loop_rate(1000);

    n_node.getParam("urdf_path", urdf_path);
    n_node.getParam("urdf_name", urdf_name);
    n_node.getParam("kinova_only", kinova_only);
    n_node.getParam("use_joy", use_joy);

    vector<string> package_dirs;
    package_dirs.push_back(urdf_path);
    std::string urdfFileName = package_dirs[0] + "/" + urdf_name;

    if (kinova_only){
        robot = std::make_shared<RobotWrapper>(urdfFileName, package_dirs, false);
        joint_state_publisher = n_node.advertise<sensor_msgs::JointState>("/arm/joint_states", 100);   
        joint_msg.name.resize(7);
        joint_msg.position.resize(7);

        std::vector<std::string> joint_names;
        joint_names.push_back("joint_1");
        joint_names.push_back("joint_2");
        joint_names.push_back("joint_3");
        joint_names.push_back("joint_4");
        joint_names.push_back("joint_5");
        joint_names.push_back("joint_6");
        joint_names.push_back("joint_7");
        joint_msg.name = joint_names;

        q.setZero(7);
        v.setZero(7);
        J.setZero(6, 7);

        arm_q = q;
        arm_v = v;
        q_ref.resize(7);
        v_ref.resize(7);
    }
    else{
        robot = std::make_shared<RobotWrapper>(urdfFileName, package_dirs, pinocchio::JointModelFreeFlyer(), false);
        joint_state_publisher = n_node.advertise<sensor_msgs::JointState>("/arm/joint_states", 100);   
        joint_msg.name.resize(7);
        joint_msg.position.resize(7);

        body_state_publisher = n_node.advertise<geometry_msgs::Pose>("/body_pose", 100); 

        cmd_pose_subscriber = n_node.subscribe("base_to_footprint_pose", 1, &cmdPoseCallback);
        body_state_subscriber = n_node.subscribe("odom", 1, &bodyStateCallback);

        std::vector<std::string> joint_names;
        joint_names.push_back("joint_1");
        joint_names.push_back("joint_2");
        joint_names.push_back("joint_3");
        joint_names.push_back("joint_4");
        joint_names.push_back("joint_5");
        joint_names.push_back("joint_6");
        joint_names.push_back("joint_7");
        joint_msg.name = joint_names;

        q.setZero(26);
        q(6) = 1.0;
        v.setZero(25);
        J.setZero(6, 7);

        arm_q = q.segment(7, 7);
        arm_v = v.segment(6, 7);
        q_ref.resize(7);
        v_ref.resize(6);
    }
    tsid = std::make_shared<InverseDynamicsFormulationAccForce>("tsid", *robot);
    tsid->computeProblemData(cur_time, q, v);
    data = tsid->data();

    postureTask = std::make_shared<TaskJointPosture>("task-posture", *robot);
    postureTask->Kp(Vector7::Ones() * 10.0);
    postureTask->Kd(2.0*postureTask->Kp().cwiseSqrt());

    eeTask = std::make_shared<TaskSE3Equality>("task-se3", *robot, "joint_7", Vector3d(0,0,0));
    eeTask->Kp(Vector::Ones(6) * 10.);
    eeTask->Kd(2.0*eeTask->Kp().cwiseSqrt());

    TrajectorySample sampleEE, samplePosture;
    sampleEE.resize(12, 6);
    samplePosture.resize(7);
    trajPosture = std::make_shared<TrajectoryEuclidianCubic>("traj_posture");
    trajEE = std::make_shared<TrajectorySE3Cubic>("traj_ee");

    auto solver = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG, "quadprog");    

    double transition_time = 4.0;
    cur_time = 0.0;
    auto start_time = ros::Time::now().toSec();   

    bool step1_flag = false;
    bool step2_flag = false;
    bool finish_flag = false;

    while (ros::ok()){
        cur_time = ros::Time::now().toSec() - start_time;
        robot->computeAllTerms(data, q, v);

        if (cur_time  < transition_time && cur_time > 1.0){
            if (!step1_flag){
                tsid->addMotionTask(*postureTask, 1e-5, 0);
                q_ref(0) = 0.0;
                q_ref(1) = -2.0944;
                q_ref(2) = 3.14;
                q_ref(3) = -2.443;
                q_ref(4) = 0.0;
                q_ref(5) = 0.3490;
                q_ref(6) = 1.57;

                trajPosture->setInitSample(arm_q);
                trajPosture->setDuration(2.0);
                trajPosture->setStartTime(cur_time);
                trajPosture->setGoalSample(q_ref);   
                
            
                step1_flag = true;
            }
            trajPosture->setCurrentTime(cur_time);
            samplePosture = trajPosture->computeNext();
            postureTask->setReference(samplePosture);
           
            const HQPData & HQPData = tsid->computeProblemData(cur_time, q, v);       
            v_ref = tsid->getAccelerations(solver->solve(HQPData));

            if (!kinova_only){
                arm_q = pinocchio::integrate(robot->model(), q, 0.001 * v_ref).segment(7,7);
                q.segment(7, 7) = arm_q;
            }
            else{
                arm_q = pinocchio::integrate(robot->model(), q, 0.001 * v_ref);
                q = arm_q;   
            }
        } 
        if (cur_time >= transition_time){
            if (!step2_flag){
                tsid->removeTask("task-posture");
                tsid->addMotionTask(*postureTask, 1e-5, 1);
                tsid->addMotionTask(*eeTask, 1, 0);
                q_ref(0) = 0.0;
                q_ref(1) = -2.0944;
                q_ref(2) = 3.14;
                q_ref(3) = -2.443;
                q_ref(4) = 0.0;
                q_ref(5) = 0.3490;
                q_ref(6) = 1.57;

                trajPosture->setInitSample(arm_q);
                trajPosture->setDuration(0.1);
                trajPosture->setStartTime(cur_time);
                trajPosture->setGoalSample(q_ref);   

                trajEE->setStartTime(cur_time);
                trajEE->setDuration(2.0);
                SE3 H_ee_ref = robot->position(data, robot->model().getJointId("joint_7"));
                trajEE->setInitSample(H_ee_ref);
                H_ee_ref.translation()(0) += 0.15;
                H_ee_ref.translation()(1) += 0.15;                
                trajEE->setGoalSample(H_ee_ref);

                body_angles.setZero();// assume that body pose is zero
                body_angles_target(0) = 0.4; // roll
                body_angles_target(1) = 0.3; // pitch
                body_angles_target(2) = 0.2; // yaw              
    
                step2_flag = true;
            }

            trajPosture->setCurrentTime(cur_time);
            samplePosture = trajPosture->computeNext();
            postureTask->setReference(samplePosture);

            trajEE->setCurrentTime(cur_time);
            sampleEE = trajEE->computeNext();
            eeTask->setReference(sampleEE);
           
            const HQPData & HQPData = tsid->computeProblemData(cur_time, q, v);       
            v_ref = tsid->getAccelerations(solver->solve(HQPData));

            if (!kinova_only){
                arm_q = pinocchio::integrate(robot->model(), q, 0.001 * v_ref).segment(7,7);
                q.segment(7, 7) = arm_q;

                if (!use_joy){
                    tf::Quaternion quat_body;
                    for (int i=0; i<3; i++)
                        body_angles_cubic(i) = cubic(cur_time, transition_time, transition_time + 2.0, body_angles(i), body_angles_target(i), 0., 0.);
                    quat_body.setRPY(body_angles_cubic(0), body_angles_cubic(1), body_angles_cubic(2));

                    pose_msg.orientation.x = quat_body.getX();
                    pose_msg.orientation.y = quat_body.getY();
                    pose_msg.orientation.z = quat_body.getZ();
                    pose_msg.orientation.w = quat_body.getW();

                    if (cur_time < transition_time + 2.0)
                        body_state_publisher.publish(pose_msg);
                }
            }   
            else{
                arm_q = pinocchio::integrate(robot->model(), q, 0.001 * v_ref);
                q = arm_q;   
            }
        }

        // joint publish
        joint_msg.header.stamp = ros::Time::now();
        for (int i=0; i<7; i++)
            joint_msg.position[i] = arm_q(i);
        joint_state_publisher.publish(joint_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void cmdPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    odom_pos(0) = msg->pose.pose.position.x;
    odom_pos(1) = msg->pose.pose.position.y;
    odom_pos(2) = msg->pose.pose.position.z;
    
    quat1 = tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
}

void bodyStateCallback(const nav_msgs::Odometry::ConstPtr& msg){
    q(0) = msg->pose.pose.position.x + odom_pos(0);
    q(1) = msg->pose.pose.position.y + odom_pos(1);
    q(2) = msg->pose.pose.position.z + odom_pos(2);

    quat2 = tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Quaternion quat_res;

    quat_res = quat2 * quat1;
    q(3) = quat_res.getX();
    q(4) = quat_res.getY();
    q(5) = quat_res.getZ();
    q(6) = quat_res.getW();
}