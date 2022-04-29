#ifndef __spot_kinova_ctrl__
#define __spot_kinova_ctrl__

//Pinocchio Header
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp> 

//ROS Header
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Transform.h"
#include "std_msgs/Int16.h"
#include "tf/transform_datatypes.h"

//SYSTEM Header
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

//KIMM spot kinova Header
#include <kimm_spot_kinova/math/fwd.hpp>
#include "kimm_spot_kinova/robots/robot-wrapper.hpp"
#include <kimm_spot_kinova/formulation/inverse_dynamics_formulation_vel.hpp>
#include <kimm_spot_kinova/tasks/task_joint_posture.hpp>
#include <kimm_spot_kinova/tasks/task_se3_equality.hpp>
#include <kimm_spot_kinova/solver/solver_HQP_eiquadprog.hpp>
#include <kimm_spot_kinova/trajectories/trajectory_euclidian.hpp>
#include <kimm_spot_kinova/trajectories/trajectory_se3.hpp>
#include <kimm_spot_kinova/solver/solver_HQP_factory.hxx>
//#include <kimm_spot_kinova/solver/util.hpp>

// Real Kinova
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>
#include <spot_kinova_framework/utilities/kinova_utilities.hpp>
#include <ActuatorConfigClientRpc.h>

// Real Spot
#include "spot_msgs/TrajectoryAction.h"

using namespace std;
using namespace Eigen;
using namespace pinocchio;
namespace k_api = Kinova::Api;

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef struct Kinova_State {
    Vector7d q;
    Vector7d v;
    Vector7d q_des;
    Vector7d v_des;
    Vector7d q_ref;
    Vector7d v_ref;
    SE3 H_ee_ref;
    SE3 H_ee_init;
    SE3 H_ee;
    double duration;
    bool lowlevel_ctrl;
} kinova_state;
typedef struct Spot_State {
    Vector3d position;
    tf::Quaternion orientation;
    tf::Quaternion orientation_ref;
    tf::Quaternion orientation_des;
    tf::Quaternion orientation_init;    // body pose
    tf::Quaternion nav; // navigation 2d
    double duration;
    bool orientation_publish;
    bool body_tilted;
} spot_state;
typedef struct State {   
    kinova_state kinova;
    spot_state spot;
    VectorXd q; // 7 + 7(kinova) + 12(spot) =26
    VectorXd v; // 6 + 7(kinova) + 12(spot) =25
    double time;
    double duration;
} state;   


namespace RobotController{
    class SpotKinovaWrapper{
        public: 
            SpotKinovaWrapper(const std::string & robot_node, const bool & issimulation, ros::NodeHandle & node);
            ~SpotKinovaWrapper(){};

            void initialize();
            void ctrl_update(const int& ); 
            void kinova_update(); 
            void spot_update(Vector3d& x, tf::Quaternion& quat, tf::Quaternion& pose, tf::Quaternion& nav); 
            void compute(const double &); 
            
            void init_joint_posture_ctrl(ros::Time time);
            void comput_joint_posture_ctrl(ros::Time time);
            void init_se3_ctrl(ros::Time time);
            void comput_se3_ctrl(ros::Time time);
            void init_body_posture_ctrl(ros::Time time);
            void comput_body_posture_ctrl(ros::Time time);


            int ctrltype(){
                return ctrl_mode_;
            }
            State & state(){
                return state_;
            }
            bool simulation(){
                return issimulation_;
            }
            void computeAllTerms(){
                robot_->computeAllTerms(data_, state_.q, state_.v);
                state_.v.setZero(); 
            }

        private:
            bool issimulation_, mode_change_;
            std::string robot_node_;
            
            State state_;
            double time_;
            int ctrl_mode_;

            std::shared_ptr<spotkinova::robots::RobotWrapper> robot_;
            pinocchio::Model model_;
            pinocchio::Data data_;

            std::shared_ptr<spotkinova::InverseDynamicsFormulationAccForce> tsid_;           
            std::shared_ptr<spotkinova::tasks::TaskJointPosture> postureTask_;
            std::shared_ptr<spotkinova::tasks::TaskSE3Equality> eeTask_;

            std::shared_ptr<spotkinova::trajectory::TrajectoryEuclidianCubic> trajPosture_Cubic_;
            std::shared_ptr<spotkinova::trajectory::TrajectoryEuclidianConstant> trajPosture_Constant_;
            std::shared_ptr<spotkinova::trajectory::TrajectoryEuclidianTimeopt> trajPosture_Timeopt_;
            std::shared_ptr<spotkinova::trajectory::TrajectorySE3Cubic> trajEE_Cubic_;
            std::shared_ptr<spotkinova::trajectory::TrajectorySE3Constant> trajEE_Constant_;
            std::shared_ptr<spotkinova::trajectory::TrajectorySE3Timeopt> trajEE_Timeopt_;            

            spotkinova::trajectory::TrajectorySample sampleEE_, samplePosture_;
            spotkinova::solver::SolverHQPBase * solver_;

            ros::NodeHandle n_node_;

            // For real Kinova
            k_api::TransportClientTcp* transport_;
            k_api::TransportClientUdp* transport_real_time_;
            k_api::RouterClient* router_;
            k_api::RouterClient* router_real_time_;
            k_api::Session::CreateSessionInfo* create_session_info_;
            k_api::SessionManager* session_manager_;
            k_api::SessionManager* session_manager_real_time_;
            k_api::Base::BaseClient* base_;
            k_api::BaseCyclic::BaseCyclicClient* base_cyclic_;
            k_api::ActuatorConfig::ActuatorConfigClient* actuator_config_;
            k_api::Base::ServoingModeInformation* servoingMode_;
            k_api::ActuatorConfig::ControlModeInformation* control_mode_message_;
            k_api::Base::RequestedActionType* action_type_;

            k_api::Base::JointAngles joint_from_kinova_;
            k_api::BaseCyclic::Feedback base_feedback_;
            k_api::BaseCyclic::Command  base_command_;

            std::function<void(k_api::Base::ActionNotification)> 
            create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
            {
                return [&finish_promise] (k_api::Base::ActionNotification notification)
                {
                    const auto action_event = notification.action_event();
                    switch(action_event)
                    {
                    case k_api::Base::ActionEvent::ACTION_END:
                    case k_api::Base::ActionEvent::ACTION_ABORT:
                        finish_promise.set_value(action_event);
                        break;
                    default:
                        break;
                    }
                };
            }
            std::function<void(k_api::Base::ActionNotification)>
                create_event_listener_by_ref(k_api::Base::ActionEvent& returnAction)
            {
                return [&returnAction](k_api::Base::ActionNotification notification)
                {
                    const auto action_event = notification.action_event();
                    switch(action_event)
                    {
                    case k_api::Base::ActionEvent::ACTION_END:
                    case k_api::Base::ActionEvent::ACTION_ABORT:
                        returnAction = action_event;
                        break;
                    default:
                        break;
                    }
                };
            }

    };
}
#endif