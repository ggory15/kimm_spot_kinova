#include "spot_kinova_framework/controller/controller.hpp"

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace spotkinova;
using namespace spotkinova::trajectory;
using namespace spotkinova::math;
using namespace spotkinova::tasks;
using namespace spotkinova::solver;
using namespace spotkinova::robots;



namespace RobotController{
    SpotKinovaWrapper::SpotKinovaWrapper(const std::string & robot_node, const bool & issimulation, ros::NodeHandle & node)
    : robot_node_(robot_node), issimulation_(issimulation), n_node_(node)
    {
        time_ = 0.;
        mode_change_ = false;
        ctrl_mode_ = 999;
    }
    void SpotKinovaWrapper::initialize(){
        string model_path, urdf_name, username, password, ip_address;
        int PORT;
        n_node_.getParam("/" + robot_node_ +"/urdf_path", model_path);    
        n_node_.getParam("/" + robot_node_ +"/urdf_name", urdf_name);  
        // ROS_WARN_STREAM("/" + robot_node_ +"/robot_urdf_path");

        if (!issimulation_){
            n_node_.getParam("/" + robot_node_ +"/username", username);    
            n_node_.getParam("/" + robot_node_ +"/password", password);  
            n_node_.getParam("/" + robot_node_ +"/port_number", PORT);    
            n_node_.getParam("/" + robot_node_ +"/ip_address", ip_address);  
        }

        vector<string> package_dirs;
        package_dirs.push_back(model_path);
        string urdfFileName = package_dirs[0] + urdf_name;
        robot_ = std::make_shared<RobotWrapper>(urdfFileName, package_dirs, pinocchio::JointModelFreeFlyer(), false);
        model_ = robot_->model();

        state_.q.setZero(26);
        state_.v.setZero(25);
        state_.kinova.q.setZero();
        state_.kinova.v.setZero();
        state_.spot.body_tilted = false;
        state_.spot.orientation_publish = false;

        tsid_ = std::make_shared<InverseDynamicsFormulationAccForce>("tsid", *robot_);
        tsid_->computeProblemData(time_, state_.q, state_.v);
        data_ = tsid_->data();

        postureTask_ = std::make_shared<TaskJointPosture>("task-posture", *robot_);
        state_.kinova.ee_offset = Vector3d(0, 0, -0.1);
        eeTask_ = std::make_shared<TaskSE3Equality>("task-se3", *robot_, "joint_7", state_.kinova.ee_offset);
        Vector7d posture_gain;
        Vector6d ee_gain;
        posture_gain.setOnes();
        ee_gain.setOnes();

        if (!issimulation_){
        	posture_gain *= 20.0;
            ee_gain *= 20.0;
        }
        else {
        	posture_gain *= 10.0;
            ee_gain *= 3.0;	
        }
        postureTask_->Kp(posture_gain);
        postureTask_->Kd(2.0*postureTask_->Kp().cwiseSqrt());
        eeTask_->Kp(ee_gain);
        eeTask_->Kd(2.0*eeTask_->Kp().cwiseSqrt());

        sampleEE_.resize(12, 6);
        samplePosture_.resize(7);

        trajPosture_Cubic_ = std::make_shared<TrajectoryEuclidianCubic>("traj_posture");
        trajPosture_Constant_ = std::make_shared<TrajectoryEuclidianConstant>("traj_posture_constant");
        trajPosture_Timeopt_ = std::make_shared<TrajectoryEuclidianTimeopt>("traj_posture_timeopt");
        
        trajEE_Cubic_ = std::make_shared<TrajectorySE3Cubic>("traj_ee");
        trajEE_Constant_ = std::make_shared<TrajectorySE3Constant>("traj_ee_constant");
        Vector3d Maxvel_ee = Vector3d::Ones()*0.2;
        Vector3d Maxacc_ee = Vector3d::Ones()*0.2;
        trajEE_Timeopt_ = std::make_shared<TrajectorySE3Timeopt>("traj_ee_timeopt", Maxvel_ee, Maxacc_ee);

        solver_ = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG, "quadprog");     

        // For Real Kinova
        if (!issimulation_){
            auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };

            transport_ = new k_api::TransportClientTcp();
            transport_real_time_ = new k_api::TransportClientUdp();
            router_ = new k_api::RouterClient(transport_, error_callback);
            router_real_time_ = new k_api::RouterClient(transport_real_time_, error_callback);
            transport_->connect(ip_address, PORT);
            transport_real_time_->connect(ip_address, PORT+1);

            create_session_info_ = new k_api::Session::CreateSessionInfo();
            create_session_info_->set_username(username);
            create_session_info_->set_password(password);
            create_session_info_->set_session_inactivity_timeout(60000);   // (milliseconds)
            create_session_info_->set_connection_inactivity_timeout(2000); // (milliseconds)

            // Session manager service wrapper
            std::cout << "Creating session for communication" << std::endl;
            session_manager_ = new k_api::SessionManager(router_);
            session_manager_->CreateSession(*create_session_info_);
            session_manager_real_time_ = new k_api::SessionManager(router_real_time_);
            session_manager_real_time_->CreateSession(*create_session_info_);
            std::cout << "Session created" << std::endl;

            base_ = new k_api::Base::BaseClient(router_);
            base_cyclic_ = new k_api::BaseCyclic::BaseCyclicClient(router_real_time_);
            actuator_config_ = new k_api::ActuatorConfig::ActuatorConfigClient(router_);
            servoingMode_ = new k_api::Base::ServoingModeInformation();
            servoingMode_->set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
            base_->SetServoingMode(*servoingMode_);

            state_.kinova.lowlevel_ctrl = false;
            base_->ClearFaults();
        }
    }

    void SpotKinovaWrapper::kinova_update(){
        if (!issimulation_){
            if (!state_.kinova.lowlevel_ctrl){
                  base_feedback_ = base_cyclic_->RefreshFeedback();
                for (int i=0; i<7; i++){
                    state_.kinova.q(i) = wrapRadiansFromMinusPiToPi(base_feedback_.actuators(i).position() * M_PI / 180.0);
                    state_.kinova.v(i) = base_feedback_.actuators(i).velocity() * M_PI / 180.0;
                }
            }
            else{
                for (int i=0; i<7; i++) {
                    state_.kinova.q(i) = wrapRadiansFromMinusPiToPi(base_feedback_.actuators(i).position() * M_PI / 180.0);
                    state_.kinova.v(i) = base_feedback_.actuators(i).velocity() * M_PI / 180.0;
                }
            }
        }
        state_.q.segment(7, 7) = state_.kinova.q; 
        robot_->computeAllTerms(data_, state_.q, state_.v);
        state_.v.setZero();
        SE3 tf_identity;
        tf_identity.setIdentity();
        tf_identity.translation() = state_.kinova.ee_offset;
        state_.kinova.H_ee = robot_->position(data_, robot_->model().getJointId("joint_7")) * tf_identity;
    }

    void SpotKinovaWrapper::spot_update(Vector3d& x, tf::Quaternion& quat, tf::Quaternion& pose, tf::Quaternion& nav){
        for (int i=0; i<3; i++)
            state_.q(i) = x(i);

        state_.q(3) = quat.getX();
        state_.q(4) = quat.getY();
        state_.q(5) = quat.getZ();
        state_.q(6) = quat.getW();

        state_.spot.orientation = pose;
        state_.spot.nav = nav;
    }


    void SpotKinovaWrapper::init_joint_posture_ctrl(ros::Time time){
        tsid_->removeTask("task-posture");
        tsid_->removeTask("task-se3");
        tsid_->addMotionTask(*postureTask_, 1e-5, 0);

        trajPosture_Cubic_->setInitSample(state_.kinova.q);
        trajPosture_Cubic_->setDuration(state_.kinova.duration);
        trajPosture_Cubic_->setStartTime(time.toSec());
        trajPosture_Cubic_->setGoalSample(state_.kinova.q_ref);  
        
        if (issimulation_){
            
        }
        else{
            base_->ClearFaults();

            servoingMode_->set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
            base_->SetServoingMode(*servoingMode_);
            base_feedback_ = base_cyclic_->RefreshFeedback();
            base_command_.clear_actuators();
            for(int i = 0; i < 7; i++)
            {
                base_command_.add_actuators()->set_position(base_feedback_.actuators(i).position());
            }
            base_feedback_ = base_cyclic_->Refresh(base_command_);
            control_mode_message_ = new k_api::ActuatorConfig::ControlModeInformation();
            control_mode_message_->set_control_mode(k_api::ActuatorConfig::ControlMode::VELOCITY);

            for (int i=0; i<7; i++)
                actuator_config_->SetControlMode(*control_mode_message_, i+1);
        }
    }
    void SpotKinovaWrapper::compute_joint_posture_ctrl(ros::Time time){
        trajPosture_Cubic_->setCurrentTime(time.toSec());
        samplePosture_ = trajPosture_Cubic_->computeNext();
        postureTask_->setReference(samplePosture_);
    
        const HQPData & HQPData = tsid_->computeProblemData(time.toSec(), state_.q, state_.v);       
        state_.v = tsid_->getAccelerations(solver_->solve(HQPData));
        //state_.kinova.q = pinocchio::integrate(robot_->model(), state_.q, 0.001 * state_.v).segment(7,7);

        if (issimulation_){            
            state_.kinova.q = pinocchio::integrate(robot_->model(), state_.q, 0.001 * state_.v).segment(7,7);
        }
        else{
            for(int i = 0; i < 7; i++)
            {
                base_command_.mutable_actuators(i)->set_position(base_feedback_.actuators(i).position());
                base_command_.mutable_actuators(i)->set_velocity(state_.v(i+6) *180.0 / M_PI);       		    
            }
            
            base_command_.set_frame_id(base_command_.frame_id() + 1);
            if (base_command_.frame_id() > 65535)
                base_command_.set_frame_id(0);

            for (int idx = 0; idx < 7; idx++)
            {
                base_command_.mutable_actuators(idx)->set_command_id(base_command_.frame_id());
            }

            base_feedback_ = base_cyclic_->Refresh(base_command_, 0);
        }
    }
    
    void SpotKinovaWrapper::init_se3_ctrl(ros::Time time){
        tsid_->removeTask("task-posture");
        tsid_->removeTask("task-se3");
        tsid_->addMotionTask(*postureTask_, 1e-5, 1);
        tsid_->addMotionTask(*eeTask_, 1, 0);

        state_.kinova.q_ref = state_.kinova.q;

        trajPosture_Cubic_->setInitSample(state_.kinova.q);
        trajPosture_Cubic_->setDuration(0.1);
        trajPosture_Cubic_->setStartTime(time.toSec());
        trajPosture_Cubic_->setGoalSample(state_.kinova.q_ref);   

        trajEE_Cubic_->setStartTime(time.toSec());
        trajEE_Cubic_->setDuration(state_.kinova.duration);

        trajEE_Cubic_->setInitSample(state_.kinova.H_ee);     
        if (state_.kinova.isrelative){
            Eigen::Quaterniond recieve_quat_eigen(state_.q(6), state_.q(3), state_.q(4), state_.q(5));
            state_.kinova.H_ee_ref.translation() = recieve_quat_eigen.toRotationMatrix() * state_.kinova.H_ee_ref.translation() + state_.kinova.H_ee.translation();
            state_.kinova.H_ee_ref.rotation() = recieve_quat_eigen.toRotationMatrix()*state_.kinova.H_ee_ref.rotation() * state_.kinova.H_ee.rotation();
        }
        trajEE_Cubic_->setGoalSample(state_.kinova.H_ee_ref);

        Vector6d ee_gain_tmp;
        ee_gain_tmp.setOnes();

        if (!issimulation_ && state_.spot.orientation_publish){
            ee_gain_tmp *= 2.0;
            eeTask_->Kp(ee_gain_tmp);
            eeTask_->Kd(2.0*eeTask_->Kp().cwiseSqrt());
        }

        if (issimulation_){
            
        }
        else{
            base_->ClearFaults();

            servoingMode_->set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
            base_->SetServoingMode(*servoingMode_);
            
            base_feedback_ = base_cyclic_->RefreshFeedback();
            base_command_.clear_actuators();
            for(int i = 0; i < 7; i++)
            {
                base_command_.add_actuators()->set_position(base_feedback_.actuators(i).position());
            }
            base_feedback_ = base_cyclic_->Refresh(base_command_);
            control_mode_message_ = new k_api::ActuatorConfig::ControlModeInformation();
            control_mode_message_->set_control_mode(k_api::ActuatorConfig::ControlMode::VELOCITY);

            for (int i=0; i<7; i++)
                actuator_config_->SetControlMode(*control_mode_message_, i+1);    
            std::this_thread::sleep_for(std::chrono::milliseconds(100));        
        }
    }
    void SpotKinovaWrapper::compute_se3_ctrl(ros::Time time){
        trajPosture_Cubic_->setCurrentTime(time.toSec());
        samplePosture_ = trajPosture_Cubic_->computeNext();
        postureTask_->setReference(samplePosture_);

        trajEE_Cubic_->setCurrentTime(time.toSec());
        sampleEE_ = trajEE_Cubic_->computeNext();
        eeTask_->setReference(sampleEE_);
            
        const HQPData & HQPData = tsid_->computeProblemData(time.toSec(), state_.q, state_.v);       
        state_.v = tsid_->getAccelerations(solver_->solve(HQPData));
               
        if (issimulation_)
            state_.kinova.q = pinocchio::integrate(robot_->model(), state_.q, 0.001 * state_.v).segment(7,7);
        else{
            for(int i = 0; i < 7; i++)
            {
                base_command_.mutable_actuators(i)->set_position(base_feedback_.actuators(i).position());
                base_command_.mutable_actuators(i)->set_velocity( (state_.v.segment(6,7)(i) - 0.5 * state_.kinova.v(i))* 180.0 /M_PI );        		    
            }
            
            base_command_.set_frame_id(base_command_.frame_id() + 1);
            if (base_command_.frame_id() > 65535)
                base_command_.set_frame_id(0);

            for (int idx = 0; idx < 7; idx++)
            {
                base_command_.mutable_actuators(idx)->set_command_id(base_command_.frame_id());
            }

            base_feedback_ = base_cyclic_->Refresh(base_command_, 0);
        }
    }

    void SpotKinovaWrapper::done_se3_ctrl(){
        control_mode_message_ = new k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message_->set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        for (int i=0; i<7; i++)
            actuator_config_->SetControlMode(*control_mode_message_, i+1);
        
        servoingMode_->set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        base_->SetServoingMode(*servoingMode_);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        action_type_ = new k_api::Base::RequestedActionType();
        action_type_->set_action_type(k_api::Base::REACH_JOINT_ANGLES);                    
        auto action_list = base_->ReadAllActions(*action_type_);
        auto action_handle = k_api::Base::ActionHandle();
        action_handle.set_identifier(0);
    }

    void SpotKinovaWrapper::init_body_posture_ctrl(ros::Time time){
        state_.spot.orientation_init = state_.spot.orientation;
        time_= time.toSec();
    }
    void SpotKinovaWrapper::compute_body_posture_ctrl(ros::Time time){
        double cubic_t = cubic(time.toSec(), time_, time_ + state_.spot.duration, 0.0, 1.0, 0.0, 0.0 );
        state_.spot.orientation_des = state_.spot.orientation_init.slerp(state_.spot.orientation_ref, cubic_t);
    }

    void SpotKinovaWrapper::init_predefined_posture_ctrl(string & name){
        base_->ClearFaults();
        control_mode_message_ = new k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message_->set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        for (int i=0; i<7; i++)
            actuator_config_->SetControlMode(*control_mode_message_, i+1);
        
        servoingMode_->set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        base_->SetServoingMode(*servoingMode_);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        action_type_ = new k_api::Base::RequestedActionType();
        action_type_->set_action_type(k_api::Base::REACH_JOINT_ANGLES);                    
        auto action_list = base_->ReadAllActions(*action_type_);
        auto action_handle = k_api::Base::ActionHandle();
        action_handle.set_identifier(0);

        for (auto action : action_list.action_list()) 
        {
            if (action.name() == name) 
            {
                action_handle = action.handle();
            }
        }

        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base_->OnNotificationActionTopic(
            SpotKinovaWrapper::create_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions()
        );

        // Execute action
        base_->ExecuteActionFromReference(action_handle);

        // Wait for future value from promise
        const auto status = finish_future.wait_for(std::chrono::seconds{20});
        base_->Unsubscribe(promise_notification_handle);

        if(status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
        }
        const auto promise_event = finish_future.get();
    }

    void SpotKinovaWrapper::init_open_gripper(){
        k_api::Base::GripperCommand gripper_command;
        gripper_command.set_mode(k_api::Base::GRIPPER_POSITION);
        auto finger = gripper_command.mutable_gripper()->add_finger();
        finger->set_finger_identifier(1);
        finger->set_value(0.0f);
        base_->SendGripperCommand(gripper_command);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        //std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    }
    
    void SpotKinovaWrapper::init_close_girpper(){
        k_api::Base::GripperCommand gripper_command;
        gripper_command.set_mode(k_api::Base::GRIPPER_POSITION);
        auto finger = gripper_command.mutable_gripper()->add_finger();
        finger->set_finger_identifier(1);
        finger->set_value(1.0f);
        base_->SendGripperCommand(gripper_command);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    void SpotKinovaWrapper::init_se3_array_ctrl(ros::Time time){
        tsid_->removeTask("task-posture");
        tsid_->removeTask("task-se3");
        tsid_->addMotionTask(*postureTask_, 1e-5, 1);
        tsid_->addMotionTask(*eeTask_, 1, 0);

        state_.kinova.q_ref = state_.kinova.q;

        trajPosture_Cubic_->setInitSample(state_.kinova.q);
        trajPosture_Cubic_->setDuration(0.1);
        trajPosture_Cubic_->setStartTime(time.toSec());
        trajPosture_Cubic_->setGoalSample(state_.kinova.q_ref);   
        
        trajEE_Cubic_->setStartTime(time.toSec());
        trajEE_Cubic_->setDuration(state_.kinova.duration_array[0]);
        state_.kinova.H_ee_init = state_.kinova.H_ee;
        stime_ = time.toSec();
        isfinished_ = false;
        array_cnt_ = 0;

        trajEE_Cubic_->setInitSample(state_.kinova.H_ee);     
        state_.kinova.H_ee_ref = state_.kinova.H_ee_ref_array[0];
        state_.kinova.H_ee_init = state_.kinova.H_ee;

        if (state_.kinova.isrelative){
            Eigen::Quaterniond recieve_quat_eigen(state_.q(6), state_.q(3), state_.q(4), state_.q(5));
            state_.kinova.H_ee_ref.translation() = recieve_quat_eigen.toRotationMatrix() * state_.kinova.H_ee_ref_array[array_cnt_].translation() + state_.kinova.H_ee_init.translation();
            state_.kinova.H_ee_ref.rotation() = state_.kinova.H_ee_init.rotation() * state_.kinova.H_ee_ref_array[array_cnt_].rotation();
        }
        trajEE_Cubic_->setGoalSample(state_.kinova.H_ee_ref);
                
        Vector6d ee_gain_tmp;
        ee_gain_tmp.setOnes();

        if (!issimulation_ && state_.spot.orientation_publish){
            ee_gain_tmp *= 2.0;
            eeTask_->Kp(ee_gain_tmp);
            eeTask_->Kd(2.0*eeTask_->Kp().cwiseSqrt());
        }

        if (issimulation_){
            
        }
        else{
            base_->ClearFaults();

            servoingMode_->set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
            base_->SetServoingMode(*servoingMode_);
            
            base_feedback_ = base_cyclic_->RefreshFeedback();
            base_command_.clear_actuators();
            for(int i = 0; i < 7; i++)
            {
                base_command_.add_actuators()->set_position(base_feedback_.actuators(i).position());
            }
            base_feedback_ = base_cyclic_->Refresh(base_command_);
            control_mode_message_ = new k_api::ActuatorConfig::ControlModeInformation();
            control_mode_message_->set_control_mode(k_api::ActuatorConfig::ControlMode::VELOCITY);

            for (int i=0; i<7; i++)
                actuator_config_->SetControlMode(*control_mode_message_, i+1);    
            std::this_thread::sleep_for(std::chrono::milliseconds(100));        
        }
    }        
    void SpotKinovaWrapper::compute_se3_array_ctrl(ros::Time time){
        trajPosture_Cubic_->setCurrentTime(time.toSec());
        samplePosture_ = trajPosture_Cubic_->computeNext();
        postureTask_->setReference(samplePosture_);
        
        int array_size = state_.kinova.duration_array.size();

        if ( (time.toSec() - stime_) >= state_.kinova.duration_array[array_cnt_] && array_cnt_ < array_size){     
            stime_ = time.toSec(); 
            array_cnt_++;

            if (array_cnt_ < array_size){
                trajEE_Cubic_->setStartTime(stime_);
                trajEE_Cubic_->setDuration(state_.kinova.duration_array[array_cnt_]);
                trajEE_Cubic_->setInitSample(state_.kinova.H_ee);     
                state_.kinova.H_ee_ref = state_.kinova.H_ee_ref_array[array_cnt_];
                if (state_.kinova.isrelative){
                    Eigen::Quaterniond recieve_quat_eigen(state_.q(6), state_.q(3), state_.q(4), state_.q(5));
                    Eigen::Vector3d pos_des;
                    Eigen::Matrix3d rot_des;
                    pos_des.setZero();
                    rot_des.setIdentity();
                    for (int i=0; i<array_cnt_; i++){
                        pos_des += state_.kinova.H_ee_ref_array[i+1].translation();
                        rot_des *= state_.kinova.H_ee_ref_array[i+1].rotation();
                    }
                    state_.kinova.H_ee_ref.translation() = recieve_quat_eigen.toRotationMatrix() * pos_des + state_.kinova.H_ee_init.translation();
                    state_.kinova.H_ee_ref.rotation() = state_.kinova.H_ee_init.rotation() * rot_des;
                }
                
                trajEE_Cubic_->setGoalSample(state_.kinova.H_ee_ref);    
            }            
        }

        
        trajEE_Cubic_->setCurrentTime(time.toSec());
        sampleEE_ = trajEE_Cubic_->computeNext();
        eeTask_->setReference(sampleEE_);
           
        const HQPData & HQPData = tsid_->computeProblemData(time.toSec(), state_.q, state_.v);       
        state_.v = tsid_->getAccelerations(solver_->solve(HQPData));
               
        if (issimulation_)
            state_.kinova.q = pinocchio::integrate(robot_->model(), state_.q, 0.001 * state_.v).segment(7,7);
        else{
            for(int i = 0; i < 7; i++)
            {
                base_command_.mutable_actuators(i)->set_position(base_feedback_.actuators(i).position());
                base_command_.mutable_actuators(i)->set_velocity( (state_.v.segment(6,7)(i) - 0.5 * state_.kinova.v(i))* 180.0 /M_PI );        		    
            }
            
            base_command_.set_frame_id(base_command_.frame_id() + 1);
            if (base_command_.frame_id() > 65535)
                base_command_.set_frame_id(0);

            for (int idx = 0; idx < 7; idx++)
            {
                base_command_.mutable_actuators(idx)->set_command_id(base_command_.frame_id());
            }

            base_feedback_ = base_cyclic_->Refresh(base_command_, 0);
        }
    }
    

}