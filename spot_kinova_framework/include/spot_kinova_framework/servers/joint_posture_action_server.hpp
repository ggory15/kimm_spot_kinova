#pragma once

#include <spot_kinova_framework/servers/action_server_base.hpp>
#include <spot_kinova_msgs/JointPostureAction.h>

#include <fstream>

using namespace pinocchio;
using namespace Eigen;

class JointPostureActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<spot_kinova_msgs::JointPostureAction> as_;

  spot_kinova_msgs::JointPostureFeedback feedback_;
  spot_kinova_msgs::JointPostureResult result_;
  spot_kinova_msgs::JointPostureGoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;

public:
  JointPostureActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SpotKinovaWrapper>  &mu);

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;

protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time);
};