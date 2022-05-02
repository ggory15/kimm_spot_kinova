#pragma once

#include <spot_kinova_framework/servers/action_server_base.hpp>
#include <spot_kinova_msgs/PredefinedPostureAction.h>

#include <fstream>

using namespace pinocchio;
using namespace Eigen;

class PredefinedPostureActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<spot_kinova_msgs::PredefinedPostureAction> as_;

  spot_kinova_msgs::PredefinedPostureFeedback feedback_;
  spot_kinova_msgs::PredefinedPostureResult result_;
  spot_kinova_msgs::PredefinedPostureGoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;

public:
  PredefinedPostureActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SpotKinovaWrapper>  &mu);

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;

protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time);
};