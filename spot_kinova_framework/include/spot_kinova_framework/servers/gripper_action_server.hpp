#pragma once

#include <spot_kinova_framework/servers/action_server_base.hpp>
#include <spot_kinova_msgs/GripperAction.h>

#include <fstream>

using namespace pinocchio;
using namespace Eigen;

class GripperActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<spot_kinova_msgs::GripperAction> as_;

  spot_kinova_msgs::GripperFeedback feedback_;
  spot_kinova_msgs::GripperResult result_;
  spot_kinova_msgs::GripperGoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;

public:
  GripperActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SpotKinovaWrapper>  &mu);

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;

protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time);
};