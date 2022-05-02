#pragma once

#include <spot_kinova_framework/servers/action_server_base.hpp>
#include <spot_kinova_msgs/WholebodyAction.h>
#include "tf/transform_datatypes.h"

#include <fstream>

using namespace pinocchio;
using namespace Eigen;

class WholebodyActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<spot_kinova_msgs::WholebodyAction> as_;

  spot_kinova_msgs::WholebodyFeedback feedback_;
  spot_kinova_msgs::WholebodyResult result_;
  spot_kinova_msgs::WholebodyGoalConstPtr goal_;
  bool isrelative_;

  void goalCallback() override;
  void preemptCallback() override;

public:
  WholebodyActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SpotKinovaWrapper>  &mu);

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;

protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time);
};