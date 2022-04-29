#pragma once

#include <spot_kinova_framework/servers/action_server_base.hpp>
#include <spot_kinova_msgs/SE3Action.h>
#include "tf/transform_datatypes.h"

#include <fstream>

using namespace pinocchio;
using namespace Eigen;

class SE3ActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<spot_kinova_msgs::SE3Action> as_;

  spot_kinova_msgs::SE3Feedback feedback_;
  spot_kinova_msgs::SE3Result result_;
  spot_kinova_msgs::SE3GoalConstPtr goal_;
  bool isrelative_;

  void goalCallback() override;
  void preemptCallback() override;

public:
  SE3ActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SpotKinovaWrapper>  &mu);

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;

protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time);
};