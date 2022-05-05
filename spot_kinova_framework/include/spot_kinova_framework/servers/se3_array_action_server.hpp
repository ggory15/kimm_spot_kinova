#pragma once

#include <spot_kinova_framework/servers/action_server_base.hpp>
#include <spot_kinova_msgs/SE3ArrayAction.h>
#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h"
#include <fstream>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "actionlib/client/simple_goal_state.h"
#include "actionlib/client/simple_client_goal_state.h"
#include "actionlib/client/terminal_state.h"
#include "pinocchio/fwd.hpp"

using namespace pinocchio;
using namespace Eigen;

class SE3ArrayActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<spot_kinova_msgs::SE3ArrayAction> as_;

  spot_kinova_msgs::SE3ArrayFeedback feedback_;
  spot_kinova_msgs::SE3ArrayResult result_;
  spot_kinova_msgs::SE3ArrayGoalConstPtr goal_;

  bool isrelative_;

  void goalCallback() override;
  void preemptCallback() override;

public:
  SE3ArrayActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SpotKinovaWrapper>  &mu);

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;

protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time);
};