#pragma once

#include <spot_kinova_framework/servers/action_server_base.hpp>
#include <spot_kinova_msgs/WalkAction.h>
#include "geometry_msgs/PoseStamped.h"
#include <move_base_msgs/MoveBaseActionResult.h>
#include "tf/transform_datatypes.h"
#include <fstream>
#include "spot_msgs/TrajectoryAction.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "actionlib/client/simple_goal_state.h"
#include "actionlib/client/simple_client_goal_state.h"
#include "actionlib/client/terminal_state.h"
#include "pinocchio/fwd.hpp"

using namespace pinocchio;
using namespace Eigen;

class WalkActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<spot_kinova_msgs::WalkAction> as_;
  actionlib::SimpleActionClient<spot_msgs::TrajectoryAction>* ac_;

  spot_kinova_msgs::WalkFeedback feedback_;
  spot_kinova_msgs::WalkResult result_;
  spot_kinova_msgs::WalkGoalConstPtr goal_;

  ros::Publisher move_base_publisher_;
  ros::Subscriber move_base_subscriber_;
  geometry_msgs::PoseStamped msg_;
  bool walk_done_, isrelative_;

  void goalCallback() override;
  void preemptCallback() override;
  void movebaseCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);

public:
  WalkActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SpotKinovaWrapper>  &mu);

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;



protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time);
};