#pragma once

#include <spot_kinova_framework/servers/action_server_base.hpp>
#include <spot_kinova_msgs/WalkSimulationAction.h>
#include "geometry_msgs/PoseStamped.h"
#include <move_base_msgs/MoveBaseActionResult.h>
#include "tf/transform_datatypes.h"
#include <fstream>

using namespace pinocchio;
using namespace Eigen;

class WalkSimulationActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<spot_kinova_msgs::WalkSimulationAction> as_;

  spot_kinova_msgs::WalkSimulationFeedback feedback_;
  spot_kinova_msgs::WalkSimulationResult result_;
  spot_kinova_msgs::WalkSimulationGoalConstPtr goal_;

  ros::Publisher move_base_publisher_;
  ros::Subscriber move_base_subscriber_;
  geometry_msgs::PoseStamped msg_;
  bool walk_done_, isrelative_;

  void goalCallback() override;
  void preemptCallback() override;
  void movebaseCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);

public:
  WalkSimulationActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SpotKinovaWrapper>  &mu);

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;



protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time);
};