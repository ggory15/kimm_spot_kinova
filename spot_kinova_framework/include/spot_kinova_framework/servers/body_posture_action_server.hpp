#pragma once

#include <spot_kinova_framework/servers/action_server_base.hpp>
#include <spot_kinova_msgs/BodyPostureAction.h>
#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h"
#include <fstream>

using namespace pinocchio;
using namespace Eigen;

class BodyPostureActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<spot_kinova_msgs::BodyPostureAction> as_;

  spot_kinova_msgs::BodyPostureFeedback feedback_;
  spot_kinova_msgs::BodyPostureResult result_;
  spot_kinova_msgs::BodyPostureGoalConstPtr goal_;

  ros::Publisher move_base_publisher_;
  ros::Subscriber move_base_subscriber_;
  geometry_msgs::PoseStamped msg_;

  void goalCallback() override;
  void preemptCallback() override;

public:
  BodyPostureActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SpotKinovaWrapper>  &mu);

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;

protected:
  void setSucceeded() override;
  void setAborted() override;
};