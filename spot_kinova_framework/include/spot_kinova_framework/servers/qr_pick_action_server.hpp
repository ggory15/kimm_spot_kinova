#pragma once

#include <spot_kinova_framework/servers/action_server_base.hpp>
#include <spot_kinova_msgs/QRPickAction.h>
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

class QRPickActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<spot_kinova_msgs::QRPickAction> as_;

  spot_kinova_msgs::QRPickFeedback feedback_;
  spot_kinova_msgs::QRPickResult result_;
  spot_kinova_msgs::QRPickGoalConstPtr goal_;

  ros::Subscriber qr_subscriber_;
  // geometry_msgs::PoseArray msg_;
  bool qr_recieved_;
  geometry_msgs::Pose qr_msg_;

  bool isrelative_;
  SE3 qr_tf_;

  void goalCallback() override;
  void preemptCallback() override;
  void qrCallback(const geometry_msgs::Pose::ConstPtr& msg);

public:
  QRPickActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SpotKinovaWrapper>  &mu);

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;



protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time);
};