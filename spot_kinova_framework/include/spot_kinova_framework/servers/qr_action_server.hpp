#pragma once

#include <spot_kinova_framework/servers/action_server_base.hpp>
#include <spot_kinova_msgs/QRAction.h>
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

class QRActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<spot_kinova_msgs::QRAction> as_;
  actionlib::SimpleActionClient<spot_msgs::TrajectoryAction>* ac_;

  spot_kinova_msgs::QRFeedback feedback_;
  spot_kinova_msgs::QRResult result_;
  spot_kinova_msgs::QRGoalConstPtr goal_;

  ros::Subscriber qr_subscriber_;
  geometry_msgs::PoseStamped msg_;
  bool qr_recieved_;
  geometry_msgs::Pose qr_msg_;

  // ros::node node_;

  void goalCallback() override;
  void preemptCallback() override;
  void qrCallback(const geometry_msgs::Pose::ConstPtr& msg);

public:
  QRActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SpotKinovaWrapper>  &mu);

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;



protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time);
};