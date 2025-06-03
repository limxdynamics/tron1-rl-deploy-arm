// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#include "robot_controllers/ControllerBase.h"
#include "pluginlib/class_list_macros.hpp"
#include <std_msgs/Float32MultiArray.h>

namespace robot_controller {

// Initializes the controller.
bool ControllerBase::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh) {
  // Retrieve the robot type from the environment variable "ROBOT_TYPE"
  const char* value = ::getenv("ROBOT_TYPE");
  if (value && strlen(value) > 0) {
    robotType_ = std::string(value);
  } else {
    ROS_ERROR("Error: Please set the ROBOT_TYPE using 'export ROBOT_TYPE=<robot_type>'.");
    abort();
  }

  if (!loadRLCfg()) {
    ROS_ERROR("Error in loadRLCfg");
  }
  if (!loadModel()) {
    ROS_ERROR("Error in loadModel");
  }
  std::map<std::string, double> initState;
  if (robotType_ == "WF_TRON1A")
  {
      initState = getWheelRobotCfg().initState;
  }
  else if (robotType_ == "SF_TRON1A")
  {
      initState = getSoleRobotCfg().initState;
  }

  // Hardware interface
  auto *hybridJointInterface = robot_hw->get<robot_common::HybridJointInterface>();
  initJointAngles_.resize(initState.size());
  // Note: The order of joint angles below should match the order during training.
  std::string jointNames;
  for (size_t i = 0; i < jointNames_.size(); i++) {
    hybridJointHandles_.push_back(hybridJointInterface->getHandle(jointNames_[i]));
    initJointAngles_(i) = initState[jointNames_[i]];
    jointNames += jointNames_[i];
    if (i != jointNames_.size() -1) {
      jointNames += ", ";
    }
  }

  ROS_INFO_STREAM("hybridJointHandles jointNames: " << jointNames.c_str());

  defaultJointAngles_.resize(hybridJointHandles_.size());
  defaultJointAngles_.setZero();

  imuSensorHandles_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("limx_imu");

  auto *contactInterface = robot_hw->get<robot_common::ContactSensorInterface>();

  // Subscribe to command velocity topic.
  cmdVelSub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &ControllerBase::cmdVelCallback, this); 
  debugPub_ = nh_.advertise<std_msgs::Float32MultiArray>("/obs_debug_info", false, 1);

  return true;
}

} // namespace robot_controller

PLUGINLIB_EXPORT_CLASS(robot_controller::ControllerBase, controller_interface::ControllerBase)
