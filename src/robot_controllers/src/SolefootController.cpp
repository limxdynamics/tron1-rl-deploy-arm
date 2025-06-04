// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#include "robot_controllers/SolefootController.h"

#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <numeric>

namespace robot_controller {

// Initialize the controller
bool SolefootController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh) {
  // initialize initial positions
  standCenterPos_.setZero(8 + 6);
  standJointPos_.setZero(8);

  eeCommands_.resize(9); // x y z xx xy xz yx yy yz
  eeCommands_ << 0.346+0.1, 0, 0.241, 0, 0, 1, -1, 0, 0;

  ee_init_pos_ << 0.346+0.1, 0, 0.241;
  ee_init_rpy_ << -M_PI/2, 0, -M_PI/2;

  // register publishers and subscribers
  gripper_cmd_pub_ = nh_.advertise<std_msgs::Bool>("/gripper_cmd", false, 10);

  ee_pos_cmd_rc_delta_ = nh_.subscribe<std_msgs::Float32MultiArray>("/EEPose_cmd_rc", 10, &SolefootController::EEPoseCmdRCCallback, this);
  ee_pos_cmd_rc_delta_msg_.data.resize(6+1);
  rc_ee_cmd.zero();
  return ControllerBase::init(robot_hw, nh);
}

// Perform initialization when the controller starts
void SolefootController::starting(const ros::Time &time) {
  for (size_t i = 0; i < hybridJointHandles_.size(); i++) {
    ROS_INFO_STREAM("starting hybridJointHandle: " << hybridJointHandles_[i].getPosition());
    defaultJointAngles_[i] = hybridJointHandles_[i].getPosition();
  }

  standCenterPos_ << 0.0,  0.6,  1.2, -0.7,
                             0.0, -0.6, -1.2, -0.7,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  standJointPos_ << 0.0, 0.0, 0.0, -0.11,
                      0.0, 0.0, 0.0, -0.11;

  scalar_t durationSecs = 1.0;
  standDuration_ = durationSecs * 1800.0;
  standCenterDuration_ = durationSecs * 1000.0;
  standCenterPercent_ = 0.0;

  initStandPercent_ = 0.0;
  initStandDuration_ = durationSecs * 800.0;

  commandX_.clear();
  commandY_.clear();
  commandYaw_.clear();

  commandX_.push_back(0.0);
  commandY_.push_back(0.0);
  commandYaw_.push_back(0.0);

  loopCount_ = 0;
  loopCountKeep_ = 0;
  work_mode_flag_ = 10;
  mode_ = Mode::SOLE_STAND;

  gaitIndex_ = 0.0;
}

// Update function called periodically
void SolefootController::update(const ros::Time &time, const ros::Duration &period) {
  switch (mode_)
  {
  case Mode::SOLE_STAND:
      handleSoleStandMode();
      clearData();
      break;
  case Mode::SOLE_WALK:
      handleRLSoleWalkMode();
      break;
  }
  loopCount_++;
  loopCountKeep_++;
  last_mode_ = mode_;
}

void SolefootController::handleSoleStandMode() {
  if (standCenterPercent_ < 1)
  {
    for (int j = 0; j < hybridJointHandles_.size(); j++)
    {
      if(j==8||j==9||j==10){ // arm j123
        scalar_t pos_des = defaultJointAngles_[j] * (1 - standCenterPercent_) + standCenterPos_[j] * standCenterPercent_;
        hybridJointHandles_[j].setCommand(pos_des, 0, 20, 1, 0, 0);
      }
      else if(j==11||j==12||j==13){ // arm j456
        scalar_t pos_des = defaultJointAngles_[j] * (1 - standCenterPercent_) + standCenterPos_[j] * standCenterPercent_;
        hybridJointHandles_[j].setCommand(pos_des, 0, 10, 1, 0, 0);
      }
      else{
        scalar_t pos_des = defaultJointAngles_[j] * (1 - standCenterPercent_) + standCenterPos_[j] * standCenterPercent_;
        hybridJointHandles_[j].setCommand(pos_des, 0, 100, 6, 0, 0);
      }
    }
    standCenterPercent_ += 1 / standCenterDuration_;
  }
  else
  {
    mode_ = Mode::SOLE_WALK;
  }
}

void SolefootController::handleRLSoleWalkMode()
{
  locomotionFlag_ = 0;
  sitDownFlag_ = 0;
  if (work_mode_flag_ != 1)
  {
    ROS_INFO_STREAM("------------------RL Sole Walk------------------");

    loadRLCfg();
    loadModel();
    isfirstRecObs_ = true;
    work_mode_flag_ = 1;
  }
  
  // compute observation & actions
  if (soleRobotCfg_.controlCfg.decimation == 0)
  {
    std::cerr << "----error----  soleRobotCfg_.controlCfg.decimation" << std::endl;
    return;
  }
  if (loopCount_ % soleRobotCfg_.controlCfg.decimation == 0)
  {
    computeObservation();
    computeEncoder();
    computeActions();
    // limit action range
    scalar_t actionMin = -soleRobotCfg_.soleRlCfg.clipActions;
    scalar_t actionMax = soleRobotCfg_.soleRlCfg.clipActions;
    std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                    [actionMin, actionMax](scalar_t x)
                    { return std::max(actionMin, std::min(actionMax, x)); });
  }
  // set action
  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
  for (size_t i = 0; i < hybridJointHandles_.size(); i++)
  {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
  }
  for (int i = 0; i < hybridJointHandles_.size(); i++)
  {
    // arm j1 j2 j3
    if(i==8||i==9||i==10){ 
        scalar_t actionMin = jointPos(i) - initJointAngles_(i, 0) +
        (soleRobotCfg_.controlCfg.arm_j123_damping * jointVel(i) - soleRobotCfg_.controlCfg.arm_j123_torque_limit) /
            soleRobotCfg_.controlCfg.arm_j123_stiffness;
        scalar_t actionMax =
            jointPos(i) - initJointAngles_(i, 0) +
            (soleRobotCfg_.controlCfg.arm_j123_damping * jointVel(i) + soleRobotCfg_.controlCfg.arm_j123_torque_limit) /
                soleRobotCfg_.controlCfg.arm_j123_stiffness;
        actions_[i] = std::max(actionMin / soleRobotCfg_.controlCfg.action_scale_pos,
                            std::min(actionMax / soleRobotCfg_.controlCfg.action_scale_pos, (scalar_t)actions_[i]));
        scalar_t pos_des = actions_[i] * soleRobotCfg_.controlCfg.action_scale_pos ;
        lastActions_(i, 0) = actions_[i];
        hybridJointHandles_[i].setCommand(pos_des, 0, soleRobotCfg_.controlCfg.arm_j123_stiffness, 
                                            soleRobotCfg_.controlCfg.arm_j123_damping, 0, 0);
    }
    // arm j4 j5 j6
    else if(i==11||i==12||i==13){ 
        scalar_t actionMin = jointPos(i) - initJointAngles_(i, 0) +
        (soleRobotCfg_.controlCfg.arm_j456_damping * jointVel(i) - soleRobotCfg_.controlCfg.arm_j456_torque_limit) /
            soleRobotCfg_.controlCfg.arm_j456_stiffness;
        scalar_t actionMax =
            jointPos(i) - initJointAngles_(i, 0) +
            (soleRobotCfg_.controlCfg.arm_j456_damping * jointVel(i) + soleRobotCfg_.controlCfg.arm_j456_torque_limit) /
                soleRobotCfg_.controlCfg.arm_j456_stiffness;
        actions_[i] = std::max(actionMin / soleRobotCfg_.controlCfg.action_scale_pos,
                            std::min(actionMax / soleRobotCfg_.controlCfg.action_scale_pos, (scalar_t)actions_[i]));
        lastActions_(i, 0) = actions_[i];
        scalar_t pos_des = actions_[i] * soleRobotCfg_.controlCfg.action_scale_pos ;
        hybridJointHandles_[i].setCommand(pos_des, 0, soleRobotCfg_.controlCfg.arm_j456_stiffness, 
                                            soleRobotCfg_.controlCfg.arm_j456_damping, 0, 0);
    }
    // ankle joint
    else if(i==3||i==7){ 
        scalar_t actionMin = jointPos(i) - initJointAngles_(i, 0) +
        (soleRobotCfg_.controlCfg.ankle_joint_damping * jointVel(i) - soleRobotCfg_.controlCfg.ankle_joint_torque_limit) /
            soleRobotCfg_.controlCfg.ankle_joint_stiffness;
        scalar_t actionMax =
            jointPos(i) - initJointAngles_(i, 0) +
            (soleRobotCfg_.controlCfg.ankle_joint_damping * jointVel(i) + soleRobotCfg_.controlCfg.ankle_joint_torque_limit) /
                soleRobotCfg_.controlCfg.ankle_joint_stiffness;
        actions_[i] = std::max(actionMin / soleRobotCfg_.controlCfg.action_scale_pos,
                            std::min(actionMax / soleRobotCfg_.controlCfg.action_scale_pos, (scalar_t)actions_[i]));
        lastActions_(i, 0) = actions_[i];
        scalar_t pos_des = actions_[i] * soleRobotCfg_.controlCfg.action_scale_pos ;
        hybridJointHandles_[i].setCommand(pos_des, 0, soleRobotCfg_.controlCfg.ankle_joint_stiffness, 
                                            soleRobotCfg_.controlCfg.ankle_joint_damping, 0, 0);
    }
    // leg joint
    else if(i==0||i==1||i==2||i==4||i==5||i==6){
        scalar_t actionMin = jointPos(i) - initJointAngles_(i, 0) +
        (soleRobotCfg_.controlCfg.leg_joint_damping * jointVel(i) - soleRobotCfg_.controlCfg.leg_joint_torque_limit) /
            soleRobotCfg_.controlCfg.leg_joint_stiffness;
        scalar_t actionMax =
            jointPos(i) - initJointAngles_(i, 0) +
            (soleRobotCfg_.controlCfg.leg_joint_damping * jointVel(i) + soleRobotCfg_.controlCfg.leg_joint_torque_limit) /
                soleRobotCfg_.controlCfg.leg_joint_stiffness;
        actions_[i] = std::max(actionMin / soleRobotCfg_.controlCfg.action_scale_pos,
                            std::min(actionMax / soleRobotCfg_.controlCfg.action_scale_pos, (scalar_t)actions_[i]));
        lastActions_(i, 0) = actions_[i];
        scalar_t pos_des = actions_[i] * soleRobotCfg_.controlCfg.action_scale_pos;
        hybridJointHandles_[i].setCommand(pos_des, 0, soleRobotCfg_.controlCfg.leg_joint_stiffness, 
                                            soleRobotCfg_.controlCfg.leg_joint_damping, 0, 0);
    }
  }
}

bool SolefootController::loadModel() {
  // Load ONNX models for policy, encoder, and gait generator.

  std::string policyModelPath;
  if (!nh_.getParam("/policyFile", policyModelPath)) {
    ROS_ERROR("Failed to retrieve policy path from the parameter server!");
    return false;
  }

  std::string encoderModelPath;
  if (!nh_.getParam("/encoderFile", encoderModelPath)) {
    ROS_ERROR("Failed to retrieve encoder path from the parameter server!");
    return false;
  }

  // create env
  onnxEnvPtr_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxController"));
  // create session
  Ort::SessionOptions sessionOptions;
  sessionOptions.SetIntraOpNumThreads(1);
  sessionOptions.SetInterOpNumThreads(1);

  std::cerr << "onnxEnvPtr_.use_count = " << onnxEnvPtr_.use_count() << std::endl; 
  Ort::AllocatorWithDefaultOptions allocator;
  // policy session
  std::cout << "load policy from" << policyModelPath.c_str() << std::endl;
  policySessionPtr_ = std::make_unique<Ort::Session>(*onnxEnvPtr_, policyModelPath.c_str(), sessionOptions);
  policyInputNames_.clear();
  policyOutputNames_.clear();
  policyInputShapes_.clear();
  policyOutputShapes_.clear();
  for (int i = 0; i < policySessionPtr_->GetInputCount(); i++)
  {
    policyInputNames_.push_back(policySessionPtr_->GetInputName(i, allocator));
    policyInputShapes_.push_back(policySessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    std::cerr << policySessionPtr_->GetInputName(i, allocator) << std::endl;
    std::vector<int64_t> shape = policySessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
    std::cerr << "Shape: [";
    for (size_t j = 0; j < shape.size(); ++j)
    {
      std::cout << shape[j];
      if (j != shape.size() - 1)
      {
          std::cerr << ", ";
      }
    }
    std::cout << "]" << std::endl;
  }
  for (int i = 0; i < policySessionPtr_->GetOutputCount(); i++)
  {
    policyOutputNames_.push_back(policySessionPtr_->GetOutputName(i, allocator));
    std::cerr << policySessionPtr_->GetOutputName(i, allocator) << std::endl;
    policyOutputShapes_.push_back(
        policySessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    std::vector<int64_t> shape = policySessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
    std::cerr << "Shape: [";
    for (size_t j = 0; j < shape.size(); ++j)
    {
      std::cout << shape[j];
      if (j != shape.size() - 1)
      {
          std::cerr << ", ";
      }
    }
    std::cout << "]" << std::endl;
  }

  // encoder session
  std::cout << "load encoder from" << encoderModelPath.c_str() << std::endl;
  encoderSessionPtr_ = std::make_unique<Ort::Session>(*onnxEnvPtr_, encoderModelPath.c_str(), sessionOptions);
  encoderInputNames_.clear();
  encoderOutputNames_.clear();
  encoderInputShapes_.clear();
  encoderOutputShapes_.clear();
  for (int i = 0; i < encoderSessionPtr_->GetInputCount(); i++)
  {
    encoderInputNames_.push_back(encoderSessionPtr_->GetInputName(i, allocator));
    encoderInputShapes_.push_back(
        encoderSessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    std::cerr << encoderSessionPtr_->GetInputName(i, allocator) << std::endl;
    std::vector<int64_t> shape = encoderSessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
    std::cerr << "Shape: [";
    for (size_t j = 0; j < shape.size(); ++j)
    {
      std::cout << shape[j];
      if (j != shape.size() - 1)
      {
        std::cerr << ", ";
      }
    }
    std::cout << "]" << std::endl;
  }
  for (int i = 0; i < encoderSessionPtr_->GetOutputCount(); i++)
  {
    encoderOutputNames_.push_back(encoderSessionPtr_->GetOutputName(i, allocator));
    std::cerr << encoderSessionPtr_->GetOutputName(i, allocator) << std::endl;
    encoderOutputShapes_.push_back(
        encoderSessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    std::vector<int64_t> shape = encoderSessionPtr_->GetOutputTypeInfo(
                                                        i)
                                      .GetTensorTypeAndShapeInfo()
                                      .GetShape();
    std::cerr << "Shape: [";
    for (size_t j = 0; j < shape.size(); ++j)
    {
      std::cout << shape[j];
      if (j != shape.size() - 1)
      {
        std::cerr << ", ";
      }
    }
    std::cout << "]" << std::endl;
  }
  ROS_INFO_STREAM("Load Onnx model from successfully !!!");
  return true;
}

// Loads the reinforcement learning configuration.
bool SolefootController::loadRLCfg() {
  auto &initState = soleRobotCfg_.initState;
  auto &initStandState = soleRobotCfg_.initStandState;
  auto &controlCfg = soleRobotCfg_.controlCfg;
  auto &obsScales = soleRobotCfg_.soleRlCfg.obsScales;
  auto &gaitCfg = soleRobotCfg_.gaitCfg;
  auto &estimationCfg = soleRobotCfg_.estimationCfg;

  try {
    // Load parameters from ROS parameter server.
    int error = 0;
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/joint_names", jointNames_));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/default_joint_angle/abad_L_Joint", initState["abad_L_Joint"]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/default_joint_angle/hip_L_Joint", initState["hip_L_Joint"]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/default_joint_angle/knee_L_Joint", initState["knee_L_Joint"]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/default_joint_angle/abad_R_Joint", initState["abad_R_Joint"]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/default_joint_angle/hip_R_Joint", initState["hip_R_Joint"]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/default_joint_angle/knee_R_Joint", initState["knee_R_Joint"]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/default_joint_angle/J1", initState["J1"]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/default_joint_angle/J2", initState["J2"]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/default_joint_angle/J3", initState["J3"]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/default_joint_angle/J4", initState["J4"]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/default_joint_angle/J5", initState["J5"]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/default_joint_angle/J6", initState["J6"]));
    standDuration_ = 0.5;
    // kp, kd
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/control/leg_joint_stiffness", controlCfg.leg_joint_stiffness));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/control/leg_joint_damping", controlCfg.leg_joint_damping));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/control/ankle_joint_stiffness", controlCfg.ankle_joint_stiffness));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/control/ankle_joint_damping", controlCfg.ankle_joint_damping));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/control/arm_j123_stiffness", controlCfg.arm_j123_stiffness));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/control/arm_j123_damping", controlCfg.arm_j123_damping));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/control/arm_j456_stiffness", controlCfg.arm_j456_stiffness));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/control/arm_j456_damping", controlCfg.arm_j456_damping));
    // torque limits
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/control/leg_joint_torque_limit", controlCfg.leg_joint_torque_limit));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/control/ankle_joint_torque_limit", controlCfg.ankle_joint_torque_limit));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/control/arm_j123_torque_limit", controlCfg.arm_j123_torque_limit));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/control/arm_j456_torque_limit", controlCfg.arm_j456_torque_limit));
    // others
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/control/action_scale_pos", controlCfg.action_scale_pos));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/control/decimation", controlCfg.decimation));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/normalization/clip_scales/clip_observations", soleRobotCfg_.soleRlCfg.clipObs));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/normalization/clip_scales/clip_actions", soleRobotCfg_.soleRlCfg.clipActions));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/normalization/obs_scales/lin_vel", obsScales.linVel));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/normalization/obs_scales/ang_vel", obsScales.angVel));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/normalization/obs_scales/dof_pos", obsScales.dofPos));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/normalization/obs_scales/dof_vel", obsScales.dofVel));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/normalization/obs_scales/height_measurements", obsScales.heightMeasurements));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/size/actions_size", actionsSize_));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/size/observations_size", observationSize_));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/size/obs_history_length", obsHistoryLength_));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/size/encoder_output_size", encoderOutputSize_));

    error += static_cast<int>(!nh_.getParam("/PointfootCfg/imuOrientationOffset_/yaw", imuOrientationOffset_[0]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/imuOrientationOffset_/pitch", imuOrientationOffset_[1]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/imuOrientationOffset_/roll", imuOrientationOffset_[2]));

    error += static_cast<int>(!nh_.getParam("/PointfootCfg/user_cmd_scales/lin_vel_x", soleRobotCfg_.userCmdCfg.linVel_x));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/user_cmd_scales/lin_vel_y", soleRobotCfg_.userCmdCfg.linVel_y));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/user_cmd_scales/ang_vel_yaw", soleRobotCfg_.userCmdCfg.angVel_yaw));

    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/default_joint_angle/ankle_L_Joint", initState["ankle_L_Joint"]));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/init_state/default_joint_angle/ankle_R_Joint", initState["ankle_R_Joint"]));

    error += static_cast<int>(!nh_.getParam("/PointfootCfg/gait/frequencies", soleRobotCfg_.gaitCfg.frequencies));
    error += static_cast<int>(!nh_.getParam("/PointfootCfg/gait/swing_height", soleRobotCfg_.gaitCfg.swing_height));
    
    if (error) {
      ROS_ERROR("Load parameters from ROS parameter server error!!!");
    }
    encoderInputSize_ = obsHistoryLength_ * observationSize_;
    soleRobotCfg_.print();
    clearData();

    // Resize vectors.
    actions_.resize(actionsSize_);
    observations_.resize(observationSize_);
    proprioHistoryVector_.resize(observationSize_ * obsHistoryLength_);
    encoderOut_.resize(encoderOutputSize_);
    lastActions_.resize(actionsSize_);

    // Initialize vectors.
    lastActions_.setZero();
    commands_.setZero();
    scaledCommandsSole_.setZero();
    baseLinVel_.setZero();
    basePosition_.setZero();
  } catch (const std::exception &e) {
    // Error handling.
    ROS_ERROR("Error in the PointfootCfg: %s", e.what());
    return false;
  }
  ROS_INFO_STREAM("Load Sole Robot Cfg from successfully !!!");
  return true;
}

void SolefootController::computeObservation() {
  Eigen::Quaterniond q_wi;
  for (size_t i = 0; i < 4; ++i)
  {
    q_wi.coeffs()(i) = imuSensorHandles_.getOrientation()[i];
  }

  vector3_t zyx = quatToZyx(q_wi);
  matrix_t inverseRot = getRotationMatrixFromZyxEulerAngles(zyx).inverse();

  vector3_t gravityVector(0, 0, -1);
  vector3_t projectedGravity(inverseRot * gravityVector);

  vector3_t _zyx(0.0, 0.0, 0.0);
  matrix_t rot = getRotationMatrixFromZyxEulerAngles(_zyx);
  projectedGravity = rot * projectedGravity;

  auto &initState = soleRobotCfg_.initState;
  vector_t jointPos(initState.size());
  vector_t jointVel(initState.size());
  vector_t jointTor(initState.size());

  for (size_t i = 0; i < hybridJointHandles_.size(); ++i)
  {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
    jointTor(i) = hybridJointHandles_[i].getEffort();
  }
  
  // command
  vector_t commands = commands_;
  vector_t actions(lastActions_);

  matrix_t commandScaler = Eigen::DiagonalMatrix<scalar_t, 3>(soleRobotCfg_.userCmdCfg.linVel_x,
                                                              soleRobotCfg_.userCmdCfg.linVel_y,
                                                              soleRobotCfg_.userCmdCfg.angVel_yaw);

  // clang-format on
  commands = commandScaler * commands;
  for (size_t i = 0; i < commands.size(); i++)
  {
    scaledCommandsSole_[i] = static_cast<tensor_element_t>(commands(i));
  }

  if(commandX_.size() > windowLen_){
    commandX_.erase(commandX_.begin());
    commandY_.erase(commandY_.begin());
    commandYaw_.erase(commandYaw_.begin());
  }
  commandX_.push_back(scaledCommandsSole_[0]);
  commandY_.push_back(scaledCommandsSole_[1]);
  commandYaw_.push_back(scaledCommandsSole_[2]);

  scaledCommandsSole_[0] = sliding_window(commandX_, windowLen_);
  scaledCommandsSole_[1] = sliding_window(commandY_, windowLen_);
  scaledCommandsSole_[2] = sliding_window(commandYaw_, windowLen_);

  if(locomotionFlag_ == 0 && (std::abs(scaledCommandsSole_[0]) < 0.05 && std::abs(scaledCommandsSole_[1]) < 0.05 && std::abs(scaledCommandsSole_[2]) < 0.05)){
    standStillFlag_ = 1;
  }
  if(sitDownFlag_ == 0){
    if(locomotionFlag_ == 1 || (std::abs(scaledCommandsSole_[0]) >= 0.05 || std::abs(scaledCommandsSole_[1]) >= 0.05 || std::abs(scaledCommandsSole_[2]) >= 0.05)){
      standStillFlag_ = 0;
    }
  }else{
    standStillFlag_ = 1;
  }

  handleExtraCommands();

  double modifiedAngularVelocityZ = imuSensorHandles_.getAngularVelocity()[2];
  if(std::abs(scaledCommandsSole_[2]) < 0.1 && std::abs(modifiedAngularVelocityZ) <= 0.4){
    modifiedAngularVelocityZ = 0.0;
  }
  vector3_t baseAngVel(imuSensorHandles_.getAngularVelocity()[0], imuSensorHandles_.getAngularVelocity()[1],
                        modifiedAngularVelocityZ);

  baseAngVel = rot * baseAngVel;
  vector3_t vel_commands,ee_pos,ee_rpy;
  Eigen::Quaternion<scalar_t> ee_quat;
  vector4_t ee_quat_vec;
  matrix_t ee_mat;
  vector_t ee_mat_vec;
  ee_mat_vec.resize(6);
  ee_mat_vec.setZero();
  float base_height_cmd = 0.78;

  base_height_cmd = baseHeightCmd_;
  vel_commands << commands_[0], commands_[1], commands_[2]*4 ; 

  ee_pos << rc_ee_cmd.ee_position[0] + ee_init_pos_[0], rc_ee_cmd.ee_position[1] + ee_init_pos_[1], rc_ee_cmd.ee_position[2] + ee_init_pos_[2];
  ee_rpy << ee_init_rpy_[0] + rc_ee_cmd.ee_rpy[1], 
            ee_init_rpy_[1] + rc_ee_cmd.ee_rpy[2], 
            ee_init_rpy_[2] + rc_ee_cmd.ee_rpy[0];

  ee_quat = getQuaternionFromRpy(ee_rpy);
  if(ee_quat.w() < 0){
    ee_quat_vec << -ee_quat.w(), -ee_quat.x(), -ee_quat.y(), -ee_quat.z();
  }
  else{
    ee_quat_vec << ee_quat.w(), ee_quat.x(), ee_quat.y(), ee_quat.z();
  }

  vector3_t ee_zyx(ee_rpy[2], ee_rpy[1], ee_rpy[0]);
  ee_mat = getRotationMatrixFromZyxEulerAngles(ee_zyx);
  ee_mat_vec <<   ee_mat(0,0), ee_mat(0,1), ee_mat(0,2),
                  ee_mat(1,0), ee_mat(1,1), ee_mat(1,2);

  // clamp ee_pos
  ee_pos[0] = std::min(0.8, std::max(0.0, ee_pos[0]));
  ee_pos[1] = std::min(0.5, std::max(-0.5, ee_pos[1]));
  ee_pos[2] = std::min(0.5, std::max(-0.3, ee_pos[2]));

  ee_pos_cmd_debug_msg_.position.x = ee_pos[0];
  ee_pos_cmd_debug_msg_.position.y = ee_pos[1];
  ee_pos_cmd_debug_msg_.position.z = ee_pos[2];

  ee_pos_cmd_debug_msg_.orientation.x = ee_quat_vec[1];
  ee_pos_cmd_debug_msg_.orientation.y = ee_quat_vec[2];
  ee_pos_cmd_debug_msg_.orientation.z = ee_quat_vec[3];
  ee_pos_cmd_debug_msg_.orientation.w = ee_quat_vec[0];
  // ee_pos_cmd_debug_pub_.publish(ee_pos_cmd_debug_msg_);

  vector_t gait_command = handleGaitCommand();
  vector_t gait_phase = handleGaitPhase(gait_command);

  vector_t obs(observationSize_);

  obs <<  baseAngVel,
          projectedGravity,
          vel_commands,
          ee_pos,
          ee_mat_vec,
          base_height_cmd,
          jointPos,
          jointVel,
          jointTor,
          actions,
          gait_phase,
          gait_command;

  // prepare for one-hot-encoder
  vector_t obs_temp(observationSize_);
  obs_temp = obs;
  obs_temp[0] = 0;
  obs_temp[1] = 0;
  if (isfirstRecObs_)
  {
    int64_t inputSize = std::accumulate(encoderInputShapes_[0].begin(), encoderInputShapes_[0].end(),
                                        static_cast<int64_t>(1), std::multiplies<int64_t>());
    proprioHistoryBuffer_.resize(inputSize);
    proprioHistoryBufferForEstimation_.resize(inputSize);
    for (size_t i = 0; i < obsHistoryLength_; i++)
    {
        proprioHistoryBuffer_.segment(i * observationSize_,
                                        observationSize_) = obs.cast<tensor_element_t>();
        proprioHistoryBufferForEstimation_.segment(i * observationSize_,
                                        observationSize_) = obs_temp.cast<tensor_element_t>();
    }
    isfirstRecObs_ = false;
  }

  proprioHistoryBuffer_.head(proprioHistoryBuffer_.size() - observationSize_) = proprioHistoryBuffer_.tail(
      proprioHistoryBuffer_.size() - observationSize_);
  proprioHistoryBuffer_.tail(observationSize_) = obs.cast<tensor_element_t>();

  proprioHistoryBufferForEstimation_.head(proprioHistoryBufferForEstimation_.size() - observationSize_) = proprioHistoryBufferForEstimation_.tail(
      proprioHistoryBufferForEstimation_.size() - observationSize_);
  proprioHistoryBufferForEstimation_.tail(observationSize_) = obs_temp.cast<tensor_element_t>();

  for (size_t i = 0; i < obs.size(); i++)
  {
    observations_[i] = static_cast<tensor_element_t>(obs(i));
  }
  for (size_t i = 0; i < proprioHistoryBuffer_.size(); i++)
  {
    proprioHistoryVector_[i] = static_cast<tensor_element_t>(proprioHistoryBuffer_(i));
  }
  // Limit observation range
  scalar_t obsMin = -soleRobotCfg_.soleRlCfg.clipObs;
  scalar_t obsMax = soleRobotCfg_.soleRlCfg.clipObs;

  std::transform(observations_.begin(), observations_.end(), observations_.begin(),
                  [obsMin, obsMax](scalar_t x)
                  { return std::max(obsMin, std::min(obsMax, x)); });
}

void SolefootController::computeEncoder() {
  Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator,
    OrtMemType::OrtMemTypeDefault);
  std::vector<Ort::Value> inputValues;
  inputValues.push_back(Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, proprioHistoryBuffer_.data(),
              proprioHistoryBuffer_.size(),
              encoderInputShapes_[0].data(),
              encoderInputShapes_[0].size()));

  Ort::RunOptions runOptions;
  std::vector<Ort::Value> outputValues =
  encoderSessionPtr_->Run(runOptions, encoderInputNames_.data(), inputValues.data(), 1,
  encoderOutputNames_.data(), 1);
  for (int i = 0; i < encoderOutputSize_; i++)
  {
    encoderOut_[i] = *(outputValues[0].GetTensorMutableData<tensor_element_t>() + i);
  }
}

// Computes actions using the policy model.
void SolefootController::computeActions() {
  // create input tensor object
  Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator,
    OrtMemType::OrtMemTypeDefault);
  std::vector<Ort::Value> inputValues;
  std::vector<tensor_element_t> combined_obs;

  std::vector<tensor_element_t> latent;
  for (const auto &item : encoderOut_)
  {
    latent.push_back(item);
  }
  for (const auto &item : observations_)
  {
    combined_obs.push_back(item);
  }
  inputValues.push_back(
  Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, combined_obs.data(), combined_obs.size(),
  policyInputShapes_[0].data(), policyInputShapes_[0].size()));
  inputValues.push_back(
  Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, latent.data(), latent.size(),
  policyInputShapes_[1].data(), policyInputShapes_[1].size()));

  // run inference
  Ort::RunOptions runOptions;
  std::vector<Ort::Value> outputValues = policySessionPtr_->Run(runOptions, policyInputNames_.data(),
            inputValues.data(), 2, policyOutputNames_.data(), 1);
  // vector_t action(8);
  for (int i = 0; i < actionsSize_; i++)
  {
    actions_[i] = *(outputValues[0].GetTensorMutableData<tensor_element_t>() + i);
  }
}

void SolefootController::cmdVelCallback(const geometry_msgs::TwistConstPtr &msg) {
  commands_(0) = std::min(0.5, std::max(msg->linear.x, -0.5));
  commands_(1) = std::min(0.5, std::max(msg->linear.y, -0.5));
  commands_(2) = std::min(0.3, std::max(msg->angular.z, -0.3));

  double command_z = std::min(1.0, std::max(msg->linear.z, -1.0));
  baseHeightCmd_ = 0.8 + command_z / 5;

  imuOrientationOffset_[0] = msg->angular.x;
  imuOrientationOffset_[1] = msg->angular.y;
}

void SolefootController::clearData(){
  actions_.resize(actionsSize_);
  filtered_actions_.resize(2);
  observations_.resize(observationSize_);
  proprioHistoryVector_.resize(observationSize_ * obsHistoryLength_);
  encoderOut_.resize(encoderOutputSize_);
  oneHotEncoderOut_.resize(oneHotEncoderOutputSize_);
  lastActions_.resize(actionsSize_);
  gaitGeneratorOut_.resize(gaitGeneratorOutputSize_);
  lastActions_.setZero();
  commands_.setZero();
  extraCommands_.setZero();
  scaledCommandsSole_.setZero();
  baseLinVel_.setZero();
  basePosition_.setZero();
}

void SolefootController::handleExtraCommands(){
  scaledCommandsSole_[3] = static_cast<tensor_element_t>(extraCommands_(0));
  if(standStillFlag_ == 1){
    scaledCommandsSole_[4] = static_cast<tensor_element_t>(1);
    scaledCommandsSole_.head(3).setZero();
  } else {
    scaledCommandsSole_[4] = static_cast<tensor_element_t>(0);
  }
}

vector_t SolefootController::handleGaitCommand(){
  vector_t gait(4);
  // frequency, phase offset, contact duration, swing height
  gait << 1.3, 0.5, 0.5, 0.12;
  return gait;
}

vector_t SolefootController::handleGaitPhase(vector_t &gait){
  vector_t gait_clock(2);
  gaitIndex_ += 0.02 * gait(0);
  if (gaitIndex_ > 1.0)
  {
    gaitIndex_ = 0.0;
  }
  if(commands_.norm() < 0.01){
    gaitIndex_ = 0.0;
  }

  gait_clock << sin(gaitIndex_ * 2 * M_PI), cos(gaitIndex_ * 2 * M_PI);
  return gait_clock;
}

double SolefootController::sliding_window(std::vector<double>& data, int window_size) {
  std::vector<double> window(window_size, 1.0 / window_size);
  std::vector<double> smooth_data;
  int len = data.size();
  for (int i = 0; i < std::min(window_size - 1, len); ++i) {
    double mean = std::accumulate(data.begin(), data.begin() + i + 1, 0.0) / (i + 1);
    if(std::abs(mean) < 0.01){
      mean = 0.0;
    }else{
      mean = round(mean * 100) / 100;
    }
    smooth_data.push_back(mean);
  }

  for (int i = window_size - 1; i < data.size(); ++i) {
    double sum = 0.0;
    for (int j = i - window_size + 1; j <= i; ++j) {
      sum += data[j] * window[i - j];
    }
    if(std::abs(sum) < 0.01){
      sum = 0.0;
    }else{
      sum = round(sum * 100) / 100;
    }
    smooth_data.push_back(sum);
  }
  return smooth_data.back();
}

void SolefootController::EEPoseCmdRCCallback(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  ee_pos_cmd_rc_delta_msg_ = *msg;
  rc_ee_cmd.ee_position[0] += msg->data[0];
  rc_ee_cmd.ee_position[1] += msg->data[1];
  rc_ee_cmd.ee_position[2] += msg->data[2];

  rc_ee_cmd.ee_position[0] = std::min(0.8-ee_init_pos_[0], std::max(0.0-ee_init_pos_[0], rc_ee_cmd.ee_position[0]));
  rc_ee_cmd.ee_position[1] = std::min(0.5-ee_init_pos_[1], std::max(-0.5-ee_init_pos_[1], rc_ee_cmd.ee_position[1]));
  rc_ee_cmd.ee_position[2] = std::min(0.5-ee_init_pos_[2], std::max(-0.3-ee_init_pos_[2], rc_ee_cmd.ee_position[2]));

  rc_ee_cmd.ee_rpy[0] += msg->data[3];
  rc_ee_cmd.ee_rpy[1] += msg->data[4];
  rc_ee_cmd.ee_rpy[2] += msg->data[5];

  rc_ee_cmd.gripper_cmd = msg->data[6];

  gripper_cmd_msg_.data = rc_ee_cmd.gripper_cmd;
  
  gripper_cmd_pub_.publish(gripper_cmd_msg_);
}
} // namespace

// Export the class as a plugin.
PLUGINLIB_EXPORT_CLASS(robot_controller::SolefootController, controller_interface::ControllerBase)
