// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#include "robot_controllers/WheelfootController.h"

#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <numeric>

namespace robot_controller {

// Initialize the controller
bool WheelfootController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh) {
  wheel_stand_pos_.resize(8+6);
  wheel_stand_pos_<<  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      1.36,-1.36, //knee_L, knee_R
                      0.0, 0.0, 0.0, 0.0, 0.0;

  eeCommands_.resize(9); // x y z xx xy xz yx yy yz
  eeCommands_ << 0.346+0.1, 0, 0.241, 0, 0, 1, -1, 0, 0;

  ee_init_pos_ << 0.346+0.1, 0, 0.241;
  ee_init_rpy_ << -M_PI/2, 0, -M_PI/2;

  ee_init_ori_<<   0,  0,  1,
                  -1,  0,  0,
                    0, -1,  0;

  gripper_cmd_pub_ = nh_.advertise<std_msgs::Bool>("/gripper_cmd", false, 10);

  ee_pos_cmd_rc_delta_ = nh_.subscribe<std_msgs::Float32MultiArray>("/EEPose_cmd_rc", 10, &WheelfootController::EEPoseCmdRCCallback, this);
  ee_pos_cmd_rc_delta_msg_.data.resize(6+1);
  rc_ee_cmd.zero();

  return ControllerBase::init(robot_hw, nh);
}

// Perform initialization when the controller starts
void WheelfootController::starting(const ros::Time &time) {
  for (size_t i = 0; i < hybridJointHandles_.size(); i++) {
    ROS_INFO_STREAM("starting hybridJointHandle: " << hybridJointHandles_[i].getPosition());
    defaultJointAngles_[i] = hybridJointHandles_[i].getPosition();
  }

  scalar_t durationSecs = 1.5;
  standDuration_ = durationSecs * 1000.0;
  standPercent_ = 0.0;
  loopCount_ = 0;
  work_mode_flag_ = 10;
  mode_ = Mode::WHEEL_STAND;
}

// Update function called periodically
void WheelfootController::update(const ros::Time &time, const ros::Duration &period) {
  switch (mode_)
  {
  case Mode::WHEEL_STAND:
    handleWheelStandMode();
    break;
  case Mode::WHEEL_WALK:
    handleRLWheelMoveMode();
    break;
  }
  loopCount_++;
  last_mode_ = mode_;
}

void WheelfootController::handleWheelStandMode()
{
  static bool is_arm_stand = false;
  if (standPercent_ < 1 && !is_arm_stand)
  {
    for (int j = 0; j < hybridJointHandles_.size(); j++)
    {
      if (j==12||j==13) // wheel
      {
        hybridJointHandles_[j].setCommand(0, 0, 0, 1, 0, 0);
      }

      else if(j==9||j==10||j==11) // j456
      {
        scalar_t pos_des = defaultJointAngles_[j] * (1 - standPercent_) + wheel_stand_pos_[j] * standPercent_;
        hybridJointHandles_[j].setCommand(pos_des, 0, 10, 1, 0, 0);
      }
      else if (j==0||j==3||j==6) // j123
      {
        scalar_t pos_des = defaultJointAngles_[j] * (1 - standPercent_) + wheel_stand_pos_[j] * standPercent_;
        hybridJointHandles_[j].setCommand(pos_des, 0, 10, 1, 0, 0);
      }
      else // if(j==1||j==2||j==4||j==5||j==7||j==8)
      {
        scalar_t pos_des = defaultJointAngles_[j] * (1 - standPercent_) + wheel_stand_pos_[j] * standPercent_;
        hybridJointHandles_[j].setCommand(pos_des, 0, 50, 6, 0, 0);
      }
    }
    standPercent_ += 1 / standDuration_;
    if(standPercent_ >= 1)
    {
      is_arm_stand = true;
      mode_ = Mode::WHEEL_WALK;
    }
  }
}

void WheelfootController::handleRLWheelMoveMode()
{
  if (work_mode_flag_ != 2)
  {
    ROS_INFO_STREAM("------------------RL Move------------------");
    policy_path_ = "policy_move";
    loadRLCfg();
    loadModel();
    isfirstRecObs_ = true;
    work_mode_flag_ = 2;

  }
  // compute observation & actions
  if (wheelRobotCfg_.controlCfg.decimation == 0)
  {
    std::cerr << "----error----  wheelRobotCfg_.controlCfg.decimation" << std::endl;
    return;
  }

  if (loopCount_ % wheelRobotCfg_.controlCfg.decimation == 0)
  {
    computeObservation();
    computeEncoder();
    computeActions();
  }

  // set action
  std_msgs::Float32MultiArray debugInfoArray;
  debugInfoArray.data.resize(hybridJointHandles_.size() * 6);
  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
  for (size_t i = 0; i < hybridJointHandles_.size(); i++)
  {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
  }
  for (int i = 0; i < hybridJointHandles_.size(); i++)
  {
    if(i ==12 || i==13) { // wheel
      scalar_t actionMin = (jointVel(i) - wheelRobotCfg_.controlCfg.wheel_joint_torque_limit / wheelRobotCfg_.controlCfg.wheel_joint_damping);
      scalar_t actionMax = (jointVel(i) + wheelRobotCfg_.controlCfg.wheel_joint_torque_limit / wheelRobotCfg_.controlCfg.wheel_joint_damping);
      
      actions_[i] = std::max(actionMin / wheelRobotCfg_.controlCfg.action_scale_vel,
                    std::min(actionMax / wheelRobotCfg_.controlCfg.action_scale_vel, (scalar_t) actions_[i]));
      scalar_t velocity_des = actions_[i] * wheelRobotCfg_.controlCfg.action_scale_vel;

      lastActions_(i, 0) = actions_[i];
      hybridJointHandles_[i].setCommand(0, velocity_des, 0, wheelRobotCfg_.controlCfg.wheel_joint_damping, 0, 0);

      // debug
      debugInfoArray.data[i * 6] = velocity_des;
      debugInfoArray.data[i * 6 + 1] = hybridJointHandles_[i].getPosition();
      debugInfoArray.data[i * 6 + 2] = debugInfoArray.data[i * 6] - debugInfoArray.data[i * 6 + 1];
      debugInfoArray.data[i * 6 + 3] = hybridJointHandles_[i].getVelocity();
      debugInfoArray.data[i * 6 + 4] = hybridJointHandles_[i].getEffort();
    }
    // arm j123
    else if(i==0||i==3||i==6){
      scalar_t actionMin =
          jointPos(i) - initJointAngles_(i, 0) +
          ( wheelRobotCfg_.controlCfg.arm_j123_damping* jointVel(i) - wheelRobotCfg_.controlCfg.arm_j123_torque_limit) / wheelRobotCfg_.controlCfg.arm_j123_stiffness;
      scalar_t actionMax =
          jointPos(i) - initJointAngles_(i, 0) +
          (wheelRobotCfg_.controlCfg.arm_j123_damping * jointVel(i) + wheelRobotCfg_.controlCfg.arm_j123_torque_limit) / wheelRobotCfg_.controlCfg.arm_j123_stiffness;
      
      actions_[i] = std::max(actionMin / wheelRobotCfg_.controlCfg.action_scale_pos,
                              std::min(actionMax / wheelRobotCfg_.controlCfg.action_scale_pos, (scalar_t)actions_[i]));
      scalar_t pos_des = actions_[i] * wheelRobotCfg_.controlCfg.action_scale_pos + initJointAngles_(i, 0);

      hybridJointHandles_[i].setCommand(pos_des, 0, wheelRobotCfg_.controlCfg.arm_j123_stiffness, wheelRobotCfg_.controlCfg.arm_j123_damping,
                                          0, 2);
      lastActions_(i, 0) = actions_[i];

      // debug
      debugInfoArray.data[i * 6] = pos_des;
      debugInfoArray.data[i * 6 + 1] = hybridJointHandles_[i].getPosition();
      debugInfoArray.data[i * 6 + 2] = debugInfoArray.data[i * 6] - debugInfoArray.data[i * 6 + 1];
      debugInfoArray.data[i * 6 + 3] = hybridJointHandles_[i].getVelocity();
      debugInfoArray.data[i * 6 + 4] = hybridJointHandles_[i].getEffort();
      debugInfoArray.data[i * 6 + 5] =
          wheelRobotCfg_.controlCfg.arm_j123_stiffness * (pos_des - debugInfoArray.data[i * 6 + 1]) -
          wheelRobotCfg_.controlCfg.arm_j123_damping * debugInfoArray.data[i * 6 + 3];
    }
    // arm j456
    else if(i==9||i==10||i==11){
      scalar_t actionMin =
          jointPos(i) - initJointAngles_(i, 0) +
          ( wheelRobotCfg_.controlCfg.arm_j456_damping* jointVel(i) - wheelRobotCfg_.controlCfg.arm_j456_torque_limit) / wheelRobotCfg_.controlCfg.arm_j456_stiffness;
      scalar_t actionMax =
          jointPos(i) - initJointAngles_(i, 0) +
          (wheelRobotCfg_.controlCfg.arm_j456_damping * jointVel(i) + wheelRobotCfg_.controlCfg.arm_j456_torque_limit) / wheelRobotCfg_.controlCfg.arm_j456_stiffness;
      
      actions_[i] = std::max(actionMin / wheelRobotCfg_.controlCfg.action_scale_pos,
                              std::min(actionMax / wheelRobotCfg_.controlCfg.action_scale_pos, (scalar_t)actions_[i]));
      scalar_t pos_des = actions_[i] * wheelRobotCfg_.controlCfg.action_scale_pos + initJointAngles_(i, 0);

      hybridJointHandles_[i].setCommand(pos_des, 0, wheelRobotCfg_.controlCfg.arm_j456_stiffness, wheelRobotCfg_.controlCfg.arm_j456_damping,
                                          0, 2);
      lastActions_(i, 0) = actions_[i];

      //debug
      debugInfoArray.data[i * 6] = pos_des;
      debugInfoArray.data[i * 6 + 1] = hybridJointHandles_[i].getPosition();
      debugInfoArray.data[i * 6 + 2] = debugInfoArray.data[i * 6] - debugInfoArray.data[i * 6 + 1];
      debugInfoArray.data[i * 6 + 3] = hybridJointHandles_[i].getVelocity();
      debugInfoArray.data[i * 6 + 4] = hybridJointHandles_[i].getEffort();
      debugInfoArray.data[i * 6 + 5] =
          wheelRobotCfg_.controlCfg.arm_j456_stiffness * (pos_des - debugInfoArray.data[i * 6 + 1]) -
          wheelRobotCfg_.controlCfg.arm_j456_damping * debugInfoArray.data[i * 6 + 3];
    }
    //leg joint
    else // if (i==1 || i==2 || i==4 || i==5 || i==7 || i==8) { 
    {
      scalar_t actionMin =
        jointPos(i) - initJointAngles_(i, 0) +
        (wheelRobotCfg_.controlCfg.leg_joint_damping * jointVel(i) - wheelRobotCfg_.controlCfg.leg_joint_torque_limit) /
            wheelRobotCfg_.controlCfg.leg_joint_stiffness;
      scalar_t actionMax =
        jointPos(i) - initJointAngles_(i, 0) +
        (wheelRobotCfg_.controlCfg.leg_joint_damping * jointVel(i) + wheelRobotCfg_.controlCfg.leg_joint_torque_limit) /
            wheelRobotCfg_.controlCfg.leg_joint_stiffness;
      actions_[i] = std::max(actionMin / wheelRobotCfg_.controlCfg.action_scale_pos,
                          std::min(actionMax / wheelRobotCfg_.controlCfg.action_scale_pos, (scalar_t)actions_[i]));
      scalar_t pos_des = actions_[i] * wheelRobotCfg_.controlCfg.action_scale_pos + initJointAngles_(i, 0);
      if(!policy_move_damping_mode_){
        hybridJointHandles_[i].setCommand(pos_des, 0, wheelRobotCfg_.controlCfg.leg_joint_stiffness, wheelRobotCfg_.controlCfg.leg_joint_damping,
                                      0, 0);
      }
      else                                              
        hybridJointHandles_[i].setCommand(0, 0, 0, wheelRobotCfg_.controlCfg.leg_joint_damping,
                                      0, 0);

      lastActions_(i, 0) = actions_[i];
      debugInfoArray.data[i * 6] = pos_des;
      debugInfoArray.data[i * 6 + 1] = hybridJointHandles_[i].getPosition();
      debugInfoArray.data[i * 6 + 2] = debugInfoArray.data[i * 6] - debugInfoArray.data[i * 6 + 1];
      debugInfoArray.data[i * 6 + 3] = hybridJointHandles_[i].getVelocity();
      debugInfoArray.data[i * 6 + 4] = hybridJointHandles_[i].getEffort();
      debugInfoArray.data[i * 6 + 5] =
        wheelRobotCfg_.controlCfg.leg_joint_stiffness * (pos_des - debugInfoArray.data[i * 6 + 1]) -
        wheelRobotCfg_.controlCfg.leg_joint_damping * debugInfoArray.data[i * 6 + 3];
    }
  }
  debugPub_.publish(debugInfoArray);
}
//
bool WheelfootController::loadModel() {
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
  ROS_INFO_STREAM("Successfully loaded ONNX models!");
  return true;
}

// Loads the reinforcement learning configuration.
bool WheelfootController::loadRLCfg() {
  auto &initState = wheelRobotCfg_.initState;
  auto &controlCfg = wheelRobotCfg_.controlCfg;
  auto &obsScales = wheelRobotCfg_.wheelRlCfg.obsScales;

  try {
    // Load parameters from ROS parameter server.
    int error = 0;
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/joint_names", jointNames_));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/init_state/default_joint_angle/abad_L_Joint", initState["abad_L_Joint"]));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/init_state/default_joint_angle/hip_L_Joint", initState["hip_L_Joint"]));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/init_state/default_joint_angle/knee_L_Joint", initState["knee_L_Joint"]));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/init_state/default_joint_angle/abad_R_Joint", initState["abad_R_Joint"]));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/init_state/default_joint_angle/hip_R_Joint", initState["hip_R_Joint"]));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/init_state/default_joint_angle/knee_R_Joint", initState["knee_R_Joint"]));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/init_state/default_joint_angle/wheel_L_Joint", initState["wheel_L_Joint"]));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/init_state/default_joint_angle/wheel_R_Joint", initState["wheel_R_Joint"]));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/init_state/default_joint_angle/J1", initState["J1"]));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/init_state/default_joint_angle/J2", initState["J2"]));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/init_state/default_joint_angle/J3", initState["J3"]));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/init_state/default_joint_angle/J4", initState["J4"]));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/init_state/default_joint_angle/J5", initState["J5"]));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/init_state/default_joint_angle/J6", initState["J6"]));
    // kp, kd
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/control/leg_joint_stiffness", controlCfg.leg_joint_stiffness));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/control/leg_joint_damping", controlCfg.leg_joint_damping));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/control/wheel_joint_stiffness", controlCfg.wheel_joint_stiffness));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/control/wheel_joint_damping", controlCfg.wheel_joint_damping));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/control/arm_j123_stiffness", controlCfg.arm_j123_stiffness));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/control/arm_j123_damping", controlCfg.arm_j123_damping));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/control/arm_j456_stiffness", controlCfg.arm_j456_stiffness));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/control/arm_j456_damping", controlCfg.arm_j456_damping));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/control/action_scale_pos", controlCfg.action_scale_pos));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/control/action_scale_vel", controlCfg.action_scale_vel));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/control/decimation", controlCfg.decimation));
    // torque limits
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/control/leg_joint_torque_limit", controlCfg.leg_joint_torque_limit));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/control/wheel_joint_torque_limit", controlCfg.wheel_joint_torque_limit));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/control/arm_j123_torque_limit", controlCfg.arm_j123_torque_limit));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/control/arm_j456_torque_limit", controlCfg.arm_j456_torque_limit));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/normalization/clip_scales/clip_observations", wheelRobotCfg_.wheelRlCfg.clipObs));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/normalization/clip_scales/clip_actions", wheelRobotCfg_.wheelRlCfg.clipActions));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/normalization/obs_scales/lin_vel", obsScales.linVel));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/normalization/obs_scales/ang_vel", obsScales.angVel));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/normalization/obs_scales/dof_pos", obsScales.dofPos));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/normalization/obs_scales/dof_vel", obsScales.dofVel));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/normalization/obs_scales/height_measurements", obsScales.heightMeasurements));

    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/size/actions_size", actionsSize_));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/size/commands_size", commandsSize_));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/size/observations_size", observationSize_));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/size/obs_history_length", obsHistoryLength_));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/size/encoder_output_size", encoderOutputSize_));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/size/gait_generator_output_size", gaitGeneratorOutputSize_));

    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/imu_orientation_offset/yaw", imu_orientation_offset[0]));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/imu_orientation_offset/pitch", imu_orientation_offset[1]));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/imu_orientation_offset/roll", imu_orientation_offset[2]));

    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/user_cmd_scales/lin_vel_x", wheelRobotCfg_.userCmdCfg.linVel_x));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/user_cmd_scales/lin_vel_y", wheelRobotCfg_.userCmdCfg.linVel_y));
    error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/user_cmd_scales/ang_vel_yaw", wheelRobotCfg_.userCmdCfg.angVel_yaw));

    if(policy_path_ == "policy_move"){
      policy_move_damping_mode_ = false;
    }
    else{
      obsScales.dofTor = 0;
    }

    if (policy_path_ == "policy_move")
    {
      error += static_cast<int>(!nh_.getParam("/LeggedRobotCfg/user_cmd_scales/base_height", wheelRobotCfg_.userCmdCfg.base_height));
    }
    const char *is_sim = ::getenv("IS_SIM");
    if (is_sim && strlen(is_sim) > 0)
    {
      try
      {
        isSim_ = std::atoi(is_sim);
        ROS_INFO_STREAM("isSim_: " << isSim_);
      }
      catch (const std::exception &e)
      {
        ROS_ERROR_STREAM("IS_SIM: " << e.what());
      }
    }
    if (error) {
      ROS_ERROR("Load parameters from ROS parameter server error!!!");
    }

    encoderInputSize_ = obsHistoryLength_ * observationSize_;
    wheelRobotCfg_.print();
    actions_.resize(actionsSize_, 0.0);
    observations_.resize(observationSize_, 0.0);
    proprioHistoryVector_.resize(observationSize_ * obsHistoryLength_, 0.0);
    encoderOut_.resize(encoderOutputSize_, 0.0);
    lastActions_.resize(actionsSize_);
    
    lastActions_.setZero();
    commands_.setZero();
    commandsStand_.setZero();
    commandsMove_.setZero();
  } catch (const std::exception &e) {
    // Error handling.
    ROS_ERROR("Error in the LeggedRobotCfg: %s", e.what());
    return false;
  }
  return true;
}

void WheelfootController::computeObservation() {
  Eigen::Quaterniond q_wi;
  for (size_t i = 0; i < 4; ++i)
  {
    q_wi.coeffs()(i) = imuSensorHandles_.getOrientation()[i];
  }

  vector3_t zyx = quatToZyx(q_wi);
  matrix_t inverseRot = getRotationMatrixFromZyxEulerAngles(zyx).inverse();

  vector3_t gravityVector(0, 0, -1);
  vector3_t projectedGravity(inverseRot * gravityVector);

  vector3_t baseAngVel(imuSensorHandles_.getAngularVelocity()[0], imuSensorHandles_.getAngularVelocity()[1],
                        imuSensorHandles_.getAngularVelocity()[2]);

  // imu offset, to be added to baseAngVel
  imu_orientation_offset[1] = 0.015; //0.01;
  imu_orientation_offset[0] = 0.0;
  vector3_t _zyx(0.0, imu_orientation_offset[1], imu_orientation_offset[0]);

  matrix_t rot = getRotationMatrixFromZyxEulerAngles(_zyx);
  baseAngVel = rot * baseAngVel;
  projectedGravity = rot * projectedGravity;

  auto &initState = wheelRobotCfg_.initState;
  vector_t jointPos(initState.size());
  vector_t jointVel(initState.size());
  vector_t jointTor(initState.size());

  for (size_t i = 0; i < hybridJointHandles_.size(); ++i)
  {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
    jointTor(i) = hybridJointHandles_[i].getEffort();
  }

  vector3_t commands = commands_;
  vector4_t commandsStand = commandsStand_;
  vector4_t commandsMove = commandsMove_;

  vector_t actions(lastActions_);
  vector3_t commandScaler;
  commandScaler.setZero();
  commandScaler << wheelRobotCfg_.userCmdCfg.linVel_x,
                    wheelRobotCfg_.userCmdCfg.linVel_y,
                    wheelRobotCfg_.userCmdCfg.angVel_yaw;

  vector4_t commandScalerStandSit;
  commandScalerStandSit.setZero();
  commandScalerStandSit << wheelRobotCfg_.userCmdCfg.linVel_x,
                            wheelRobotCfg_.userCmdCfg.linVel_y,
                            wheelRobotCfg_.userCmdCfg.angVel_yaw,
                            scalar_t(1);
  
  vector4_t commandScalerMove;
  commandScalerMove.setZero();
  commandScalerMove << wheelRobotCfg_.userCmdCfg.linVel_x,
                        wheelRobotCfg_.userCmdCfg.linVel_y,
                        wheelRobotCfg_.userCmdCfg.angVel_yaw,
                        wheelRobotCfg_.userCmdCfg.base_height;

  vector_t joint_pos(actionsSize_-2),
            joint_vel(actionsSize_),
            joint_tor(actionsSize_);

  joint_pos.setZero();
  joint_pos << jointPos[0] - initJointAngles_[0],
                jointPos[1] - initJointAngles_[1],
                jointPos[2] - initJointAngles_[2],
                jointPos[3] - initJointAngles_[3],
                jointPos[4] - initJointAngles_[4],
                jointPos[5] - initJointAngles_[5],
                jointPos[6] - initJointAngles_[6],
                jointPos[7] - initJointAngles_[7],
                jointPos[8] - initJointAngles_[8],
                jointPos[9] - initJointAngles_[9],
                jointPos[10] - initJointAngles_[10],
                jointPos[11] - initJointAngles_[11]; // no 12 13

  joint_vel.setZero();
  joint_vel = jointVel;
  joint_vel<< jointVel[0], 
              jointVel[1],
              jointVel[2],
              jointVel[3],
              jointVel[4],
              jointVel[5],
              jointVel[6],
              jointVel[7],
              jointVel[8],
              jointVel[9],
              jointVel[12],
              jointVel[13],
              jointVel[10],
              jointVel[11];
  
  joint_tor.setZero();
  joint_tor = jointTor;
  joint_tor <<jointTor[0],
              jointTor[1],
              jointTor[2],
              jointTor[3],
              jointTor[4],
              jointTor[5],
              jointTor[6],
              jointTor[7],
              jointTor[8],
              jointTor[9],
              jointTor[12],
              jointTor[13],
              jointTor[10],
              jointTor[11];

  // clang-format off
  vector_t obs(observationSize_);
  obs.setZero();

  vector3_t ee_pos, ee_rpy;
  Eigen::Quaternion<scalar_t> ee_quat;
  vector4_t ee_quat_vec;
  Eigen::Matrix3d ee_mat;
  vector_t ee_mat_vec;
  ee_mat_vec.resize(6);
  ee_mat_vec.setZero();
  float base_height_cmd=0.8;

  commands << commandsMove[0], commandsMove[1], commandsMove[2]*4 ; 
  base_height_cmd = commandsMove[3];
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

  int zero_vel_cmd = 0;
  if(commands.norm() < 0.05){
    zero_vel_cmd = 1;
  }
  
  obs << baseAngVel , //* wheelRobotCfg_.obsScales.angVel,                     //
      projectedGravity,
      commands, 
      zero_vel_cmd,
      ee_pos,
      ee_mat_vec,
      base_height_cmd,
      joint_pos ,//* wheelRobotCfg_.obsScales.dofPos,  //
      joint_vel ,//* wheelRobotCfg_.obsScales.dofVel,                          //
      joint_tor ,
      actions;

  for (int i = 0; i < commands.size(); ++i)
  {
    commands[i] = commandScaler[i] * commands[i];
  }
  for(int i = 0; i < commandsStand.size(); ++i)
  {
    commandsStand[i] = commandScalerStandSit[i] * commandsStand[i];
    commandsMove[i] = commandScalerMove[i] * commandsMove[i];
  }
  if (isfirstRecObs_)
  {
      int64_t inputSize = std::accumulate(encoderInputShapes_[0].begin(), encoderInputShapes_[0].end(),
                                          static_cast<int64_t>(1), std::multiplies<int64_t>());
      proprioHistoryBuffer_.resize(inputSize);
      for (size_t i = 0; i < obsHistoryLength_; i++)
      {
        proprioHistoryBuffer_.segment(i * observationSize_,
                                        observationSize_) = obs.cast<tensor_element_t>();
      }
      isfirstRecObs_ = false;
  }

  proprioHistoryBuffer_.head(proprioHistoryBuffer_.size() - observationSize_) = proprioHistoryBuffer_.tail(
      proprioHistoryBuffer_.size() - observationSize_);
  proprioHistoryBuffer_.tail(observationSize_) = obs.cast<tensor_element_t>();

  for (size_t i = 0; i < obs.size(); i++)
  {
    observations_[i] = static_cast<tensor_element_t>(obs(i));
  }
  for (size_t i = 0; i < proprioHistoryBuffer_.size(); i++)
  {
    proprioHistoryVector_[i] = static_cast<tensor_element_t>(proprioHistoryBuffer_(i));
  }

}

void WheelfootController::computeEncoder() {
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
void WheelfootController::computeActions() {
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
            inputValues.data(), 2, policyOutputNames_.data(),
            1);

  for (int i = 0; i < actionsSize_; i++)
  {
    actions_[i] = *(outputValues[0].GetTensorMutableData<tensor_element_t>() + i);
  }
}

void WheelfootController::cmdVelCallback(const geometry_msgs::TwistConstPtr &msg) {
  double command_x = (msg->linear.x > 1.0) ? 1.0 : (msg->linear.x < -1.0 ? -1.0 : msg->linear.x);
  double command_y = (msg->linear.y > 1.0) ? 1.0 : (msg->linear.y < -1.0 ? -1.0 : msg->linear.y);
  double command_yaw = (msg->angular.z > 1.0) ? 1.0 : (msg->angular.z < -1.0 ? -1.0 : msg->angular.z);
  double command_z = (msg->linear.z > 1.0) ? 1.0 : (msg->linear.z < -1.0 ? -1.0 : msg->linear.z);
  if (mode_ == Mode::WHEEL_WALK && (policy_path_ == "policy_pre_stand" || policy_path_ == "policy_stand"))
  {
    commandsStand_(0) = 0.0;
    commandsStand_(1) = 0.0;
    commandsStand_(2) = 0.0;
    commandsStand_(3) = 0.65;
  }

  if (mode_ == Mode::WHEEL_WALK && policy_path_ == "policy_move")
  {
    commandsMove_(0) = command_x;
    commandsMove_(1) = 0.0;
    commandsMove_(2) = command_yaw;
    commandsMove_(3) = 0.75 + command_z / 5;
  }
  
  imu_orientation_offset[0] = msg->angular.x;
  imu_orientation_offset[1] = msg->angular.y;
}

// void WheelfootController::EEPoseCmdCallback(const geometry_msgs::PoseConstPtr &msg)
// {
//   vector_t ee_commands;

//   ee_commands.resize(7);
//   ee_commands << msg->position.x, msg->position.y, msg->position.z,
//                   msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;

// }

void WheelfootController::EEPoseCmdRCCallback(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  ee_pos_cmd_rc_delta_msg_ = *msg;

  // prevent ee offset in standing up
  if(mode_ == Mode::WHEEL_WALK)
  {
    rc_ee_cmd.ee_position[0] += msg->data[0];
    rc_ee_cmd.ee_position[1] += msg->data[1];
    rc_ee_cmd.ee_position[2] += msg->data[2];

    rc_ee_cmd.ee_rpy[0] += msg->data[3];
    rc_ee_cmd.ee_rpy[1] += msg->data[4];
    rc_ee_cmd.ee_rpy[2] += msg->data[5];

    rc_ee_cmd.gripper_cmd = msg->data[6]; 
  }
  gripper_cmd_msg_.data = rc_ee_cmd.gripper_cmd;
  
  gripper_cmd_pub_.publish(gripper_cmd_msg_);
}

} // namespace

// Export the class as a plugin.
PLUGINLIB_EXPORT_CLASS(robot_controller::WheelfootController, controller_interface::ControllerBase)
