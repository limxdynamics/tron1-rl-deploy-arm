// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_SOLEFOOT_CONTROLLER_H_
#define _LIMX_SOLEFOOT_CONTROLLER_H_

#include <iostream>
#include <thread>
#include <fstream>
#include <csignal>
#include <random>
#include <chrono>
#include <cstdint>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "robot_controllers/ControllerBase.h"
#include "limxsdk/pointfoot.h"

namespace robot_controller {

// Class for controlling a biped robot with point foot
class SolefootController : public ControllerBase {
  using tensor_element_t = float; // Type alias for tensor elements
  using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>; // Type alias for matrices

public:
  SolefootController() = default; // Default constructor

  ~SolefootController() override = default; // Destructor

  // Enumeration for controller modes
  enum class Mode : uint8_t {
    SOLE_STAND, 
    SOLE_WALK, 
  };

  // Initialize the controller
  bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh) override;

  // Perform actions when the controller starts
  void starting(const ros::Time &time) override;

  // Update the controller
  void update(const ros::Time &time, const ros::Duration &period) override;

protected:
  // Load the model for the controller
  bool loadModel() override;

  // Load RL configuration settings
  bool loadRLCfg() override;

  // Compute actions for the controller
  void computeActions() override;

  // Compute observations for the controller
  void computeObservation() override;

  // Compute encoder for the controller
  void computeEncoder() override;

  void handleSoleStandMode() override;

  void handleRLSoleWalkMode() override;

  void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg) override;
  void EEPoseCmdRCCallback(const std_msgs::Float32MultiArrayConstPtr &msg);

  void handleExtraCommands();

  // compute gait
  vector_t handleGaitCommand();

  // compute gait phase
  vector_t handleGaitPhase(vector_t &gait);

  Mode mode_; // Controller mode
  Mode last_mode_;

  int work_mode_flag_{10};

private:
  double sliding_window(std::vector<double>& data, int window_size);

  void clearData();

  struct RCEECmd{
      Eigen::Vector3d ee_position;
      Eigen::Vector3d ee_rpy;
      bool gripper_cmd{false};
      void zero() {
          ee_position << 0, 0, 0;
          ee_rpy << 0, 0, 0;
      }
  };

  vector3_t ee_init_pos_;
  vector3_t ee_init_rpy_;

  ros::Publisher gripper_cmd_pub_;
  std_msgs::Bool gripper_cmd_msg_;

  geometry_msgs::Pose ee_pos_cmd_debug_msg_;

  ros::Subscriber ee_pos_cmd_rc_delta_;
  std_msgs::Float32MultiArray ee_pos_cmd_rc_delta_msg_;
  RCEECmd rc_ee_cmd;
  // File path for policy model
  std::string policyFilePath_;

  std::shared_ptr<Ort::Env> onnxEnvPtr_; // Shared pointer to ONNX environment

  // ONNX session pointers
  std::unique_ptr<Ort::Session> encoderSessionPtr_;
  std::vector<const char *> encoderInputNames_;
  std::vector<const char *> encoderOutputNames_;

  // Names and shapes of inputs and outputs for ONNX sessions
  std::vector<std::vector<int64_t>> policyInputShapes_;
  std::vector<std::vector<int64_t>> policyOutputShapes_;

  std::unique_ptr<Ort::Session> policySessionPtr_;
  std::vector<const char *> policyInputNames_;
  std::vector<const char *> policyOutputNames_;
  std::vector<std::vector<int64_t>> encoderInputShapes_;
  std::vector<std::vector<int64_t>> encoderOutputShapes_;

  std::unique_ptr<Ort::Session> gaitGeneratorSessionPtr_;
  std::vector<const char *> gaitGeneratorInputNames_;
  std::vector<const char *> gaitGeneratorOutputNames_;
  std::vector<std::vector<int64_t>> gaitGeneratorInputShapes_;
  std::vector<std::vector<int64_t>> gaitGeneratorOutputShapes_;

  std::vector<tensor_element_t> proprioHistoryVector_;
  Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> proprioHistoryBuffer_;
  Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> proprioHistoryBufferForEstimation_;

  bool isfirstRecObs_{true};
  int encoderInputSize_, encoderOutputSize_;
  int oneHotEncoderInputSize_, oneHotEncoderOutputSize_;

  vector3_t baseLinVel_;
  vector3_t basePosition_;
  vector_t lastActions_;

  double baseHeightCmd_ = 0.7; 

  int actionsSize_;
  int observationSize_;
  int obsHistoryLength_;
  int gaitGeneratorOutputSize_;
  float imuOrientationOffset_[3];
  std::vector<tensor_element_t> actions_;
  std::vector<tensor_element_t> filtered_actions_;
  std::vector<tensor_element_t> observations_;
  std::vector<tensor_element_t> encoderOut_;
  std::vector<tensor_element_t> oneHotEncoderOut_;
  std::vector<tensor_element_t> gaitGeneratorOut_;

  double gaitIndex_{0.0};

  vector3_t extraCommands_;
  vector5_t scaledCommandsSole_;

  int locomotionFlag_ = 0;
  int standStillFlag_ = 0;
  int sitDownFlag_ = 0;

  std::vector<double> commandX_;
  std::vector<double> commandY_;
  std::vector<double> commandYaw_;
  const int windowLen_ = 50;

  // stand pos
  vector_t standCenterPos_;
  vector_t standJointPos_;
  scalar_t standCenterPercent_;
  scalar_t standCenterDuration_;
  scalar_t initStandPercent_;
  scalar_t initStandDuration_;

  bool needDamping_{false};
  bool isNoCommand_{false};
};

} // namespace robot_controller
#endif //_LIMX_SOLEFOOT_CONTROLLER_H_
