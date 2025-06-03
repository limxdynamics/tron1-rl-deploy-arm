// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_WHEELFOOT_CONTROLLER_H_
#define _LIMX_WHEELFOOT_CONTROLLER_H_

#include "ros/ros.h"
#include "robot_controllers/ControllerBase.h"
#include "limxsdk/pointfoot.h"

namespace robot_controller {

// Class for controlling a biped robot with point foot
class WheelfootController : public ControllerBase {
  using tensor_element_t = float; // Type alias for tensor elements
  using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>; // Type alias for matrices

public:
  WheelfootController() = default; // Default constructor

  ~WheelfootController() override = default; // Destructor

  // Enumeration for controller modes
  enum class Mode : uint8_t {
    WHEEL_STAND, 
    WHEEL_WALK, 
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

  // Compute observations for the controller
  void computeObservation() override;

  // Compute encoder for the controller
  void computeEncoder() override;

  // Compute actions for the controller
  void computeActions() override;

  // Handle walk mode
  void handleWheelStandMode() override;
  void handleRLWheelMoveMode() override;

  // Callback function for command velocity
  void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg) override;

  // Get the robot configuration
  void EEPoseCmdRCCallback(const std_msgs::Float32MultiArrayConstPtr &msg);

  Mode mode_;

  Mode last_mode_;

  int work_mode_flag_{10};

private:
  struct RCEECmd{
    Eigen::Vector3d ee_position;
    Eigen::Vector3d ee_rpy;
    bool gripper_cmd{false};
    void zero() {
        ee_position << 0, 0, 0;
        ee_rpy << 0, 0, 0;
    }
  };

  struct WheelPDcontroller
  {
    double wheel_pos[2] = {0.0, 0.0};
    double wheel_cnt[2] = {0.0, 0.0};

    double wheel_pos_offset[2] = {0.0, 0.0};

    double wheel_pos_last[2] = {0.0, 0.0};
    
    double wheel_pos_total[2] = {0.0, 0.0};

    double wheel_move_x = 0.0;
    double wheel_move_yaw = 0.0;

    double wheel_move_x_target = 0.0;
    double wheel_move_yaw_target = 0.0;

    double wheel_move_x_last = 0.0;
    double wheel_move_yaw_last = 0.0;

    double x_kp = 0.05;
    double x_kd = 0.00;
    double yaw_kp = -0.2;
    double yaw_kd = 0.00;

    double x_pid_out = 0.0;
    double yaw_pid_out = 0.0;
  };

  WheelPDcontroller wheel_pd_;

  vector3_t ee_init_pos_;
  vector3_t ee_init_rpy_;
  Eigen::Matrix3d ee_init_ori_;

  ros::Publisher gripper_cmd_pub_;
  std_msgs::Bool gripper_cmd_msg_;

  ros::Publisher ee_pos_cmd_debug_pub_;
  geometry_msgs::Pose ee_pos_cmd_debug_msg_;

  ros::Subscriber ee_pos_cmd_rc_delta_;
  std_msgs::Float32MultiArray ee_pos_cmd_rc_delta_msg_;
  RCEECmd rc_ee_cmd;

  // onnx policy model
  std::string policyFilePath_;
  std::shared_ptr<Ort::Env> onnxEnvPtr_;

  // ONNX session pointers
  std::unique_ptr<Ort::Session> encoderSessionPtr_;
  std::vector<const char *> encoderInputNames_;
  std::vector<const char *> encoderOutputNames_;
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

  bool isfirstRecObs_{true};
  int encoderInputSize_, encoderOutputSize_;

  vector3_t baseLinVel_;
  vector3_t basePosition_;
  vector_t lastActions_;
  vector_t wheelActions_;

  int actionsSize_;
  int commandsSize_;
  int observationSize_;
  int obsHistoryLength_;
  int gaitGeneratorOutputSize_;
  float imu_orientation_offset[3];
  std::vector<tensor_element_t> actions_;
  std::vector<tensor_element_t> observations_;
  std::vector<tensor_element_t> encoderOut_;
  std::vector<tensor_element_t> gaitGeneratorOut_;

  double gait_index_{0.0};

  std::string policy_path_{"policy"};

  Eigen::Matrix<double, 14, 1> wheel_stand_pos_;
  bool policy_move_damping_mode_{false};
};

} // namespace robot_controller
#endif //_LIMX_WHEELFOOT_CONTROLLER_H_
