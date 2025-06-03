// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_POINTFOOT_HW_H_
#define _LIMX_POINTFOOT_HW_H_

#include <robot_hw/RobotHW.h>
#include <robot_hw/RobotData.h>
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"
#include "realtime_tools/realtime_buffer.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "limxsdk/pointfoot.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
// diagnostic_msgs: int level, string name, string message, string hardward_id, vector<KeyValue> values
// mros::DiagnosticValue: header, int level, string name, string code, string message 
// Diagnostics::publish(name, code, level, message); // level 0->OK, 2->ERROR

#include <airbot/airbot.hpp>
#include <airbot_msgs/SetInt32.h>
#include <airbot_msgs/ArmCmd.h>
#include <airbot_msgs/ArmState.h>

namespace hw {

const std::vector<std::string> CONTACT_SENSOR_NAMES = {"L_FOOT", "R_FOOT"};

class PointfootHW : public hw::RobotHW {
public:
  PointfootHW() = default;

  /**
   * \brief Get necessary parameters from the parameter server and initialize hardware_interface.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when initialization is successful, False when failed.
   */
  bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;

  /**
   * \brief Communicate with hardware to get data and status of the robot.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time &time, const ros::Duration &period) override;

  /**
   * \brief Communicate with hardware to publish commands to the robot.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time &time, const ros::Duration &period) override;

  bool loadUrdf(ros::NodeHandle &nh) override;

  bool startBipedController();

  bool stopBipedController();

private:
  void grasp(bool to_grasp);

  bool setupJoints();

  bool setupImu();

  bool setupContactSensor(ros::NodeHandle &nh);

  void cmdVelCallbackFromSDK(const geometry_msgs::TwistConstPtr &msg);

  void gripperCmdCallback(const std_msgs::BoolConstPtr& msg);

  void armStateCallback(const airbot_msgs::ArmStateConstPtr& msg);

  std::vector<MotorData> jointData_; // Vector to store motor data for each joint.
  ImuData imuData_{}, imuData_last_{};

  std::shared_ptr< arm::Robot<6> > airbot_arm_hw_;
  RobotCmdData<6> airbot_arm_cmd_;
  RobotFeedbackData<6> airbot_arm_feedback_;

  bool arm_init_flag_; //arm init flag , = true when start rl controller(standing)
  bool arm_activate_flag_;

  realtime_tools::RealtimeBuffer<limxsdk::RobotState> robotstate_buffer_;
  realtime_tools::RealtimeBuffer<airbot_msgs::ArmState> armstate_buffer_;
  realtime_tools::RealtimeBuffer<limxsdk::ImuData> imudata_buffer_;
  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> sdk_twist_buffer_;
  limxsdk::RobotCmd robotCmd_;
  airbot_msgs::ArmCmd armCmd_;

  bool contactState_[2]{}; // NOLINT(modernize-avoid-c-arrays)
  int contactThreshold_{40};

  int calibration_state_{-1};

  ros::Subscriber armstate_sub_;

  ros::Subscriber gripper_cmd_sub_;
  bool gripper_cmd_{false};

  ros::Publisher armcmd_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher ee_rc_cmd_delta_pub_;

  std::map<std::string, int> joystick_btn_map_;
  std::map<std::string, int> joystick_axes_map_;
  ros::ServiceClient switch_controllers_client_; // named switch_client_ in yg
  ros::ServiceClient list_controllers_client_;
  
  limxsdk::SensorJoy usercmd_last_;

  double imu_offset_roll_{0.0}, imu_offset_pitch_{0.0};
  double imu_offset_roll_move_{0.0}, imu_offset_pitch_move_{0.0}, imu_offset_roll_stairs_{0.0}, imu_offset_pitch_stairs_{0.0};
  int mode_flag_{10};
  int seq_num_{10};
  int walking_state_{-1}, stairs_state_{-1};
  bool imu_offset_changed_{false};
  int encoder_type_{1};
  int imu_exception_count_{0};
  bool is_stance_end_{false}, is_rlStand_end_{false}, is_rlWheelStand_end_{false}, is_rlSoleStand_end_{false};
  std::string controller_type_; // 0:pointFootController; 1:wheelFootController; 2:bipedController
  int joint_num_{6};
  int robot_subtype{0}; // 0:pointFoot; 1:pointfoot_tron; 2:wheelFoot_tron; 3:biped_tron;
  bool is_crit_low_bat_{false},select_btn_changed_{false}, is_recover_done_{false}, is_contact_ground_{true};
  int last_select_value_{0};
  bool stair_command_{false};
  std::mutex rccmd_mutex_;
  // ros::Subscriber cmdVelSubFromSDK_;
  ros::Time cmdVelFromSDKLast_;

  limxsdk::PointFoot *robot_;

  std::string controller_name_;

  std::string robot_type_;      // Type of the robot (e.g., point foot, wheel foot, sole foot)
  bool is_point_foot_{false};   // Indicates if the robot has a point foot configuration
  bool is_wheel_foot_{false};   // Indicates if the robot has a wheel foot configuration
  bool is_sole_foot_{false};    // Indicates if the robot has a sole foot configuration

  bool r2_btn_changed_{false};
  int last_r2_value_{0};
  
  ros::Time btn_trigger_time_{0.0};
  ros::Time btn_continue_time_{0.0};

  ros::Time r2_btn_trigger_time_{0.0};
  ros::Time r2_btn_continue_time_{0.0};

  ros::Time low_bat_trigger_time_{0.0};
  ros::Time low_bat_continue_time_{0.0};
  int is_sim_{1};
};

} // namespace hw

#endif //_LIMX_POINTFOOT_HW_H_
