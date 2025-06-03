#ifndef COMMAND_TYPES_H
#define COMMAND_TYPES_H

#include <array>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <string>
#include <string_view>
#include <vector>
using std::array;
using std::vector;

#include "airbot/libraries/atomic_queue.h"
#include "airbot/modules/boards/board_driver.hpp"

enum MotorControlState { INIT = 0, JOINT_POS = 1, JOINT_VEL = 2, JOINT_MIT = 3 };
constexpr std::string_view MotorControlStateStr[] = {"INIT", "JOINT_POS", "JOINT_VEL", "JOINT_MIT"};

/**
 * @brief The operation mode of the robot
 * ONLINE: online mode, robot is controlled by external commands
 * DEMONSTRATE: demonstrate mode, robot is controlled by human dragging
 * RECORDING: recording mode, robot is recording the trajectory while dragging
 * OFFLINE: offline mode, robot is controlled by offline commands
 * REPLAY_REACHING: robot is reaching the start point of the recorded trajectory
 * REPLAY_WAITING: robot is waiting for the start signal of the recorded trajectory
 * REPLAYING: robot is replaying the recorded trajectory
 * ERROR: robot is in error state
 */
enum ArmMode {
  ONLINE = 0,
  DEMONSTRATE = 1,
  RECORDING = 2,
  OFFLINE = 3,
  REPLAY_REACHING = 4,
  REPLAY_WAITING = 5,
  REPLAYING = 6,
  ERROR = 7
};
constexpr std::string_view ArmModeStr[] = {"ONLINE",          "DEMONSTRATE",    "RECORDING", "OFFLINE",
                                           "REPLAY_REACHING", "REPLAY_WAITING", "REPLAYING", "ERROR"};

constexpr std::string_view SnapModeStr[] = {"SNAP_RELEASE", "SNAP_SHORT_PRESS", "SNAP_LONG_PRESS", "SNAP_DOUBLE_PRESS"};

enum EndEffectorType { NONE = 0, DEMONSTRATOR = 1, GRIPPER = 2, YINSHI = 3 };
constexpr std::string_view EndEffectorTypeStr[] = {"teacher", "gripper", "yinshi"};

inline EndEffectorType str2endtype(const std::string& str) {
  if (str == "teacher") return EndEffectorType::DEMONSTRATOR;
  if (str == "gripper") return EndEffectorType::GRIPPER;
  if (str == "yinshi") return EndEffectorType::YINSHI;
  return EndEffectorType::NONE;
}

using ErrorCode = uint16_t;
enum Error {
  NO_ERROR = 0x00,
  DISCONNECT = 0x01,
  FIRMWARE_VER_TOO_LOW = 0x02,
  FIRMWARE_VER_TOO_HIGH = 0x03,
  SN_CODE_MISSING = 0x04,
  SN_CODE_MISMATCH = 0x05,
  MOTOR_OVER_TEMP = 0x06,
  MOTOR_OVER_CURRENT = 0x07,
  MOTOR_OTHER_ERR = 0x08,
};
constexpr std::string_view ErrorDesc[] = {"No error",
                                          "Connection lost",
                                          "Firmware version too low",
                                          "Firmware version too high",
                                          "SN code missing",
                                          "SN code mismatch with arm SN",
                                          "Motor temperature too high",
                                          "Motor current too high"};

using ArmModeTransition = uint8_t;
using ArmModeSnapAction = uint8_t;

template <typename T>
inline constexpr ArmModeSnapAction operator>>(ArmMode from, T to) {
  static_assert(std::is_enum<T>::value, "T must be an enum type");
  return static_cast<uint8_t>(from) << 4 | static_cast<uint8_t>(to);
}

template <typename T>
inline constexpr ArmModeSnapAction operator|(ArmMode from, T action) {
  static_assert(std::is_enum<T>::value, "T must be an enum type");
  return static_cast<uint8_t>(from) << 4 | static_cast<uint8_t>(action);
}

/**
 * @brief The data structure for cartesian translation
 */
using Translation = array<double, 3>;

/**
 * @brief The data structure for cartesian rotation
 */
using Rotation = array<double, 4>;

/**
 * @brief The data structure for cartesian frame (translation + rotation)
 */
using Frame = std::pair<Translation, Rotation>;

/**
 * @brief The data structure for wrench (force + torque)
 */
using Wrench = std::pair<array<double, 3>, array<double, 3>>;

/**
 * @brief The data structure for twist (linear velocity + angular velocity)
 */
using Twist = std::pair<array<double, 3>, array<double, 3>>;

/**
 * @brief The data structure for joint infomation
 * @tparam DOF The degree of freedom of the robot
 */
template <std::size_t DOF>
using Joints = array<double, DOF>;

/**
 * @brief The data structure for downward robot command
 * @tparam DOF The degree of freedom of the robot
 */
template <std::size_t DOF>
struct RobotCmdData {
  MotorControlState cmd_state;
  double target_end;                // Target end position
  Joints<DOF> target_joint_q;       // Target joint position
  Joints<DOF> plan_target_joint_q;  // Target joint torque
  Joints<DOF> target_joint_v;       // Target joint velocity
  Joints<DOF> target_joint_t;       // Target joint torque
  Joints<DOF> target_joint_kp;      // Target joint kp. Only functional in MIT mode
  Joints<DOF> target_joint_kd;      // Target joint kd. Only functional in MIT mode
  bool replay_request;              // flag to start replay
  bool recover_request;             // flag to recover from error

  void init(size_t dof) {
    cmd_state = MotorControlState::INIT;
    target_end = 0.;
    target_joint_q.fill(0.);
    plan_target_joint_q.fill(0.);
    target_joint_v.fill(0.);
    target_joint_t.fill(0.);
    target_joint_kp.fill(0.);
    target_joint_kd.fill(0.);
    replay_request = false;
    recover_request = false;
  }
};

/**
 * @brief The data structure for upward robot feedback
 * @tparam DOF The degree of freedom of the robot
 */
template <std::size_t DOF>
struct RobotFeedbackData {
  uint64_t time_stamp;
  MotorControlState current_state;
  Joints<DOF> current_joint_q;                 // Current joint position
  Joints<DOF> current_joint_v;                 // Current joint velocity
  Joints<DOF> current_joint_t;                 // Current joint torque
  Joints<DOF> current_joint_temp;              // Current joint temperature
  std::array<uint8_t, DOF> current_joint_err;  // Current joint error_id
  std::array<int, DOF> response_cnt;           // Response count for each joint
  double current_end;                          // Current end position

  Frame current_pose;              // Current pose in Cartesian space
  bool is_robot_stopped;           // flag to indicate robot is stopped
  Joints<DOF> last_valid_joint_q;  // Last valid joint positions

  void init(size_t dof) {
    time_stamp = 0;
    // current_pose is initialized to zero by KDL
    current_end = 0.;
    current_state = MotorControlState::INIT;
    current_joint_q.fill(0);
    current_joint_v.fill(0);
    current_joint_t.fill(0);
    current_joint_temp.fill(0);
    current_joint_err.fill(0);
    last_valid_joint_q.fill(0);
    response_cnt.fill(0);
    is_robot_stopped = true;
  }
};
template <std::size_t DOF>
struct LoggingData {
  std::string sn_code;
  RobotCmdData<DOF> cmd_data;
  RobotFeedbackData<DOF> fb_data;
};

/**
 * @brief The data structure for recorded robot trajectory
 * @tparam DOF The degree of freedom of the robot
 */
template <std::size_t DOF>
using Queue = atomic_queue::AtomicQueueB2<LoggingData<DOF>, std::allocator<LoggingData<DOF>>>;
template <std::size_t DOF>
struct RobotRecordData {
  std::vector<uint64_t> time_records;
  std::vector<Joints<DOF>> q_records, v_records, t_records;
  std::vector<Frame> wp_records;
  std::vector<double> end_records;

  void init() {
    q_records.clear();
    q_records.reserve(128000);
    v_records.clear();
    v_records.reserve(128000);
    t_records.clear();
    t_records.reserve(128000);
    end_records.clear();
    end_records.reserve(128000);
    wp_records.clear();
    wp_records.reserve(128000);
    time_records.clear();
    time_records.reserve(128000);
  }

  bool empty() const {
    return q_records.empty() && v_records.empty() && t_records.empty() && end_records.empty() && wp_records.empty() &&
           time_records.empty();
  }
};

/**
 * @brief The data structure for robot planning
 * @tparam DOF The degree of freedom of the robot
 */
template <std::size_t DOF>
struct RobotPlanData {
  uint64_t plan_start_timestamp;
  uint64_t plan_execute_time;
  array<array<double, 4>, DOF> plan_params;
};

inline Frame frame2array(const KDL::Frame& frame) {
  double x, y, z, w;
  frame.M.GetQuaternion(x, y, z, w);
  return std::make_pair(Translation{frame.p.x(), frame.p.y(), frame.p.z()}, Rotation{x, y, z, w});
}

inline KDL::Frame array2frame(const Frame& frame) {
  KDL::Frame f;
  f.p = KDL::Vector(frame.first[0], frame.first[1], frame.first[2]);
  f.M = KDL::Rotation::Quaternion(frame.second[0], frame.second[1], frame.second[2], frame.second[3]);
  return f;
}

template <std::size_t DOF>
inline KDL::JntArray joints2jntarray(const Joints<DOF>& joints) {
  KDL::JntArray j(DOF);
  for (int i = 0; i < DOF; i++) j(i) = joints[i];
  return j;
}

template <std::size_t DOF>
inline Joints<DOF> jntarray2joints(const KDL::JntArray& joints) {
  Joints<DOF> j;
  for (int i = 0; i < DOF; i++) j[i] = joints(i);
  return j;
}

#endif
