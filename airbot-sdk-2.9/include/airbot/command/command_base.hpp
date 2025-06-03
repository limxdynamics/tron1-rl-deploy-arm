#ifndef COMMAND_BASE_HPP
#define COMMAND_BASE_HPP

#include <Eigen/Dense>
#include <atomic>
#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <shared_mutex>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

#include "httplib.h"
#undef _res

#include "airbot/command/command_types.hpp"
#include "airbot/modules/boards/interface_board_base.hpp"
#include "airbot/modules/boards/interface_board_end.hpp"
#include "airbot/modules/controller/fk.hpp"
#include "airbot/modules/controller/fk_analytic.hpp"
#include "airbot/modules/controller/id.hpp"
#include "airbot/modules/controller/id_chain_rne.hpp"
#include "airbot/modules/controller/ik.hpp"
#include "airbot/modules/controller/ik_analytic.hpp"
#include "airbot/modules/controller/ikv_chain.hpp"
#include "airbot/modules/motors/motor_driver.hpp"
#include "airbot/utils.hpp"

#define MAGIC_DELAY 200  // This MAGIC_DELAY should be removed in the future

using arm::MotorDriver;
using std::vector;
using SnapMode = arm::BoardDriver::snap_mode_e;
namespace fs = std::filesystem;
constexpr const double DEFAULT_PLAN_VEL = 0.5;
constexpr const double LOOK_AHEAD = 0;
constexpr const double QUEUE_SIZE = 10000;
constexpr const uint32_t BLOCK_SPIN_TIME = 10;  // 10 milliseconds
constexpr const double BLOCK_THRESHOLD = 0.005;
constexpr const uint32_t BLOCK_TIMEOUT = 30000;  // 30 second
constexpr const double DEFAULT_JNT_VEL = M_PI;
constexpr const double MAX_JNT_VEL = 4 * M_PI;
constexpr const double RECOVERY_SPD_RATIO = 0.25;
constexpr const double ENCODER_ERROR = 20;

constexpr const double REF_POS[7] = {MotorDriver::joint_lower_bounder_[0] * 180 / M_PI,
                                     MotorDriver::joint_upper_bounder_[1] * 180 / M_PI,
                                     MotorDriver::joint_lower_bounder_[2] * 180 / M_PI,
                                     0,
                                     0,
                                     0,
                                     0};

constexpr const double INIT_POS[7] = {
    0, MotorDriver::joint_upper_bounder_[1] * 180 / M_PI, MotorDriver::joint_lower_bounder_[2] * 180 / M_PI, 0, 0, 0,
    0};

class KibanaLogger {
 private:
  bool inited_;
  std::string url_;
  std::string topic_name_;
  std::unique_ptr<httplib::Client> client_;

 public:
  KibanaLogger(std::string url = "http://192.168.112.156:9200", std::string topic_name = "debug")
      : url_(url), topic_name_(topic_name), inited_(false) {}

  template <std::size_t DOF>
  void push_remote_log_once(const LoggingData<DOF>& data) {
    if (!inited_) {
      client_ = std::make_unique<httplib::Client>(url_.c_str());
      nlohmann::json document;
      document["mappings"]["properties"]["timestamp"]["type"] = "date";
      document["mappings"]["properties"]["feedback"]["properties"]["timestamp"]["type"] = "date";
      client_->Put(std::string("/") + topic_name_, document.dump(), "application/json");
      inited_ = true;
    }
    nlohmann::json document;

    auto time_stamp = data.fb_data.time_stamp / 1000;
    auto current_state = data.fb_data.current_state;
    auto postion = data.fb_data.current_joint_q;
    auto velocity = data.fb_data.current_joint_v;
    auto torque = data.fb_data.current_joint_t;
    auto temperature = data.fb_data.current_joint_temp;
    auto err = data.fb_data.current_joint_err;
    auto resp_cnt = data.fb_data.response_cnt;
    auto end_pose = data.fb_data.current_pose;
    auto current_end = data.fb_data.current_end;

    auto target_state = data.cmd_data.cmd_state;
    auto target_position = data.cmd_data.target_joint_q;
    auto target_plan_position = data.cmd_data.plan_target_joint_q;
    auto target_velocity = data.cmd_data.target_joint_v;
    auto target_torque = data.cmd_data.target_joint_t;
    auto target_kp = data.cmd_data.target_joint_kp;
    auto target_kd = data.cmd_data.target_joint_kd;
    auto target_replay = data.cmd_data.replay_request;
    auto target_recover = data.cmd_data.recover_request;
    auto target_end = data.cmd_data.target_end;

    document["feedback"]["timestamp"] = time_stamp;

    for (int i = 0; i < data.cmd_data.target_joint_q.size(); i++) {
      document["feedback"][std::to_string(i)]["position"] = postion[i];
      document["feedback"][std::to_string(i)]["velocity"] = velocity[i];
      document["feedback"][std::to_string(i)]["torque"] = torque[i];
      document["feedback"][std::to_string(i)]["temperature"] = temperature[i];
      document["feedback"][std::to_string(i)]["error_id"] = err[i];
      document["feedback"][std::to_string(i)]["response_cnt"] = resp_cnt[i];
    }
    document["feedback"]["end_pose"]["x"] = end_pose.first[0];
    document["feedback"]["end_pose"]["y"] = end_pose.first[1];
    document["feedback"]["end_pose"]["z"] = end_pose.first[2];
    document["feedback"]["end_pose"]["qx"] = end_pose.second[0];
    document["feedback"]["end_pose"]["qy"] = end_pose.second[1];
    document["feedback"]["end_pose"]["qz"] = end_pose.second[2];
    document["feedback"]["end_pose"]["qw"] = end_pose.second[3];

    document["feedback"]["current_end"] = current_end;
    document["feedback"]["current_state"] = current_state;

    for (int i = 0; i < data.cmd_data.target_joint_q.size(); i++) {
      document["command"][std::to_string(i)]["position"] = target_position[i];
      document["command"][std::to_string(i)]["plan_position"] = target_plan_position[i];
      document["command"][std::to_string(i)]["velocity"] = target_velocity[i];
      document["command"][std::to_string(i)]["torque"] = target_torque[i];
      document["command"][std::to_string(i)]["kp"] = target_kp[i];
      document["command"][std::to_string(i)]["kd"] = target_kd[i];
    }
    document["command"]["target_state"] = target_state;
    document["command"]["replay_request"] = int(target_replay);
    document["command"]["recover_request"] = int(target_recover);
    document["command"]["target_end"] = target_end;
    // document["timestamp"] =
    //     std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
    //         .count();
    document["timestamp"] = time_stamp;
    document["type"] = "debug";
    document["sn"] = data.sn_code;

    client_->Post(std::string("/") + topic_name_ + "/_doc", document.dump(), "application/json");
  }
};

namespace arm {
inline array<double, 4> calc(double s_q, double s_v, double t_q, double t_v) {
  return {s_q, s_v, 3 * (t_q - s_q) - 2 * s_v - 1 * t_v, -2 * (t_q - s_q) + s_v + t_v};
}

template <std::size_t DOF>
inline array<array<double, 4>, DOF> calc_plan(const Joints<DOF>& current_q, const Joints<DOF>& current_v,
                                              const Joints<DOF>& target_q, const Joints<DOF>& target_v) {
  array<array<double, 4>, DOF> plan_params;
  for (int i = 0; i < plan_params.size(); i++)
    plan_params[i] = calc(current_q[i], current_v[i] * 1, target_q[i], target_v[i]);
  return plan_params;
}

template <std::size_t DOF>
inline Joints<DOF> plan_infer(const array<array<double, 4>, DOF>& plan_params, double t) {
  auto ret = Joints<DOF>();
  for (int i = 0; i < plan_params.size(); i++)
    ret[i] = plan_params[i][0] + plan_params[i][1] * t + plan_params[i][2] * t * t + plan_params[i][3] * t * t * t;
  return ret;
}

/**
 * @brief The Robot class for controlling the robot arm
 * @tparam DOF the number of degrees of freedom of the robot arm
 *
 * 1. plug-to-play
 * 2. control mode and light effect
 * 3. Demonstrate Mode / Replay Mode / Online Mode
 *
 */
template <std::size_t DOF>
class Robot {
  static std::unordered_map<std::string, std::pair<double, double>> end_limits;
  inline static double e2i(const double& end, const std::string& mode) {
    auto limits = end_limits[mode];
    return (end - limits.first) / (limits.second - limits.first);
  }

  inline static double i2e(const double& i, const std::string& mode) {
    auto limits = end_limits[mode];
    return i * (limits.second - limits.first) + limits.first;
  }

 private:
  /**
   * Robot status
   */
  std::atomic<ArmMode> arm_mode_;
  std::atomic<bool> is_running_, use_planning_, joint_safe_detect_, is_init_, stop_update_;
  std::atomic<uint32_t> logging_freq_;
  RobotCmdData<DOF> robot_cmd_data_;
  RobotFeedbackData<DOF> robot_fb_data_;
  ErrorCode error_code_;

  std::atomic<uint32_t> counter_;
  std::atomic<time_t> reported_time_;
  std::atomic<uint64_t> replay_index_;
  RobotPlanData<DOF> robot_plan_data_;
  RobotRecordData<DOF> recorded_data_, replay_data_;
  time_t last_update_time_;
  time_t record_start_time_;
  time_t replay_inplace_time_, replay_start_time_;

  /**
   * Robot parameters
   */
  std::string end_effector_type_;
  Joints<DOF> joint_vel_limit_;

  /**
   * Logging
   */
  std::shared_ptr<spdlog::logger> logger_;
  Queue<DOF> logging_queue_;
  KibanaLogger kibana_logger_;
  std::atomic<uint64_t> last_logging_time_;

  /**
   * Robot modules
   */
  std::shared_ptr<FKSolver<DOF>> fk_solver_;
  std::shared_ptr<IKSolver<DOF>> ik_solver_;
  std::shared_ptr<IKVSolver<DOF>> ikv_solver_;
  std::shared_ptr<IDSolver<DOF>> id_solver_;
  array<std::shared_ptr<MotorDriver>, DOF> motor_driver_;
  std::shared_ptr<MotorDriver> end_motor_driver_;
  std::shared_ptr<InterfaceBoardBase> interface_board_base_;
  std::shared_ptr<InterfaceBoardEnd> interface_board_end_;

  /**
   * Thread handles
   */
  std::thread main_update_thread_;
  std::thread logging_thread_;

  /**
   * Mutexes
   */
  mutable std::mutex cmd_mutex_;
  mutable std::shared_mutex fb_mutex_;
  mutable std::mutex record_mutex_;

 protected:
  bool _plan_target_joint_q(const Joints<DOF>& target_joint_q, bool use_planning = true,
                            double plan_vel = DEFAULT_PLAN_VEL, bool commit = true);

  // update whole data once
  void _update_once();

  inline void _set_state(MotorControlState target_state) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    robot_cmd_data_.cmd_state = target_state;
  }

  inline void _write_fb_data(const RobotFeedbackData<DOF>& fb_tmp_data) {
    std::unique_lock<std::shared_mutex> fb_lock(fb_mutex_);
    robot_fb_data_ = fb_tmp_data;
  }

  inline void _write_cmd_data(const RobotCmdData<DOF>& cmd_tmp_data) {
    std::unique_lock<std::mutex> cmd_lock(cmd_mutex_);
    robot_cmd_data_ = cmd_tmp_data;
  }

  // External trigger to change arm mode
  inline bool change_mode(const ArmMode& cmd_mode) {
    RobotCmdData<DOF> robot_cmd_data;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      robot_cmd_data = robot_cmd_data_;
    }
    auto arm_mode = arm_mode_.load(std::memory_order_relaxed);
    return _check_mode_change(arm_mode, cmd_mode) && _on_mode_change(arm_mode, cmd_mode, robot_cmd_data);
  }

  bool _check_mode_change(const ArmMode& current_mode, const ArmMode& cmd_mode);

  // Perform mode change. Return true if mode is changed
  // By design if mode change happens actions will be performed in the next round
  // No check is performed to see if the mode should be changed
  bool _on_mode_change(const ArmMode& current_mode, const ArmMode& cmd_mode, RobotCmdData<DOF>& robot_cmd_data);

 public:
  /**
   * @brief Construct a new Robot object
   *
   * Once created, the instance will perform the following actions:
   *
   * 1. initialize forward and inverse kinematics solvers
   * 2. initialize text-based loggers; logs will be saved in the `logs` directory
   * 3. initialize and check the status of the base interface board
   * 4. initialize and check the status of the motors, from the base to the end
   * 5. initialize and check the status of the end interface board
   * 6. update motor status
   * 7. start the following threads:
   *    * thread_sync_pose_: map joints to end pose
   *    * thread_update_motor_: send CAN messages to motors
   *    * thread_plan_: perform planning
   *    * thread_log_: start pushing robot status to Kibana
   *    * thread_snap_: start listening to the snap signals. will be removed in the future
   * 8. unlock the motors
   *
   * Typically, the instance should be created once and used throughout the program. Also, it is the user's
   * responsibility to ensure that **ONLY ONE** instance is controlling the robot at a time.
   *
   * @param urdf_path the path to valid AIRBOT Play URDF File. By default, when installed via apt package, two valid
   * urdf files are installed, by default when empty, the default urdf file will be chosen according to `end_mode`.
   * Available options:
   * - AIRBOT Play with no end effector: `/usr/share/airbot_play/models/airbot_play.urdf`
   * - AIRBOT Play with AIRBOT Gripper 2: `/usr/share/airbot_play/models/airbot_play_gripper.urdf`
   * @param can_interface the interface recognized by the system. Usually in the form like `can0`, `can1`, etc. The
   * currently available interfaces can be found by `ip link` command, given that `iproute2` package is
   * installed on Debian-based systems
   * @param direction the direction of the gravity. If AIRBOT Play is installed on a vertical surface, this option
   * should be altered. Available options: "down", "left", "right"
   * @param joint_vel_limit the maximum velocity of joints in Online / Replay mode.
   * @param end_mode the end effector installed at the end. Available options:
   * - `none`: no end effector is installed
   * - `gripper`: AIRBOT Gripper is installed
   * - `newteacher`: AIRBOT Demonstrator is installed
   * @param forearm_tpe the type of the forearm. Available options:
   * - `DM`: Damiao motors are installed
   * - `OD`: Self-designed motors are installed
   */
  Robot(std::string urdf_path = "", std::string can_interface = "can0", std::string direction = "down",
        double joint_vel_limit = DEFAULT_JNT_VEL, std::string end_mode = "teacher", std::string bigarm_type = "OD",
        std::string forearm_type = "DM", bool factory = false);
  ~Robot();

  inline void logging(uint32_t logging_freq) {
    if (logging_freq > 100) {
      logger_->warn("Logging frequency is too high, set to 100Hz");
      logging_freq = 100;
    }
    if (logging_freq == 0) {
      logger_->info("Turning off kibana logging");
      logging_queue_ = Queue<DOF>(QUEUE_SIZE);
    } else
      logger_->info("Setting kibana logging to {}Hz", logging_freq);

    logging_freq_.store(logging_freq, std::memory_order_relaxed);
  }

  /**
   * @brief Get the current end pose in Cartesian space.
   *
   * The reference frame is the base frame, where the origin is located at the base, x
   * axis is pointing to the front, y axis is pointing to the left, and z axis is pointing up. The rotation is
   * represented by a quaternion relative to the `(1, 0, 0)` unit vector.
   *
   * @return Frame: current end pose consisting of translation and quaternion rotation in `(( x, y, z
   * ), ( rx, ry, rz, rw ))` format.
   */
  Frame get_current_pose() const {
    std::shared_lock<std::shared_mutex> lock(fb_mutex_);
    return robot_fb_data_.current_pose;
  };
  /**
   * @brief Get the current joint positions in joint space.
   * @return Joints<DOF>: current joint positions in radians.
   */
  Joints<DOF> get_current_joint_q() const {
    std::shared_lock<std::shared_mutex> lock(fb_mutex_);
    return robot_fb_data_.current_joint_q;
  }
  /**
   * @brief Get the current joint velocities in joint space.
   * @return Joints<DOF>: current joint velocities in radians per second.
   */
  Joints<DOF> get_current_joint_v() const {
    std::shared_lock<std::shared_mutex> lock(fb_mutex_);
    return robot_fb_data_.current_joint_v;
  };
  /**
   * @brief Get the current joint torques in joint space.
   * @return Joints<DOF>: current joint torques in Newton meters.
   */
  Joints<DOF> get_current_joint_t() const {
    std::shared_lock<std::shared_mutex> lock(fb_mutex_);
    return robot_fb_data_.current_joint_t;
  };
  /**
   * @brief Get the current rotation part of the end pose.
   *
   * The rotation is represented by a quaternion relative to the `(1, 0, 0)` unit vector.
   * The reference frame is the base frame, where the origin is located at the base, x
   * axis is pointing to the front, y axis is pointing to the left, and z axis is pointing up.
   *
   * @return Translation: current rotation in quaternion format `( rx, ry, rz, rw )`.
   */
  Translation get_current_translation() const { return get_current_pose().first; };
  /**
   * @brief Get the current end effector position.
   *
   * The end effector position is a normalized value between 0 and 1, where 0 denotes that the end effector is closed
   * and 1 denotes that the end effector is open.
   *
   * @return Rotation: current end position in meters.
   */
  Rotation get_current_rotation() const { return get_current_pose().second; };
  /**
   * @brief Get the current end effector position.
   *
   * The end effector position is a normalized value between 0 and 1, where 0 denotes that the end effector is closed
   * and 1 denotes that the end effector is open.
   *
   * @return double: current end position in meters.
   */
  double get_current_end() const {
    std::shared_lock<std::shared_mutex> lock(fb_mutex_);
    return robot_fb_data_.current_end;
  };

  array<uint8_t, DOF> get_current_joint_error_code() const {
    std::shared_lock<std::shared_mutex> lock(fb_mutex_);
    return robot_fb_data_.current_joint_err;
  };
  Joints<DOF> get_current_joint_temperature() const {
    std::shared_lock<std::shared_mutex> lock(fb_mutex_);
    return robot_fb_data_.current_joint_temp;
  };

  /**
   * @brief Get the SN code of the robot arm.
   * @return std::string: the serial number of the robot arm.
   */
  inline std::string get_sn() const { return interface_board_base_->get_arm_sn_code(); }

  /**
   * @brief Position control method. Set the target end pose of the robot arm in Cartesian space.
   *
   * The reference frame is the base frame, where the origin is located at the base, x
   * axis is pointing to the front, y axis is pointing to the left, and z axis is pointing up. The rotation is
   * represented by a quaternion relative to the `(1, 0, 0)` unit vector.
   *
   * @param target_pose the target pose in Cartesian space.
   * @param use_planning whether or not to use planning. If `true`, the robot will plan the trajectory to the target
   * pose with cubic interpolation. If `false`, the robot will move directly to the target pose.
   * @param vel the velocity when reaching the target
   * @param blocking whether or not to block the current procedure call until the target is reached

   * @return if not blocking, return true if the target is set successfully; if blocking, return true if the target is
   reached successfully.
   */
  bool set_target_pose(const Frame& target_pose, bool use_planning = true, double vel = DEFAULT_PLAN_VEL,
                       bool blocking = false);
  /**
   * @brief Position control method. Set the target end pose of the robot arm in Cartesian space.
   *
   * The reference frame is the base frame, where the origin is located at the base, x
   * axis is pointing to the front, y axis is pointing to the left, and z axis is pointing up. The rotation is
   * represented by a quaternion relative to the `(1, 0, 0)` unit vector.
   *
   * @param target_translation the target translation in meters.
   * @param target_rotation the target rotation in quaternion format `( rx, ry, rz, rw )`.
   * @param use_planning whether or not to use planning. If `true`, the robot will plan the trajectory to the target
   * pose with cubic interpolation. If `false`, the robot will move directly to the target pose.
   * @param vel the velocity when reaching the target
   * @param blocking whether or not to block the current procedure call until the target is reached

   * @return if not blocking, return true if the target is set successfully; if blocking, return true if the target is
   reached successfully.
   */
  bool set_target_pose(const Translation& target_translation, const Rotation& target_rotation, bool use_planning = true,
                       double vel = DEFAULT_PLAN_VEL, bool blocking = false);
  /**
   * @brief Position control method. Set the target translation of the robot arm in Cartesian space.
   *
   * The reference frame is the base frame, where the origin is located at the base, x
   * axis is pointing to the front, y axis is pointing to the left, and z axis is pointing up.
   *
   * @param target_translation the target translation in meters.
   * @param use_planning whether or not to use planning. If `true`, the robot will plan the trajectory to the target
   * pose with cubic interpolation. If `false`, the robot will move directly to the target pose.
   * @param vel the velocity when reaching the target
   * @param blocking whether or not to block the current procedure call until the target is reached

   * @return if not blocking, return true if the target is set successfully; if blocking, return true if the target is
   reached successfully.
   */
  bool set_target_translation(const Translation& target_translation, bool use_planning = true,
                              double vel = DEFAULT_PLAN_VEL, bool blocking = false);
  /**
   * @brief Position control method. Set the target relative translation of the robot arm in Cartesian space.
   * The robot will move to the target pose by adding the relative translation to the current planning target.
   *
   * The reference frame is the base frame, where the origin is located at the base, x
   * axis is pointing to the front, y axis is pointing to the left, and z axis is pointing up.
   *
   * @param target_d_translation the target relative translation in meters.
   * @param use_planning whether or not to use planning. If `true`, the robot will plan the trajectory to the target
   * pose with cubic interpolation. If `false`, the robot will move directly to the target pose.
   * @param vel the velocity when reaching the target
   * @param blocking whether or not to block the current procedure call until the target is reached

   * @return if not blocking, return true if the target is set successfully; if blocking, return true if the target is
   reached successfully.
   */
  bool add_target_translation(const Translation& target_d_translation, bool use_planning = true,
                              double vel = DEFAULT_PLAN_VEL, bool blocking = false);
  /**
   * @brief Set the target relative translation of the robot arm in Cartesian space.
   * The robot will move to the target pose by adding the relative translation to the current planning target. The
   * addition is performed in **the base frame**.
   *
   * The reference frame is the base frame, where the origin is located at the base, x
   * axis is pointing to the front, y axis is pointing to the left, and z axis is pointing up.
   *
   * @param target_d_translation the target relative translation in quaternion format `( rx, ry, rz, rw )`.
   * @param use_planning whether or not to use planning. If `true`, the robot will plan the trajectory to the target
   * pose with cubic interpolation. If `false`, the robot will move directly to the target pose.
   * @param vel the velocity when reaching the target
   * @param blocking whether or not to block the current procedure call until the target is reached

   * @return if not blocking, return true if the target is set successfully; if blocking, return true if the target is
   reached successfully.
   */
  bool add_target_relative_translation(const Translation& target_d_translation, bool use_planning = true,
                                       double vel = DEFAULT_PLAN_VEL, bool blocking = false);

  /**
   * @brief Position control method. Set the target rotation of the robot arm in Cartesian space.
   * The robot will move to the target pose by adding the relative rotation to the current planning target. The
   * addition is performed in **the base frame**.
   *
   * The reference frame is the base frame, where the origin is located at the base, x
   * axis is pointing to the front, y axis is pointing to the left, and z axis is pointing up. The rotation is
   * represented by a quaternion relative to the `(1, 0, 0)` unit vector.
   *
   * @param target_d_rotation the target relative rotation in quaternion format `( rx, ry, rz, rw )`.
   * @param use_planning whether or not to use planning. If `true`, the robot will plan the trajectory to the target
   * pose with cubic interpolation. If `false`, the robot will move directly to the target pose.
   * @param vel the velocity when reaching the target
   * @param blocking whether or not to block the current procedure call until the target is reached

   * @return if not blocking, return true if the target is set successfully; if blocking, return true if the target is
   reached successfully.
   */
  bool add_target_relative_rotation(const Rotation& target_d_rotation, bool use_planning = true,
                                    double vel = DEFAULT_PLAN_VEL, bool blocking = false);

  /**
   * @brief Position control method. Set the target rotation of the robot arm in Cartesian space.
   *
   * The reference frame is the base frame, where the origin is located at the base, x
   * axis is pointing to the front, y axis is pointing to the left, and z axis is pointing up. The rotation is
   * represented by a quaternion relative to the `(1, 0, 0)` unit vector.
   *
   * @param target_rotation the target rotation in quaternion format `( rx, ry, rz, rw )`.
   * @param use_planning whether or not to use planning. If `true`, the robot will plan the trajectory to the target
   * pose with cubic interpolation. If `false`, the robot will move directly to the target pose.
   * @param vel the velocity when reaching the target
   * @param blocking whether or not to block the current procedure call until the target is reached

   * @return if not blocking, return true if the target is set successfully; if blocking, return true if the target is
   reached successfully.
   */
  bool set_target_rotation(const Rotation& target_rotation, bool use_planning = true, double vel = DEFAULT_PLAN_VEL,
                           bool blocking = false);
  /**
   * @brief Velocity control method. Set the target end velocities of the robot arm in Cartesian space.
   *
   * @bug This function is not implemented yet.
   *
   * @param target_vel the target end velocities in meters per second.
   */
  bool set_target_vel(const Twist& target_vel);

  /**
   * @brief Position control method. Set the target joint positions of the robot arm in joint space.
   * @param target_joint_q the target joint positions in radians.
   * @param use_planning whether or not to use planning. If `true`, the robot will plan the trajectory to the target
   * pose with cubic interpolation. If `false`, the robot will move directly to the target pose.
   * @param vel the velocity when reaching the target
   * @param blocking whether or not to block the current procedure call until the target is reached

   * @return if not blocking, return true if the target is set successfully; if blocking, return true if the target is
   reached successfully.
   */
  bool set_target_joint_q(const Joints<DOF>& target_joint_q, bool use_planning = true, double vel = DEFAULT_PLAN_VEL,
                          bool blocking = false);
  /**
   * @brief Position control method. Set the target joint positions of the robot arm in joint space.
   * The robot will move to the target pose by adding the relative joint positions to the current planning target joint
   * position targets.
   *
   * @param target_d_joint_q the target relative joint positions in radians.
   * @param use_planning whether or not to use planning. If `true`, the robot will plan the trajectory to the target
   * pose with cubic interpolation. If `false`, the robot will move directly to the target pose.
   * @param vel the velocity when reaching the target
   * @param blocking whether or not to block the current procedure call until the target is reached

   * @return if not blocking, return true if the target is set successfully; if blocking, return true if the target is
   reached successfully.
   */
  bool add_target_joint_q(const Joints<DOF>& target_d_joint_q, bool use_planning = true, double vel = DEFAULT_PLAN_VEL,
                          bool blocking = false);
  /**
   * @brief Velocity control method. Set the target joint velocities of the robot arm in joint space.
   * @param target_joint_v the target joint velocities in radians per second.
   */
  bool set_target_joint_v(const Joints<DOF>& target_joint_v);

  /**
   * @brief Velocity control method. Set the target joint velocities of the robot arm in joint space.
   * The robot will accelerate / decelerate to target joint velocities by adding the relative joint velocities to the
   * current joint velocity targets.
   *
   * @param target_joint_v the target joint velocities in radians per second.
   * @param blocking whether or not to block the current procedure call until the target is reached

   * @return if not blocking, return true if the target is set successfully; if blocking, return true if the target is
   reached successfully.
   */
  bool add_target_joint_v(const Joints<DOF>& target_d_joint_v);

  /**
   * @brief MIT control method. Set the target joint positions, velocities, kp, kd and feed-forward torques of the
   motors of the arm. This mode enables force control of the robot arm.
   * @note The gravity compensation is not included in the feed-forward torques.
   *
   * @param target_joint_q the target joint positions in radians.
   * @param target_joint_v the target joint velocities in radians per second.
   * @param target_joint_kp the target joint proportional gains.
   * @param target_joint_kd the target joint derivative gains.
   * @param target_joint_t the target joint feed-forward torques.
   *
   * @return return true if the target is set successfully
   */
  bool set_target_joint_mit(const Joints<DOF>& target_joint_q, const Joints<DOF>& target_joint_v,
                            const Joints<DOF>& target_joint_kp, const Joints<DOF>& target_joint_kd,
                            const Joints<DOF>& target_joint_t);
  /**
   * @brief MIT control method. Set the target joint positions, velocities, kp, kd and feed-forward torques of the
   motors of the arm. This mode enables force control of the robot arm.
   * @note The gravity compensation is included.
   *
   * @param target_joint_q the target joint positions in radians.
   * @param target_joint_v the target joint velocities in radians per second.
   * @param target_joint_kp the target joint proportional gains.
   * @param target_joint_kd the target joint derivative gains.
   *
   * @return return true if the target is set successfully
   */
  bool set_target_joint_mit(const Joints<DOF>& target_joint_q, const Joints<DOF>& target_joint_v,
                            const Joints<DOF>& target_joint_kp, const Joints<DOF>& target_joint_kd);
  /**
   * @brief Set the target end position of the robot arm.
   * The end position is a normalized value between 0 and 1, where 0 denotes that the end effector is closed and 1
   * denotes that the end effector is open.
   *
   * @param end the normalized target end position.
   * @param blocking whether or not to block the current procedure call until the target is reached

   * @return if not blocking, return true if the target is set successfully; if blocking, return true if the target is
   reached successfully.
   */
  bool set_target_end(const double& end_pose, bool blocking = false);
  void set_frame(const Joints<DOF>& read_meters);
  void set_initial_encoder();
  void calibration(bool is_set_zero = false);

  /**
   * @brief Check if the given end pose is reachable.
   *
   * The end pose is reachable if the end position is within the limits of the end effector.
   * The result may vary depending on the end effector installed at the end.
   *
   * @param target_pose the target pose in Cartesian space.
   * @return true if the end pose is reachable.
   */
  bool valid_target_pose(const Frame& target_pose) const;

  /**
   * @brief Check if the given joint position is reachable.
   * @param joint_q the joint position in radians.
   * @return true if the joint position is reachable.
   */
  bool valid_joint_q(const Joints<DOF>& joint_q) const;
  bool valid_joint_v(const Joints<DOF>& joint_v) const;
  bool safe_joint_q(const Joints<DOF>& joint_q) const;
  array<bool, 6> slow_joint_q(const Joints<DOF>& joint_q) const;

  /**
   * @brief Save the recorded trajectory of the robot arm to a file.
   * @param filepath the path to the file where the recorded data will be saved.
   */
  void record_save(const std::string& filepath);

  /**
   * @brief Load the recorded trajectory of the robot arm from a file.
   * @param filepath the path to the file where the recorded data is saved.
   */
  void record_load(const std::string& filepath);

  /**
   * @brief Start recording the trajectory of the robot arm.
   *
   * The robot will record the joint positions, velocities, and torques, as well as the end position, at each time step.
   * The recorded data can be saved to a file and replayed later.
   *
   * @return true if the recording is started successfully.
   */
  inline bool record_start() { return change_mode(ArmMode::RECORDING); }

  /**
   * @brief Stop recording the trajectory of the robot arm.
   * @return true if the recording is stopped successfully.
   */
  inline bool record_stop() {
    auto current_mode = arm_mode_.load(std::memory_order_relaxed);
    if (current_mode == ArmMode::RECORDING)
      return change_mode(ArmMode::DEMONSTRATE);
    else {
      logger_->warn("current mode is {}, record_stop have no effect", ArmModeStr[current_mode]);
      return false;
    }
  }

  /**
   * @brief Replay the recorded / loaded trajectory of the robot arm.
   * @return true if the replay is started successfully.
   */
  inline bool replay_start() {
    auto current_mode = arm_mode_.load(std::memory_order_relaxed);
    if (current_mode == ArmMode::OFFLINE || current_mode == ArmMode::REPLAY_REACHING ||
        current_mode == ArmMode::REPLAY_WAITING || current_mode == ArmMode::REPLAYING) {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      robot_cmd_data_.replay_request = true;
      return true;
    } else {
      logger_->warn("current mode is {}, replay_start have no effect", ArmModeStr[current_mode]);
      return false;
    }
  }

  /**
   * @brief Enter manual mode. The robot be able to be dragged by hand.
   * @return true if arm mode changes successfully.
   */
  inline bool manual_mode() {
    auto current_mode = arm_mode_.load(std::memory_order_relaxed);
    if (current_mode == ArmMode::DEMONSTRATE || current_mode == ArmMode::RECORDING) {
      logger_->warn("current mode is {}, manual_mode have no effect", ArmModeStr[current_mode]);
      return false;
    } else {
      return change_mode(ArmMode::DEMONSTRATE);
    }
  }

  /**
   * @brief Enter the offline mode. The robot will stop moving and waiting for replay command
   * @return true if arm mode changes successfully.
   */
  inline bool offline_mode() { return change_mode(ArmMode::OFFLINE); }

  /**
   * @brief Enter the online mode. The robot will be able to be controlled by external commands (e.g.
   * set_target_joint_q)
   * @return true if arm mode changes successfully.
   */
  inline bool online_mode() { return change_mode(ArmMode::ONLINE); }

  /**
   * @brief Try to recover from error mode
   */
  inline void reset_error() {
    auto mode = arm_mode_.load(std::memory_order_relaxed);
    if (mode != ArmMode::ERROR) {
      logger_->warn("current mode is {}, reset_error have no effect", ArmModeStr[mode]);
    } else {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      robot_cmd_data_.recover_request = true;
      robot_cmd_data_.cmd_state = MotorControlState::JOINT_POS;
    }
  }

  /**
   * @brief Set the max currents of OD motors when in POS or SPD mode
   */
  inline void set_max_current(const Joints<DOF>& max) {
    for (int i = 0; i < max.size(); i++) motor_driver_[i]->set_max_current(max[i]);
  }
};
};  // namespace arm

template class arm::Robot<6>;

#endif
