// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_CONTROLLER_BASE_H_
#define _LIMX_CONTROLLER_BASE_H_

#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <onnxruntime_cxx_api.h>
#include <robot_common/hardware_interface/ContactSensorInterface.h>
#include <robot_common/hardware_interface/HybridJointInterface.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <airbot_msgs/SetInt32.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <std_msgs/Bool.h>
#include <Eigen/Geometry>

namespace robot_controller {

// Define scalar and vector types for Eigen
using scalar_t = double;
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using vector4_t = Eigen::Matrix<scalar_t, 4, 1>;
using vector5_t = Eigen::Matrix<scalar_t, 5, 1>;
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;

// Utility class to measure time intervals
class TicToc {
 public:
  TicToc() { tic(); }

  void tic() { start = std::chrono::system_clock::now(); }

  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000;
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};

struct Wheel_RobotCfg {
  struct ControlCfg {
    float leg_joint_stiffness{0};     // kp for legged joint
    float leg_joint_damping{0};       // kd for legged joint
    float wheel_joint_stiffness{0};   // kp for wheel joint
    float wheel_joint_damping{0};     // kd for wheel joint

    float arm_j123_stiffness{0};      // kp for arm j123
    float arm_j123_damping{0};        // kd for arm j123
    float arm_j456_stiffness{0};      // kp for arm j456
    float arm_j456_damping{0};        // kd for arm j456

    float action_scale_pos{0};        // scale for action in position
    float action_scale_vel{0};        // scale for action in velocity
    int decimation{0};                // update interval
    float leg_joint_torque_limit{0};  // torque limit for leg joints
    float wheel_joint_torque_limit{0};// torque limit for wheel joints

    float arm_j123_torque_limit{0};   // torque limit for arm j123
    float arm_j456_torque_limit{0};   // torque limit for arm j456

    void print() {
      ROS_INFO_STREAM("=======start ObsScales========");
      ROS_INFO_STREAM("leg stiffness: " << leg_joint_stiffness);
      ROS_INFO_STREAM("leg damping: " << leg_joint_damping);
      ROS_INFO_STREAM("wheel stiffness: " << wheel_joint_stiffness);
      ROS_INFO_STREAM("wheel damping: " << wheel_joint_damping);
      ROS_INFO_STREAM("arm_j123_stiffness: " << arm_j123_stiffness);
      ROS_INFO_STREAM("arm_j123_damping: " << arm_j123_damping);
      ROS_INFO_STREAM("arm_j456_stiffness: " << arm_j456_stiffness);
      ROS_INFO_STREAM("arm_j456_damping: " << arm_j456_damping);
      ROS_INFO_STREAM("action_scale_pos: " << action_scale_pos);
      ROS_INFO_STREAM("action_scale_vel: " << action_scale_vel);
      ROS_INFO_STREAM("decimation: " << decimation);
      ROS_INFO_STREAM("action_scale_vel: " << leg_joint_torque_limit);
      ROS_INFO_STREAM("decimation: " << wheel_joint_torque_limit);
      ROS_INFO_STREAM("arm_j123_torque_limit: " << arm_j123_torque_limit);
      ROS_INFO_STREAM("arm_j456_torque_limit: " << arm_j456_torque_limit);
    }
  };

  struct RlCfg {
    struct ObsScales {
      scalar_t linVel{0};             // observation scale for linear velocity
      scalar_t angVel{0};             // observation scale for angular velocity
      scalar_t dofPos{0};             // observation scale for dof position
      scalar_t dofVel{0};             // observation scale for dof velocity
      scalar_t dofTor{0};             // observation scale for dof torque

      scalar_t heightMeasurements{0}; //observation scale for height measurements(for perception)

      void print() {
        ROS_INFO_STREAM("=======start ObsScales========");
        ROS_INFO_STREAM("linVel: " << linVel);
        ROS_INFO_STREAM("angVel: " << angVel);
        ROS_INFO_STREAM("dofPos: " << dofPos);
        ROS_INFO_STREAM("dofVel: " << dofVel);
        ROS_INFO_STREAM("dofTor: " << dofTor);
        ROS_INFO_STREAM("heightMeasurements: " << heightMeasurements);
        ROS_INFO_STREAM("=======end ObsScales========");
      }
    };

    scalar_t clipActions{0};  // max absolute value for actions
    scalar_t clipObs{0};      // max absolute value for observations
    ObsScales obsScales;
  };

  // User command configuration settings
  struct UserCmdCfg {
    double linVel_x{0.0};
    double linVel_y{0.0};
    double angVel_yaw{0.0};
    scalar_t base_height{0};
    // Print user command scaling parameters
    void print() {
      ROS_INFO_STREAM("=======Start User Cmd Scales========");
      ROS_INFO_STREAM("lin_vel_x: " << linVel_x);
      ROS_INFO_STREAM("lin_vel_y: " << linVel_y);
      ROS_INFO_STREAM("ang_vel_yaw: " << angVel_yaw);
      ROS_INFO_STREAM("=======End User Cmd Scales========\n");
    }
  };
  RlCfg wheelRlCfg;
  UserCmdCfg userCmdCfg;
  std::map<std::string, double> initState;
  ControlCfg controlCfg;

  void print() {
    wheelRlCfg.obsScales.print();
    controlCfg.print();
    userCmdCfg.print();
    std::cerr << "clipActions: " << wheelRlCfg.clipActions << std::endl;
    std::cerr << "clipObs: " << wheelRlCfg.clipObs << std::endl;
  }
};

struct Sole_RobotCfg {
  struct ControlCfg {
    float leg_joint_stiffness{0};
    float leg_joint_damping{0};
    float ankle_joint_stiffness{0};
    float ankle_joint_damping{0};

    float arm_j123_stiffness{0};
    float arm_j123_damping{0};
    float arm_j456_stiffness{0};
    float arm_j456_damping{0};

    float action_scale_pos{0};
    float action_scale_vel{0};
    int decimation{0};
    float leg_joint_torque_limit{0};
    float ankle_joint_torque_limit{0};

    float arm_j123_torque_limit{0};
    float arm_j456_torque_limit{0};

    void print() {
      std::cout << "=======start ObsScales========" << std::endl;
      std::cout << "leg stiffness: " << leg_joint_stiffness << std::endl;
      std::cout << "leg damping: " << leg_joint_damping << std::endl;
      std::cout << "ankle stiffness: " << ankle_joint_stiffness << std::endl;
      std::cout << "ankle damping: " << ankle_joint_damping << std::endl;
      std::cout << "arm_j123_stiffness: " << arm_j123_stiffness << std::endl;
      std::cout << "arm_j123_damping: " << arm_j123_damping << std::endl;
      std::cout << "arm_j456_stiffness: " << arm_j456_stiffness << std::endl;
      std::cout << "arm_j456_damping: " << arm_j456_damping << std::endl;
      std::cout << "action_scale_pos: " << action_scale_pos << std::endl;
      std::cout << "action_scale_vel: " << action_scale_vel << std::endl;
      std::cout << "decimation: " << decimation << std::endl;
      std::cout << "leg_joint_torque_limit: " << leg_joint_torque_limit
                << std::endl;
      std::cout << "ankle_joint_torque_limit: " << ankle_joint_torque_limit
                << std::endl;
      std::cout << "arm_j123_torque_limit: " << arm_j123_torque_limit
                << std::endl;
      std::cout << "arm_j456_torque_limit: " << arm_j456_torque_limit
                << std::endl;
    }
  };
  // gait settings
  struct GaitCfg {
    float frequencies;
    float swing_height;
    void print() {//(const char *tag) {
      ROS_INFO_STREAM("=======Start GaitCfg========");
      ROS_INFO_STREAM("frequencies: " << frequencies);
      ROS_INFO_STREAM("swing_height: " << swing_height);
      ROS_INFO_STREAM("=======end GaitCfg========\n");
    }
  };
  struct EstimationCfg {
    float threshold;
    float threshold_count;
    void print() {
      std::cout << "=======start EstimationCfg========" << std::endl;
      std::cerr << "threshold: " << threshold << std::endl;
      std::cerr << "threshold_count: " << threshold_count << std::endl;
    }
  };

  struct RlCfg {
    struct ObsScales {
      scalar_t linVel{0};
      scalar_t angVel{0};
      scalar_t dofPos{0};
      scalar_t dofVel{0};
      scalar_t heightMeasurements{0};

      void print() {
        std::cout << "=======start ObsScales========" << std::endl;
        std::cerr << "linVel: " << linVel << std::endl;
        std::cerr << "angVel: " << angVel << std::endl;
        std::cerr << "dofPos: " << dofPos << std::endl;
        std::cerr << "dofVel: " << dofVel << std::endl;

        std::cerr << "heightMeasurements: " << heightMeasurements << std::endl;
        std::cout << "=======end ObsScales========" << std::endl;
      }
    };

    scalar_t clipActions{0};
    scalar_t clipObs{0};
    ObsScales obsScales;
  };

  struct UserCmdCfg {
    scalar_t linVel_x{0};
    scalar_t linVel_y{0};
    scalar_t angVel_yaw{0};

    void print() {
      std::cout << "=======User Cmd Scales========" << std::endl;
      std::cerr << "lin_vel_x: " << linVel_x << std::endl;
      std::cerr << "lin_vel_y: " << linVel_y << std::endl;
      std::cerr << "ang_vel_yaw: " << angVel_yaw << std::endl;
    }
  };

  struct UserCmdRecoverCfg {
    scalar_t linVel_x;
    scalar_t linVel_y;
    scalar_t angVel_yaw;

    void print() {
      std::cout << "=======User Cmd Recover Scales========" << std::endl;
      std::cerr << "lin_vel_x: " << linVel_x << std::endl;
      std::cerr << "lin_vel_y: " << linVel_y << std::endl;
      std::cerr << "ang_vel_yaw: " << angVel_yaw << std::endl;
    }
  };

  RlCfg soleRlCfg;
  UserCmdCfg userCmdCfg;
  UserCmdRecoverCfg userCmdRecoverCfg;
  std::map<std::string, double> initState;
  std::map<std::string, double> initStandState;
  ControlCfg controlCfg;
  GaitCfg gaitCfg;
  EstimationCfg estimationCfg;

  // Print robot configuration settings
  void print() {
    soleRlCfg.obsScales.print();
    controlCfg.print();
    userCmdCfg.print();
    userCmdRecoverCfg.print();
    gaitCfg.print();
    estimationCfg.print();
    ROS_INFO_STREAM("clipActions: " << soleRlCfg.clipActions);
    ROS_INFO_STREAM("clipObs: " << soleRlCfg.clipObs);
  }
};

// Base class for controllers
class ControllerBase : public controller_interface::MultiInterfaceController<
                           robot_common::HybridJointInterface,
                           hardware_interface::ImuSensorInterface,
                           robot_common::ContactSensorInterface> {
 public:
  enum class Mode : uint8_t;  // Enumeration for controller modes

  ControllerBase() = default;  // Default constructor

  virtual ~ControllerBase() = default;  // Virtual destructor

  // Initialize the controller
  virtual bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh);

  // Perform actions when the controller starts
  virtual void starting(const ros::Time &time) {}

  // Update the controller
  virtual void update(const ros::Time &time, const ros::Duration &period) {}

  // Load the model for the controller
  virtual bool loadModel() { return false; }

  // Load RL configuration settings
  virtual bool loadRLCfg() { return false; }

  // Compute encoder for the controller
  virtual void computeEncoder() {}

  // Compute actions for the controller
  virtual void computeActions() {}

  // Compute observations for the controller
  virtual void computeObservation() {}
  
  // Handle wheel stand/move
  virtual void handleWheelStandMode() {}
  virtual void handleRLWheelMoveMode() {}
  
  // Handle sole stand/move
  virtual void handleSoleStandMode() {}
  virtual void handleRLSoleWalkMode() {}

 protected:
  // Callback function for velocity commands
  virtual void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg) {};
  // Callback function for eepose

  virtual Wheel_RobotCfg &getWheelRobotCfg() { return wheelRobotCfg_; }

  virtual Sole_RobotCfg &getSoleRobotCfg() { return soleRobotCfg_; }

  Mode mode_;
  int64_t loopCount_;
  int64_t loopCountKeep_;
  vector3_t commands_;
  vector_t eeCommands_;

  vector4_t commandsMove_;
  vector4_t commandsSit_;
  vector4_t commandsStand_;
  vector3_t scaledCommands_;
  scalar_t headingCommand_{0.0};
  scalar_t yawRateCommand_{0.0};
  Wheel_RobotCfg wheelRobotCfg_{};
  Sole_RobotCfg soleRobotCfg_{};

  std::vector<std::string> jointNames_;
  std::string robotType_;

  // hardware interface
  std::vector<robot_common::HybridJointHandle> hybridJointHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandles_;
  ros::NodeHandle nh_;

  vector_t defaultJointAngles_;         // start joint angles
  vector_t initJointAngles_;            // init joint angles in standard stand mode

  scalar_t standPercent_;
  scalar_t standDuration_;

  ros::Publisher debugPub_;
  ros::Subscriber cmdVelSub_;
  // ros::Subscriber EEPoseCmdSub_;

  int isSim_;
};
// Function to compute square of a value
template <typename T>
T square(T a) {
  return a * a;
}

// Function to convert quaternion to ZYX Euler angles
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T> &q) {
  Eigen::Matrix<SCALAR_T, 3, 1> zyx;

  SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  zyx(0) =
      std::atan2(2 * (q.x() * q.y() + q.w() * q.z()),
                 square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
  zyx(1) = std::asin(as);
  zyx(2) =
      std::atan2(2 * (q.y() * q.z() + q.w() * q.x()),
                 square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
  return zyx;
}

// Function to compute rotation matrix from ZYX Euler angles
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> getRotationMatrixFromZyxEulerAngles(
    const Eigen::Matrix<SCALAR_T, 3, 1> &eulerAngles) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);

  const SCALAR_T c1 = cos(z);
  const SCALAR_T c2 = cos(y);
  const SCALAR_T c3 = cos(x);
  const SCALAR_T s1 = sin(z);
  const SCALAR_T s2 = sin(y);
  const SCALAR_T s3 = sin(x);

  const SCALAR_T s2s3 = s2 * s3;
  const SCALAR_T s2c3 = s2 * c3;

  // Construct rotation matrix
  Eigen::Matrix<SCALAR_T, 3, 3> rotationMatrix;
  rotationMatrix << c1 * c2, c1 * s2s3 - s1 * c3, c1 * s2c3 + s1 * s3, s1 * c2,
      s1 * s2s3 + c1 * c3, s1 * s2c3 - c1 * s3, -s2, c2 * s3, c2 * c3;
  return rotationMatrix;
}
// Function to compute quaternion from rpy
template <typename SCALAR_T>
Eigen::Quaternion<SCALAR_T> getQuaternionFromRpy(
    const Eigen::Matrix<SCALAR_T, 3, 1> &rpy) {
  SCALAR_T roll = rpy(0);
  SCALAR_T pitch = rpy(1);
  SCALAR_T yaw = rpy(2);

  SCALAR_T cy = cos(yaw * 0.5);
  SCALAR_T sy = sin(yaw * 0.5);
  SCALAR_T cp = cos(pitch * 0.5);
  SCALAR_T sp = sin(pitch * 0.5);
  SCALAR_T cr = cos(roll * 0.5);
  SCALAR_T sr = sin(roll * 0.5);

  Eigen::Quaternion<SCALAR_T> q;
  q.w() = cr * cp * cy + sr * sp * sy;
  q.x() = sr * cp * cy - cr * sp * sy;
  q.y() = cr * sp * cy + sr * cp * sy;
  q.z() = cr * cp * sy - sr * sp * cy;

  return q;
}
// Function to compute rpy from rotation matrix
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> getRpyFromRotationMatrix(
    const Eigen::Matrix<SCALAR_T, 3, 3> &rotationMatrix) {
  Eigen::Matrix<SCALAR_T, 3, 1> rpy;

  SCALAR_T pitch = atan2(-rotationMatrix(2, 0),
                         sqrt(rotationMatrix(2, 1) * rotationMatrix(2, 1) +
                              rotationMatrix(2, 2) * rotationMatrix(2, 2)));

  SCALAR_T yaw = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));

  SCALAR_T roll = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));

  rpy << roll, pitch, yaw;
  return rpy;
}
// Function to compute quaternion from rotation matrix
template <typename SCALAR_T>
Eigen::Quaternion<SCALAR_T> getQuaternionFromRotationMatrix(
    const Eigen::Matrix<SCALAR_T, 3, 3> &rotationMatrix) {
  Eigen::Quaternion<SCALAR_T> q;
  SCALAR_T trace = rotationMatrix.trace();

  if (trace > 0.0) {
    SCALAR_T s = sqrt(trace + 1.0) * 2.0;
    q.w() = 0.25 * s;
    q.x() = (rotationMatrix(2, 1) - rotationMatrix(1, 2)) / s;
    q.y() = (rotationMatrix(0, 2) - rotationMatrix(2, 0)) / s;
    q.z() = (rotationMatrix(1, 0) - rotationMatrix(0, 1)) / s;
  } else if ((rotationMatrix(0, 0) > rotationMatrix(1, 1)) &&
             (rotationMatrix(0, 0) > rotationMatrix(2, 2))) {
    SCALAR_T s = sqrt(1.0 + rotationMatrix(0, 0) - rotationMatrix(1, 1) -
                      rotationMatrix(2, 2)) *
                 2.0;
    q.w() = (rotationMatrix(2, 1) - rotationMatrix(1, 2)) / s;
    q.x() = 0.25 * s;
    q.y() = (rotationMatrix(0, 1) + rotationMatrix(1, 0)) / s;
    q.z() = (rotationMatrix(0, 2) + rotationMatrix(2, 0)) / s;
  } else if (rotationMatrix(1, 1) > rotationMatrix(2, 2)) {
    SCALAR_T s = sqrt(1.0 + rotationMatrix(1, 1) - rotationMatrix(0, 0) -
                      rotationMatrix(2, 2)) *
                 2.0;
    q.w() = (rotationMatrix(0, 2) - rotationMatrix(2, 0)) / s;
    q.x() = (rotationMatrix(0, 1) + rotationMatrix(1, 0)) / s;
    q.y() = 0.25 * s;
    q.z() = (rotationMatrix(1, 2) + rotationMatrix(2, 1)) / s;
  } else {
    SCALAR_T s = sqrt(1.0 + rotationMatrix(2, 2) - rotationMatrix(0, 0) -
                      rotationMatrix(1, 1)) *
                 2.0;
    q.w() = (rotationMatrix(1, 0) - rotationMatrix(0, 1)) / s;
    q.x() = (rotationMatrix(0, 2) + rotationMatrix(2, 0)) / s;
    q.y() = (rotationMatrix(1, 2) + rotationMatrix(2, 1)) / s;
    q.z() = 0.25 * s;
  }
  return q;
}
}  // namespace robot_controller
#endif  //_LIMX_CONTROLLER_BASE_H_