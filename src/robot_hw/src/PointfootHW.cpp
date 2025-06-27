// Copyright information
//
// © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.

/*
 * This file contains the implementation of the PointfootHW class, which is a hardware interface
 * for controlling a legged robot with point foot contacts. It utilizes ROS (Robot Operating System)
 * for communication and control.
 */

#include "robot_hw/PointfootHW.h"

namespace hw {
static const std::string CONTROLLER_NAME = "/controllers/pointfoot_controller";

// Method to start the biped controller
bool PointfootHW::startBipedController() {
  controller_manager_msgs::ListControllers list_controllers;
  if (!list_controllers_client_.call(list_controllers)) {
      ROS_ERROR("Failed to call list controllers service.");
      return false;
  }

  for (const auto& controller : list_controllers.response.controller) {
    if (controller.name == controller_name_ && controller.state == "running") {
      ROS_WARN("Controller %s is already running, skipping start.", controller.name.c_str());
      return false;
    }
  }

  // Creating a message to switch controllers
  controller_manager_msgs::SwitchController sw;
  sw.request.start_controllers.push_back(controller_name_);
  sw.request.start_asap = false;
  sw.request.strictness = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;
  sw.request.timeout = ros::Duration(3.0).toSec();

  // Calling the controller_manager service to switch controllers
  if (switch_controllers_client_.call(sw.request, sw.response)) {
    if (sw.response.ok) {
      ROS_INFO("Start controller %s successfully.", sw.request.start_controllers[0].c_str());
      return true;
    } else {
      ROS_WARN("Start controller %s failed.", sw.request.start_controllers[0].c_str());
    }
  } else {
    ROS_WARN("Failed to start controller %s.", sw.request.start_controllers[0].c_str());
  }
  return false;
}

// Method to stop the biped controller
bool PointfootHW::stopBipedController() {
  std_msgs::Float32MultiArray ee_rc_cmd_delta_msg;
  ee_rc_cmd_delta_msg.data.resize(8);
  ee_rc_cmd_delta_msg.data[7] = -1;
  ee_rc_cmd_delta_pub_.publish(ee_rc_cmd_delta_msg);
  // std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(1.0 * 1e9)));
  prepare_stop_ = true;
  return true;

  // Creating a message to switch controllers
  // controller_manager_msgs::SwitchController sw;
  // sw.request.stop_controllers.push_back(controller_name_);
  // sw.request.start_asap = false;
  // sw.request.strictness = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;
  // sw.request.timeout = ros::Duration(3.0).toSec();

  // // Calling the controller_manager service to switch controllers
  // if (switch_controllers_client_.call(sw.request, sw.response)) {
  //   if (sw.response.ok) {
  //     ROS_INFO("Stop controller %s successfully.", sw.request.stop_controllers[0].c_str());
  //   } else {
  //     ROS_WARN("Stop controller %s failed.", sw.request.stop_controllers[0].c_str());
  //   }
  // } else {
  //   ROS_WARN("Failed to stop controller %s.", sw.request.stop_controllers[0].c_str());
  // }

  // for (int i = 0; i < robot_->getMotorNumber(); ++i) {
  //   robotCmd_.q[i] = jointData_[i].posDes_ = 0.0;
  //   robotCmd_.dq[i] = jointData_[i].velDes_ = 0.0;
  //   robotCmd_.Kp[i] = jointData_[i].kp_ = 0.0;
  //   robotCmd_.tau[i] = jointData_[i].tau_ff_ = 0.0;
  //   robotCmd_.Kd[i] = jointData_[i].kd_ = 1.0;
  // }
  // robot_->publishRobotCmd(robotCmd_);

  // std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(1.0 * 1e9)));

  // return true;
}

// Method to initialize the hardware interface
bool PointfootHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  // Initializing the legged robot instance
  robot_ = limxsdk::PointFoot::getInstance();
  
  const char* value = ::getenv("ROBOT_TYPE");
  if (value && strlen(value) > 0) {
    robot_type_ = std::string(value);
    if (robot_type_.find("PF") != std::string::npos) {
      ROS_ERROR("Error: PF with arm is not supported.");
      abort();
    }
  } else {
    ROS_ERROR("Error: Please set the ROBOT_TYPE using 'export ROBOT_TYPE=<robot_type>'.");
    abort();
  }
  // Determine the specific robot configuration based on the robot type
  is_wheel_foot_ = (robot_type_.find("WF") != std::string::npos);
  is_sole_foot_  = (robot_type_.find("SF") != std::string::npos);

  // read imu offset
  try {
    YAML::Node config = YAML::LoadFile("/opt/limx/imptdata/ctrl_TRON1_arm.yaml");
    if (config[robot_type_ + "_imu_offset_pitch"]) {
      imu_offset_pitch_ = config[robot_type_ + "_imu_offset_pitch"].as<double>();
    }
    if (config[robot_type_ + "_imu_offset_roll"]) {
      imu_offset_roll_ = config[robot_type_ + "_imu_offset_roll"].as<double>();
    }
  } catch (const YAML::Exception& e) {
    ROS_WARN_STREAM("Failed to parse YAML file: " << e.what() << " creating new empty file");
    imu_offset_pitch_ = imu_offset_roll_ = 0.0;
    std::ofstream fout("/opt/limx/imptdata/ctrl_TRON1_arm.yaml", std::ios::out);
    fout.close();
  }

  if (is_wheel_foot_)
  {
    controller_name_ = "/controllers/wheelfoot_controller";
    joint_num_ = 8; //8;
    robot_subtype = 2;
  }
  if (is_sole_foot_)
  {
    controller_name_ = "/controllers/solefoot_controller";
    joint_num_ = 8;
    robot_subtype = 3;
  }

  usercmd_last_.axes.resize(4, 0.0);
  usercmd_last_.buttons.resize(18, 0);
  std::cerr << "robot_type_ = " << robot_type_ << std::endl; 

  // std::string is_sim_str;
  bool is_sim = false;
  bool rn = root_nh.param<bool>("/is_sim", is_sim, 0);
  is_sim_ = int(is_sim);
  if (is_sim_ == 1) {
    calibration_state_ = 0;
  } else {
    calibration_state_ = -1;
  }

  is_recover_done_ = false;

  // Initializing the RobotHW base class
  if (!RobotHW::init(root_nh, robot_hw_nh)) {
    return false;
  }

  // Arm interface
  if(is_sim_ == 0)
  {
    // is_sim_ = 0;
    ROS_INFO("Real World Deploy , Airbot Arm SDK Init!");

    std::string urdf_path = URDF_INSTALL_PATH + "airbot_play_with_gripper.urdf";
    std::string can_interface = "can0";
    std::string direction = "down";
    std::string end_mode = "gripper";
    double joint_vel_limit = 3.14;
    ROS_INFO("Initializing arm interface");
    airbot_arm_hw_ = std::make_unique<arm::Robot<6>>(urdf_path,can_interface,direction,joint_vel_limit,end_mode);
    ROS_INFO("Arm interface initialized!");
    airbot_arm_cmd_.init(6);
    airbot_arm_feedback_.init(6);

    Joints<6> init_zeros_position =  {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

    airbot_arm_hw_->set_target_joint_q(init_zeros_position);

    gripper_cmd_sub_ = robot_hw_nh.subscribe<std_msgs::Bool>("/gripper_cmd", 10, &PointfootHW::gripperCmdCallback, this);
  }
  else{
    // is_sim_ = 1;
    ROS_INFO("Simulation Deploy , Airbot Arm ROS Topic Init!");
    airbot_msgs::ArmState init_state;
    // init_state->na = 6;
    init_state.q.resize(6, 0);
    init_state.dq.resize(6, 0);
    // init_state->vd.resize(6, 0);
    init_state.tau.resize(6, 0);
    armstate_buffer_.writeFromNonRT(init_state);
    
    armstate_sub_ = root_nh.subscribe<airbot_msgs::ArmState>("/ArmState", 10, &PointfootHW::armStateCallback, this);

    armcmd_pub_ = root_nh.advertise<airbot_msgs::ArmCmd>("/ArmCmd", false, 10);

    // armCmd_.na = 6;
    armCmd_.q.resize(6, 0);
    armCmd_.v.resize(6, 0); // in limxsdk v is replaced by dq
    armCmd_.tau.resize(6, 0);
    armCmd_.kp.resize(6, 0);
    armCmd_.kd.resize(6, 4.0);
    // armCmd_.mode.resize(6, 0);
  }

  // Resize the jointData_ vector to match the number of motors in the robot
  // jointData_.resize(robot_->getMotorNumber());
  jointData_.resize(robot_->getMotorNumber() + 6);

  arm_init_flag_ = false;
  arm_activate_flag_ = false;
  setupJoints();
  setupImu();
  setupContactSensor(robot_hw_nh);

  // robotCmd_.na = joint_num_;
  robotCmd_.q.resize(robot_->getMotorNumber(), 0);
  robotCmd_.dq.resize(robot_->getMotorNumber(), 0);
  robotCmd_.tau.resize(robot_->getMotorNumber(), 0);
  robotCmd_.Kp.resize(robot_->getMotorNumber(), 0);
  robotCmd_.Kd.resize(robot_->getMotorNumber(), 4.0);
  robotCmd_.mode.resize(robot_->getMotorNumber(), 0);

  limxsdk::RobotState init_state;
  // init_state->na = joint_num_;
  init_state.q.resize(robot_->getMotorNumber(), 0);
  init_state.dq.resize(robot_->getMotorNumber(), 0);
  // init_state->vd.resize(init_state->na, 0);
  init_state.tau.resize(robot_->getMotorNumber(), 0);
  robotstate_buffer_.writeFromNonRT(init_state);

  // Initializing robot command instance, state buffer and imu buffer
  robotCmd_ = limxsdk::RobotCmd(robot_->getMotorNumber());
  robotstate_buffer_.writeFromNonRT(limxsdk::RobotState(robot_->getMotorNumber()));
  imudata_buffer_.writeFromNonRT(limxsdk::ImuData());

  // Subscribing to robot state
  robot_->subscribeRobotState([this](const limxsdk::RobotStateConstPtr& msg) {
    robotstate_buffer_.writeFromNonRT(*msg);
  });

  // Subscribing to robot imu
  robot_->subscribeImuData([this](const limxsdk::ImuDataConstPtr& msg) {
    imudata_buffer_.writeFromNonRT(*msg);
  });

  // Retrieving joystick configuration parameters
  root_nh.getParam("/joystick_buttons", joystick_btn_map_); // TODO joystick in yg is Buttons
  for (auto button: joystick_btn_map_) {
    ROS_INFO_STREAM("Joystick button: [" << button.first << ", " << button.second << "]");
  }

  root_nh.getParam("/joystick_axes", joystick_axes_map_); // TODO joystick in yg is Axes
  for (auto axes: joystick_axes_map_) {
    ROS_INFO_STREAM("Joystick axes: [" << axes.first << ", " << axes.second << "]");
  }

  // When deploying on the real machine, this part receives data from the robot controller and processes it.
  // You can customize it according to your needs.
  robot_->subscribeSensorJoy([this](const limxsdk::SensorJoyConstPtr& msg) {
    // Logic for starting biped controller
    if (calibration_state_ == 0 && joystick_btn_map_.count("L1") > 0 && joystick_btn_map_.count("Y") > 0) {
      if (msg->buttons[joystick_btn_map_["L1"]] == 1 && msg->buttons[joystick_btn_map_["Y"]] == 1) {
        startBipedController();
        arm_init_flag_ = true;
        baseControllerStart_ = ros::Time::now();
      }
    }

    // Logic for stopping biped controller
    if (joystick_btn_map_.count("L1") > 0 && joystick_btn_map_.count("X") > 0) {
      if (msg->buttons[joystick_btn_map_["L1"]] == 1 && msg->buttons[joystick_btn_map_["X"]] == 1) {
        ROS_WARN("L1 + X stopping controller!"); // ROS_FATAL
        if (!is_sim_) {
          maintain_grasp_ = true; // close the gripper before exit
        }
        stopBipedController();
        // abort();
      }
    }

    // preparing stop
    if (prepare_stop_) {
      prepare_stop_count_++;
      if (prepare_stop_count_ > 320){
        ROS_FATAL("stopping controller finished!"); // ROS_FATAL
        // Creating a message to switch controllers
        controller_manager_msgs::SwitchController sw;
        sw.request.stop_controllers.push_back(controller_name_);
        sw.request.start_asap = false;
        sw.request.strictness = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;
        sw.request.timeout = ros::Duration(3.0).toSec();

        // Calling the controller_manager service to switch controllers
        if (switch_controllers_client_.call(sw.request, sw.response)) {
          if (sw.response.ok) {
            ROS_INFO("Stop controller %s successfully.", sw.request.stop_controllers[0].c_str());
          } else {
            ROS_WARN("Stop controller %s failed.", sw.request.stop_controllers[0].c_str());
          }
        } else {
          ROS_WARN("Failed to stop controller %s.", sw.request.stop_controllers[0].c_str());
        }
        for (int i = 0; i < robot_->getMotorNumber(); ++i) {
          robotCmd_.q[i] = jointData_[i].posDes_ = 0.0;
          robotCmd_.dq[i] = jointData_[i].velDes_ = 0.0;
          robotCmd_.Kp[i] = jointData_[i].kp_ = 0.0;
          robotCmd_.tau[i] = jointData_[i].tau_ff_ = 0.0;
          robotCmd_.Kd[i] = jointData_[i].kd_ = 1.0;
        }
        robot_->publishRobotCmd(robotCmd_);

        std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(1.0 * 1e9)));
        abort();
      }
    }

    // Publishing cmd_vel based on joystick input
    if (joystick_axes_map_.count("left_horizon") > 0 && joystick_axes_map_.count("left_vertical") > 0
      && joystick_axes_map_.count("right_horizon") > 0 && joystick_axes_map_.count("right_vertical") > 0) {
      static ros::Time lastpub;
      ros::Time now = ros::Time::now();
      if (fabs(now.toSec() - lastpub.toSec()) >= (1.0 / 30)) {
        geometry_msgs::Twist twist;
        twist.linear.x = msg->axes[joystick_axes_map_["left_vertical"]] * 0.5;
        twist.linear.y = msg->axes[joystick_axes_map_["left_horizon"]] * 0.5;
        twist.angular.z = msg->axes[joystick_axes_map_["right_horizon"]] * 0.5;
        cmd_vel_pub_.publish(twist);
        lastpub = now;
      }
    }

    if (joystick_btn_map_.count("SELECT") > 0 && msg->buttons[joystick_btn_map_["SELECT"]] == 1)
    {
      btn_continue_time_  = ros::Time::now();
      if (last_select_value_ == 0)
      {
        select_btn_changed_ = true;
        btn_trigger_time_  = ros::Time::now();
      }
    } 
    else{
      btn_continue_time_ = ros::Time(0.0);
      btn_trigger_time_ = ros::Time(0.0);
    }
    last_select_value_ = msg->buttons[joystick_btn_map_["SELECT"]];
    if (joystick_axes_map_.count("left_horizon") > 0 && joystick_axes_map_.count("left_vertical") > 0 
        && joystick_axes_map_.count("right_horizon") > 0 && joystick_axes_map_.count("right_vertical") > 0
        && joystick_btn_map_.count("U") > 0 && joystick_btn_map_.count("D") > 0 
        && joystick_btn_map_.count("L") > 0 && joystick_btn_map_.count("R") > 0 ) {
      static ros::Time lastpub;
      ros::Time now = ros::Time::now();
      if ((now.toSec() - lastpub.toSec()) >= (1.0 / 30)) {
        geometry_msgs::Twist twist;
        // compute imu orientation offset
        if (msg->buttons[joystick_btn_map_["R2"]] == 1 &&
          usercmd_last_.buttons[joystick_btn_map_["L"]] == 0 && msg->buttons[joystick_btn_map_["L"]] == 1)
        {
          imu_offset_roll_ = imu_offset_roll_ - msg->buttons[joystick_btn_map_["L"]] * 0.003;
          imu_offset_changed_ = true;
        }
        if (msg->buttons[joystick_btn_map_["R2"]] == 1 &&
          usercmd_last_.buttons[joystick_btn_map_["R"]] == 0 && msg->buttons[joystick_btn_map_["R"]] == 1)
        {
          imu_offset_roll_ = imu_offset_roll_ + msg->buttons[joystick_btn_map_["R"]] * 0.003;
          imu_offset_changed_ = true;
        }
        if (msg->buttons[joystick_btn_map_["R2"]] == 1 &&
          usercmd_last_.buttons[joystick_btn_map_["U"]] == 0 && msg->buttons[joystick_btn_map_["U"]] == 1)
        {
          imu_offset_pitch_ = imu_offset_pitch_ + msg->buttons[joystick_btn_map_["U"]] * 0.003;
          imu_offset_changed_ = true;
        }
        if (msg->buttons[joystick_btn_map_["R2"]] == 1 &&
          usercmd_last_.buttons[joystick_btn_map_["D"]] == 0 && msg->buttons[joystick_btn_map_["D"]] == 1)
        {
          imu_offset_pitch_ = imu_offset_pitch_ - msg->buttons[joystick_btn_map_["D"]] * 0.003;
          imu_offset_changed_ = true;
        }

        imu_offset_roll_ = std::min(std::max(imu_offset_roll_, -0.105), 0.105);
        imu_offset_pitch_ = std::min(std::max(imu_offset_pitch_, -0.105), 0.105);

        twist.angular.x = imu_offset_roll_;
        twist.angular.y = imu_offset_pitch_;
        // if ((now.toSec() - cmdVelFromSDKLast_.toSec()) >= 1.0) {
        twist.linear.x = msg->axes[joystick_axes_map_["left_vertical"]] * 0.5;
        twist.linear.y = msg->axes[joystick_axes_map_["left_horizon"]] * 0.5;
        twist.linear.z = msg->axes[joystick_axes_map_["right_vertical"]] * 1.0;
        twist.angular.z = msg->axes[joystick_axes_map_["right_horizon"]] * 0.5;
        cmd_vel_pub_.publish(twist);

        // write imu offset change to yaml
        if (imu_offset_changed_)
        {
          std::string limxArmPath = "/opt/limx/imptdata/ctrl_TRON1_arm.yaml";
          YAML::Node exist_config = YAML::LoadFile(limxArmPath);
          std::map<std::string, double> offset_map;
          // iterate existing yaml data
          for (YAML::const_iterator it = exist_config.begin(); it != exist_config.end(); ++it) {
            std::string key = it->first.as<std::string>();
            double offset = (it->second).as<double>();
            offset_map[key] = offset;
          }
          // add new key to map(override if exists)
          offset_map[robot_type_ + "_imu_offset_pitch"] = imu_offset_pitch_;
          offset_map[robot_type_ + "_imu_offset_roll"] = imu_offset_roll_;
          YAML::Emitter out;
          out << YAML::BeginMap;
          for (const auto& rpy_offset : offset_map) {
            out << YAML::Key << rpy_offset.first << YAML::Value << rpy_offset.second;
          }
          out << YAML::EndMap;
          std::ofstream fout(limxArmPath);
          if (fout.good()) {
            fout << out.c_str();
            ROS_INFO_STREAM("IMU offset saved to: " << limxArmPath << " pitch: "
               << imu_offset_pitch_ << " roll: " << imu_offset_roll_);
          } else {
            ROS_ERROR_STREAM("Failed to write to file: " << limxArmPath);
          }
          imu_offset_changed_ = false;
        }

        std_msgs::Float32MultiArray ee_rc_cmd_delta_msg;
        ee_rc_cmd_delta_msg.data.resize(6+1);
        // x y z R2 pressed is the imu offset, skip arm command
        if(msg->buttons[joystick_btn_map_["R2"]] == 0)
        {
          ee_rc_cmd_delta_msg.data[1] = 0.003*msg->buttons[joystick_btn_map_["L"]]-0.003*msg->buttons[joystick_btn_map_["R"]];
          if(msg->buttons[joystick_btn_map_["L2"]] == 1)
          {
            ee_rc_cmd_delta_msg.data[0] = 0;
            ee_rc_cmd_delta_msg.data[2] = 0.003*msg->buttons[joystick_btn_map_["U"]]-0.003*msg->buttons[joystick_btn_map_["D"]];
          }
          else{
            ee_rc_cmd_delta_msg.data[0] = 0.003*msg->buttons[joystick_btn_map_["U"]]-0.003*msg->buttons[joystick_btn_map_["D"]];
            ee_rc_cmd_delta_msg.data[2] = 0;
          }
        }
        // r p y 
        // pitch
        ee_rc_cmd_delta_msg.data[4] = 0.01*msg->buttons[joystick_btn_map_["Y"]]-0.01*msg->buttons[joystick_btn_map_["A"]];
        if(msg->buttons[joystick_btn_map_["L2"]] == 1)
        {
          ee_rc_cmd_delta_msg.data[3] = 0;
          // yaw
          ee_rc_cmd_delta_msg.data[5] = 0.01*msg->buttons[joystick_btn_map_["X"]]-0.01*msg->buttons[joystick_btn_map_["B"]];
        }
        else
        {
          // roll
          ee_rc_cmd_delta_msg.data[3] = 0.01*msg->buttons[joystick_btn_map_["X"]]-0.01*msg->buttons[joystick_btn_map_["B"]];
          ee_rc_cmd_delta_msg.data[5] = 0;
        }

        if(msg->buttons[joystick_btn_map_["R1"]] == 1)
        {
          ee_rc_cmd_delta_msg.data[6] = 1;
        }
        else{
          ee_rc_cmd_delta_msg.data[6] = 0;
        }
        
        ee_rc_cmd_delta_pub_.publish(ee_rc_cmd_delta_msg);
        lastpub = now;
        usercmd_last_ = *msg;
      }
    }
  });
  /*
   * Subscribing to diagnostic values for calibration state
   */
  robot_->subscribeDiagnosticValue([&](const limxsdk::DiagnosticValueConstPtr& msg) {
    // Check if the diagnostic message pertains to calibration
    if (msg->name == "calibration") {
      ROS_WARN("Calibration state: %d, msg: %s", msg->code, msg->message.c_str());
      calibration_state_ = msg->code;
      if (calibration_state_ == 0)
      {
        for (int i = 0; i < joint_num_; ++i)
        {
          robotCmd_.q[i] = 0.0;
          robotCmd_.dq[i] = 0.0;
          robotCmd_.Kp[i] = 0.0;
          robotCmd_.Kd[i] = 6.0;
          robotCmd_.tau[i] = 0.0;
          robotCmd_.mode[i] = 0;
        }
        robot_->publishRobotCmd(robotCmd_);
      }
    }
    // Check if the diagnostic message pertains to EtherCAT
    if (msg->name == "ethercat" && msg->level == diagnostic_msgs::DiagnosticStatus::ERROR) {
      ROS_FATAL("Ethercat code: %d, msg: %s", msg->code, msg->message.c_str());
      stopBipedController();
      abort();
    }

    // Check if the diagnostic message pertains to IMU
    if (msg->name == "imu" && msg->level == diagnostic_msgs::DiagnosticStatus::ERROR) {
      ROS_FATAL("IMU code: %d, msg: %s", msg->code, msg->message.c_str());
      stopBipedController();
      abort();
    }
  });
  // Advertising cmd_vel topic for publishing twist commands
  cmd_vel_pub_ = robot_hw_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // Initializing ROS service clients for controller
  switch_controllers_client_ = robot_hw_nh.serviceClient<controller_manager_msgs::SwitchController>("/pointfoot_hw/controller_manager/switch_controller");
  list_controllers_client_ = robot_hw_nh.serviceClient<controller_manager_msgs::ListControllers>("/pointfoot_hw/controller_manager/list_controllers");
  ee_rc_cmd_delta_pub_ = robot_hw_nh.advertise<std_msgs::Float32MultiArray>("/EEPose_cmd_rc", false, 10);

  return true;
}

// Method to read data from hardware
void PointfootHW::read(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  // Reading robot state
  limxsdk::RobotState robotstate = *robotstate_buffer_.readFromRT(); // size is 8
  for (int i = 0; i < robot_->getMotorNumber(); ++i) {
    jointData_[i].pos_ = robotstate.q[i];
    jointData_[i].vel_ = robotstate.dq[i];
    jointData_[i].tau_ = robotstate.tau[i];
  }
  // Reading imu data
  limxsdk::ImuData imudata = *imudata_buffer_.readFromRT();
  imuData_.ori_[0] = imudata.quat[1];
  imuData_.ori_[1] = imudata.quat[2];
  imuData_.ori_[2] = imudata.quat[3];
  imuData_.ori_[3] = imudata.quat[0];
  imuData_.angularVel_[0] = imudata.gyro[0];
  imuData_.angularVel_[1] = imudata.gyro[1];
  imuData_.angularVel_[2] = imudata.gyro[2];
  imuData_.linearAcc_[0] = imudata.acc[0];
  imuData_.linearAcc_[1] = imudata.acc[1];
  imuData_.linearAcc_[2] = imudata.acc[2];

  if(is_sim_)
  {
    //Simulation
    auto armstate = armstate_buffer_.readFromRT();
    for (int i = 0; i < 6; ++i) {
      jointData_[i + joint_num_].pos_ = armstate->q[i];
      jointData_[i + joint_num_].vel_ = armstate->dq[i];
      jointData_[i + joint_num_].tau_ = armstate->tau[i];
    }
  }
  else{
    //Real robot
    airbot_arm_feedback_.current_joint_q = airbot_arm_hw_->get_current_joint_q();
    airbot_arm_feedback_.current_joint_v = airbot_arm_hw_->get_current_joint_v();
    airbot_arm_feedback_.current_joint_t = airbot_arm_hw_->get_current_joint_t();
    for (int i = 0; i < 6; ++i) {
      jointData_[i + joint_num_].pos_ = airbot_arm_feedback_.current_joint_q[i];
      jointData_[i + joint_num_].vel_ = airbot_arm_feedback_.current_joint_v[i];
      jointData_[i + joint_num_].tau_ = airbot_arm_feedback_.current_joint_t[i];
    }
  }
}

// Method to write commands to hardware
void PointfootHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  static long loop_cnt = 0;
  bool arm_pub_flag = false;
  double start_controller_diff = (ros::Time::now() - baseControllerStart_).toSec();
  if( loop_cnt%5 == 0 && start_controller_diff > 4.0 && start_controller_diff < 1e9)
  {
    arm_pub_flag = true;
  }
  loop_cnt++;
  if(loop_cnt>=5)
  {
    loop_cnt = 0;
  }
  // Writing commands to robot
  for (int i = 0; i < robot_->getMotorNumber(); ++i) {
    robotCmd_.q[i] = static_cast<float>(jointData_[i].posDes_);
    robotCmd_.dq[i] = static_cast<float>(jointData_[i].velDes_);
    robotCmd_.Kp[i] = static_cast<float>(jointData_[i].kp_);
    robotCmd_.Kd[i] = static_cast<float>(jointData_[i].kd_);
    robotCmd_.tau[i] = static_cast<float>(jointData_[i].tau_ff_);
    robotCmd_.mode[i] = static_cast<float>(jointData_[i].mode_);
  }

  // Publishing robot commands
  if (calibration_state_ == 0) {
    robot_->publishRobotCmd(robotCmd_);
  }
  // publish arm
  if(is_sim_)
  {
    // Simulation
    for (int ai = 0; ai < 6; ++ai)
    {
      // PD controller is in simulation interface
      armCmd_.q[ai] = static_cast<float>(jointData_[ai + joint_num_].posDes_);
      armCmd_.v[ai] = static_cast<float>(jointData_[ai + joint_num_].velDes_);
      armCmd_.kp[ai] = static_cast<float>(jointData_[ai + joint_num_].kp_);
      armCmd_.kd[ai] = static_cast<float>(jointData_[ai + joint_num_].kd_);
      armCmd_.tau[ai] = static_cast<float>(jointData_[ai + joint_num_].tau_ff_);
    }
    if(arm_pub_flag){
      armcmd_pub_.publish(armCmd_);
    }
  }
  else{
    // only pulish when cmd is not zeros
    if(arm_init_flag_)
    {
      for (int i = 0; i < 6; ++i)
      {
        float pos_des = static_cast<float>(jointData_[i + joint_num_].posDes_) * 
          std::min((highDampingCnt_ + 50) / 180.0, 1.0);
        airbot_arm_cmd_.target_joint_q[i] = static_cast<float>(pos_des);
        airbot_arm_cmd_.target_joint_v[i] = static_cast<float>(jointData_[i + joint_num_].velDes_);
        airbot_arm_cmd_.target_joint_kp[i] = static_cast<float>(jointData_[i + joint_num_].kp_);
        airbot_arm_cmd_.target_joint_kd[i] = static_cast<float>(jointData_[i + joint_num_].kd_);
      }
      if(arm_pub_flag){
        float kd_factor = highDampingCnt_ < 200 ? 3.0 : 1.0;
        for (int i = 0; i < 6; ++i)
        {
          airbot_arm_cmd_.target_joint_kd[i] *= kd_factor;
        }
        // update grasp flag
        bool print_info = false;
        if(gripper_cmd_ == true && last_gripper_cmd_ == false && maintain_grasp_ == false)
        {
          ROS_INFO("received grasp open command");
          maintain_grasp_ = true;
          print_info = true;
        }
        // when gripper cmd changes, modify maintain_grasp_
        else if (gripper_cmd_ == true && last_gripper_cmd_ == false && maintain_grasp_ == true)
        {
          ROS_INFO("received grasp close command");
          maintain_grasp_ = false;
          print_info = true;
        }
        last_gripper_cmd_ = gripper_cmd_;
        bool send_success=
        airbot_arm_hw_->set_target_joint_mit(airbot_arm_cmd_.target_joint_q,
                                            airbot_arm_cmd_.target_joint_v,
                                            airbot_arm_cmd_.target_joint_kp,
                                            airbot_arm_cmd_.target_joint_kd);
        grasp(maintain_grasp_, print_info);
        if(!send_success)
        {
          ROS_WARN_STREAM("Failed to send arm command to airbot arm!!");
        }
        highDampingCnt_++;
      }
    }
    else{}
  }
}

// Method to setup joints based on URDF
bool PointfootHW::setupJoints() {
  for (const auto& joint : urdfModel_->joints_) {
    int index = -1;
    if(joint.first.find("abad_L")!=std::string::npos) index = 0;
    if(joint.first.find("hip_L")!=std::string::npos) index = 1;
    if(joint.first.find("knee_L")!=std::string::npos) index = 2;
    
    if(joint.first.find("abad_R")!=std::string::npos) index = 4;
    if(joint.first.find("hip_R")!=std::string::npos) index = 5;
    if(joint.first.find("knee_R")!=std::string::npos) index = 6;
    // wheel foot
    if(robot_subtype == 2)
    {
      if(joint.first.find("wheel_L")!=std::string::npos) index = 3;
      if(joint.first.find("wheel_R")!=std::string::npos) index = 7;
    }

    // sole foot
    if(robot_subtype == 3)
    {
      if(joint.first.find("ankle_L")!=std::string::npos) index = 3;
      if(joint.first.find("ankle_R")!=std::string::npos) index = 7;
    }
    

    if(joint.first.find("J1")!=std::string::npos) index = 8;
    if(joint.first.find("J2")!=std::string::npos) index = 9;
    if(joint.first.find("J3")!=std::string::npos) index = 10;
    if(joint.first.find("J4")!=std::string::npos) index = 11;
    if(joint.first.find("J5")!=std::string::npos) index = 12;
    if(joint.first.find("J6")!=std::string::npos) index = 13;

    if (index < 0)
      continue;

    hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_,
                                                      &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);
    hybridJointInterface_.registerHandle(robot_common::HybridJointHandle(state_handle, &jointData_[index].posDes_,
                                                                         &jointData_[index].velDes_,
                                                                         &jointData_[index].kp_, &jointData_[index].kd_,
                                                                         &jointData_[index].tau_ff_, &jointData_[index].mode_));
  }

  return true;
}

// Method to setup IMU sensor
bool PointfootHW::setupImu() {
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("limx_imu", "limx_imu", imuData_.ori_,
                                                                           imuData_.oriCov_, imuData_.angularVel_,
                                                                           imuData_.angularVelCov_, imuData_.linearAcc_,
                                                                           imuData_.linearAccCov_));
  return true;
}

// Method to setup contact sensors
bool PointfootHW::setupContactSensor(ros::NodeHandle& nh) {
  nh.getParam("/robot_hw/contact_threshold", contactThreshold_);
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactSensorInterface_.registerHandle(robot_common::ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contactState_[i]));
  }
  return true;
}

// Method to load URDF model
bool PointfootHW::loadUrdf(ros::NodeHandle& nh) {
  std::string urdfString;
  if (urdfModel_ == nullptr) {
    urdfModel_ = std::make_shared<urdf::Model>();
  }
  // Getting the URDF parameter from the parameter server
  nh.getParam("robot_description", urdfString);
  return !urdfString.empty() && urdfModel_->initString(urdfString);
}

void PointfootHW::grasp(bool to_grasp, bool print_info)
{
  if (to_grasp) {
    if (!is_sim_) {
      bool opened = airbot_arm_hw_->set_target_end(0.0);
      if (print_info)
        ROS_INFO_STREAM("set gripper open status: " << opened);
    }
  } else {
    if (!is_sim_) {
      bool closed = airbot_arm_hw_->set_target_end(1.0);
      if (print_info)
        ROS_INFO_STREAM("set gripper close status: " << closed);
    }
  }
}

void PointfootHW::gripperCmdCallback(const std_msgs::BoolConstPtr& msg) {
  gripper_cmd_ = msg->data;
}

void PointfootHW::armStateCallback(const airbot_msgs::ArmStateConstPtr& msg) {
  armstate_buffer_.writeFromNonRT(*msg);
}

}  // namespace hw
