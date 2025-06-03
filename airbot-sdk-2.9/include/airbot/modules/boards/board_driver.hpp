#ifndef BOARD_DRIVER_HPP
#define BOARD_DRIVER_HPP
#include <stdint.h>

#include <chrono>
#include <string>
#include <thread>

#include "airbot/drivers/SocketCAN.hpp"
namespace arm {
class BoardDriver {
 public:
  typedef enum {
    CMD_DEVICE_ID,
    CMD_HARDWARE_VERSION,
    CMD_FIRMWARE_VERSION,
    CMD_BOARD_SN_CODE,
    CMD_ARM_SN_CODE,
    CMD_TYPE,
    CMD_MODE,
    CMD_NAME,
    CMD_MOTOR_BODY_SN_CODE,
    CMD_MOTOR_SN_CODE,
    CMD_RESERVER0A,
    CMD_RESERVER0B,
    CMD_RESERVER0C,
    CMD_RESERVER0D,
    CMD_RESERVER0E,
    CMD_RESERVER0F,
    CMD_POS_BIAS,
    CMD_POS,
    CMD_FORCE,
    CMD_SNAP_SIGNAL,
    CMD_PRODUCT_FLAG,
    CMD_LED_MODE,
  } CmdId;

  typedef enum {
    FRAME_1 = 1,
    FRAME_2,
    FRAME_3,
    FRAME_4,
  } FrameId;

  typedef enum {
    SNAP_RELEASE = 0,
    SNAP_SHORT_PRESS = 1,
    SNAP_LONG_PRESS = 2,
    SNAP_DOUBLE_PRESS = 3,
  } snap_mode_e;

  BoardDriver();
  ~BoardDriver() {};
  virtual uint8_t Init();
  uint16_t get_board_id() { return board_id_; }
  uint32_t get_ret_id() { return ret_id_; }
  std::string get_firmware_version() const { return firmware_version_; }
  std::string get_hardware_version() const { return hardware_version_; }
  std::string get_sn_code() const { return board_sn_code_; }
  std::string get_arm_sn_code() const { return arm_sn_code_; }
  std::string get_body_sn_code() const { return body_sn_code_; }
  std::string get_whole_sn_code() const { return whole_sn_code_; }
  snap_mode_e get_snap_mode();
  uint32_t get_product_flag() const { return product_flag_; }
  void GetCmd(uint8_t cmd_id);
  void SetCmd(uint8_t cmd_id);
  void SetCmd(uint8_t cmd_id, uint8_t framd_id, uint8_t* data);
  void SetCmd(uint8_t cmd_id, uint8_t framd_id, uint32_t data);
  void SetCmd(uint8_t cmd_id, uint8_t framd_id, float data);
  void SetCmd(uint8_t cmd_id, uint8_t framd_id, double data);
  virtual bool CheckId() = 0;
  virtual void CanSendMsg(const can_frame& tx_frame) = 0;
  bool IsSnCodeValid(const std::string& sn_code);
  std::string get_board_type() const { return board_type_; }

 protected:
  virtual void CanRxMsgCallback(const can_frame& rx_frame);
  uint16_t board_id_;
  std::string hardware_version_;
  std::string firmware_version_;
  std::string board_sn_code_ = "----------------";
  std::string arm_sn_code_ = "----------------";
  std::string body_sn_code_ = "----------------";
  std::string whole_sn_code_ = "----------------";
  std::string board_type_;
  uint32_t ret_id_ = -1;
  snap_mode_e snap_mode_ = SNAP_RELEASE;
  uint32_t product_flag_ = -1;

  int DEVICE_ID;
  int OPERATE_CMD = 0b0000;
  int GET_CMD = (OPERATE_CMD | 0);
  int SET_CMD = (OPERATE_CMD | 1);
  int RET_CMD = (OPERATE_CMD | 2);
  int GET_CMD_ID;
  int SET_CMD_ID;
  int RET_CMD_ID;
  int BROADCASE_CMD_ID = 0x7ff;
  std::shared_ptr<spdlog::logger> logger_;
};
}  // namespace arm
#endif
