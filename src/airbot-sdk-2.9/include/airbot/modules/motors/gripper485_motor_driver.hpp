#ifndef GRIPPER485_MOTOR_DRIVER_HPP
#define GRIPPER485_MOTOR_DRIVER_HPP
#include <mutex>
#include <string>

#include "airbot/modules/motors/motor_driver.hpp"

namespace arm {
class Gripper485MotorDriver : public MotorDriver {
 public:
  Gripper485MotorDriver(uint16_t motor_id, std::string can_interface);
  ~Gripper485MotorDriver();
  virtual uint8_t MotorInit() override;
  virtual void MotorDeInit() override {};
  virtual bool MotorSetZero() override { return true; };
  virtual bool MotorWriteFlash() override { return true; };

  virtual void MotorGetParam(uint8_t param_cmd) override {};
  virtual void MotorPosModeCmd(float pos, float spd, bool ignore_limit) override;
  virtual void MotorSpdModeCmd(float spd) override {};
  virtual void MotorMitModeCmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) override;
  virtual void MotorSetPosParam(float kp, float kd) override {};
  virtual void MotorSetSpdParam(float kp, float ki) override {};
  virtual void MotorSetFilterParam(float position_kd_filter, float kd_spd) override {};
  virtual void set_motor_control_mode(uint8_t motor_control_mode) override {};
  virtual int get_response_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return response_count;
  }
  virtual void set_motor_id(uint8_t motor_id) override {};
  virtual void MotorLock() override {};
  virtual void MotorUnlock() override {};
  virtual void MotorResetID() override {};
  virtual bool CheckId() override { return true; };

 private:
  int response_count = 0;
  mutable std::mutex mutex_;
  virtual void CanSendMsg(const can_frame& tx_frame) override {};

  std::shared_ptr<SocketCAN> can_;
};
}  // namespace arm

#endif
