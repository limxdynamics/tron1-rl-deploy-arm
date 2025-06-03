#ifndef OD_MOTOR_DRIVER_HPP
#define OD_MOTOR_DRIVER_HPP
#include <mutex>

#include "airbot/modules/motors/motor_driver.hpp"
#define kM_NTC_MOTOR_BETA 3950
#define kSI_MOTOR_POLES 14
#define kTIMEOUT 500
#define kS_PID_KP_36 0.006
#define kS_PID_KI_36 0.1
#define kS_PID_KD_36 1.4
#define kP_PID_KP_36 0.006
#define kP_PID_KI_36 0
#define kP_PID_KD_36 4e-05
#define kS_PID_KP_10 0.006
#define kS_PID_KI_10 0.1
#define kS_PID_KD_10 0.39
#define kP_PID_KP_10 0.006
#define kP_PID_KI_10 0
#define kP_PID_KD_10 4e-05
#define kS_PID_KP_1 0.006
#define kS_PID_KI_1 0.1
#define kS_PID_KD_1 1.4
#define kP_PID_KP_1 0.0004
#define kP_PID_KI_1 0
#define kP_PID_KD_1 0
#define kMIN_RESISTANCE 0.25
#define kMAX_RESISTANCE 0.40
#define kMIN_INDUCTANCE 0.000007
#define kMAX_INDUCTANCE 0.000013
#define kERROR 0.0001

namespace arm {
class OdMotorDriver : public MotorDriver {
 public:
  enum KeyValidParam {
    TIMEOUT = 0,
    M_NTC_MOTOR_BETA,
    SI_MOTOR_POLES,
    GEAR_RATIO,
    S_PID_KP,
    S_PID_KI,
    S_PID_KD,
    P_PID_KP,
    P_PID_KI,
    P_PID_KD,
    FOC_MOTOR_R,
    FOC_MOTOR_L,
  };
  enum MotorCmdId {
    CMD_POS_KP = 0x15,
    CMD_POS_KI,
    CMD_POS_KD,
    CMD_POS_FILTER,
    CMD_SPD_KP,
    CMD_SPD_KI,
    CMD_SPD_KD,
    CMD_SPD_FILTER,
    CMD_ENCODER_DEG,
    CMD_MOTOR_DEG,
    CMD_POS_KP_STORE = 0x25,
    CMD_POS_KI_STORE,
    CMD_POS_KD_STORE,
    CMD_POS_FILTER_STORE,
    CMD_SPD_KP_STORE,
    CMD_SPD_KI_STORE,
    CMD_SPD_KD_STORE,
    CMD_SPD_FILTER_STORE,
    CMD_ENCODER_INITIAL,
  };
  OdMotorDriver(uint16_t motor_id, std::string can_interface);
  ~OdMotorDriver();
  virtual uint8_t Init() override;
  virtual void MotorLock() override;
  virtual void MotorUnlock() override;
  virtual uint8_t MotorInit() override;
  virtual void MotorDeInit() override;
  virtual bool MotorSetZero() override;
  virtual bool MotorWriteFlash() override;

  virtual void MotorGetParam(uint8_t param_cmd) override;
  virtual void MotorPosModeCmd(float pos, float spd, bool ignore_limit) override;
  virtual void MotorSpdModeCmd(float spd) override;
  virtual void MotorMitModeCmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) override;
  virtual void MotorSetPosParam(float kp, float kd) override;
  virtual void MotorSetSpdParam(float kp, float ki) override;
  virtual void MotorSetFilterParam(float position_kd_filter, float kd_spd) override;
  virtual void MotorResetID();
  virtual bool CheckId();
  virtual void set_motor_id(uint8_t motor_id) override;
  virtual void set_motor_control_mode(uint8_t motor_control_mode) override;
  float get_motor_kd_pos_filter() { return motor_kd_pos_filter; }
  float get_p_pid_ang_div() { return p_pid_ang_div; }
  float get_m_ntc_motor_beta() { return m_ntc_motor_beta; }
  uint8_t get_si_motor_poles() { return si_motor_poles; }
  float get_foc_motor_r() { return foc_motor_r; }
  float get_foc_motor_l() { return foc_motor_l; }
  float get_encoder_initial() { return encoder_initial_; }
  float get_encoder_deg() { return encoder_deg_; }
  virtual int get_response_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return response_count;
  }
  uint32_t MotorKeyParamValid();
  bool MotorSetPosBias(float pos_bias);

 private:
  int response_count = 0;
  mutable std::mutex mutex_;
  const float kKpMin = 0.0f;
  const float kKpMax = 500.0f;
  const float kKdMin = 0.0f;
  const float kKdMax = 50.0f;
  const float kIMin = -30.0f;
  const float kIMax = 30.0f;
  const float kPosMin = -12.5f;
  const float kPosMax = 12.5f;
  const float kSpdMin = -18.0f;
  const float kSpdMax = 18.0f;
  const float kTorqueMin = -30.0f;
  const float kTorqueMax = 30.0f;
  const float kOutPutMin = 0.0f;
  const float kOutPutMax = 36000.0f;
  virtual void CanRxMsgCallback(const can_frame& rx_frame) override;
  void ConfigOdMotor(uint8_t cmd);
  void OdSetPosParam(uint16_t fdbKP, uint16_t fdbKD, uint8_t ack_status);
  void OdSetSpdParam(uint16_t linkage, uint16_t speedKI, uint8_t ack_status);
  void OdSetFilterParam(uint16_t position_kd_filter, uint16_t kd_spd, uint8_t ack_status);
  void OdSetCommunicationMode(uint8_t communication_mode);  // 01: automatic telegram 02: answer mode
  virtual void CanSendMsg(const can_frame& tx_frame) override;
  uint8_t ack_status_ = 1;
  uint8_t communication_mode_ = 0;
  float motor_kd_pos_filter = 0.f;  // od 10 位置环微分滤波系数
  float p_pid_ang_div;              // od 0x0E 位置环PID角度除数
  float m_ntc_motor_beta;           // od 0x0F 温度系数
  uint8_t si_motor_poles;           // od 0x10 电机极对数
  float foc_motor_r;                // od 0x11 电机电阻
  float foc_motor_l;                // od 0x12 电机电感
  float power_;
  float encoder_initial_;
  float encoder_deg_;

  std::shared_ptr<SocketCAN> can_;
};
}  // namespace arm
#endif
