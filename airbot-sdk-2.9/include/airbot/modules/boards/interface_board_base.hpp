#ifndef INTERFACE_BOARD_BASE_HPP
#define INTERFACE_BOARD_BASE_HPP
#include <cstdint>
#include <string>

#include "airbot/drivers/SocketCAN.hpp"
#include "airbot/modules/boards/board_driver.hpp"
#define COLOR_NONE 0x00
#define COLOR_RED_CONSTANT 0x01
#define COLOR_ORANGE_CONSTANT 0x02
#define COLOR_YELLOW_CONSTANT 0x03
#define COLOR_GREEN_CONSTANT 0x04
#define COLOR_CYAN_CONSTANT 0x05
#define COLOR_BLUE_CONSTANT 0x06
#define COLOR_PURPLE_CONSTANT 0x07
#define COLOR_WHITE_CONSTANT 0x0F
#define COLOR_RED_BREATHING 0x11
#define COLOR_ORANGE_BREATHING 0x12
#define COLOR_YELLOW_BREATHING 0x13
#define COLOR_GREEN_BREATHING 0x14
#define COLOR_CYAN_BREATHING 0x15
#define COLOR_BLUE_BREATHING 0x16
#define COLOR_PURPLE_BREATHING 0x17
#define COLOR_WHITE_BREATHING 0x1F
#define COLOR_RED_FLASHING 0x21
#define COLOR_ORANGE_FLASHING 0x22
#define COLOR_YELLOW_FLASHING 0x23
#define COLOR_GREEN_FLASHING 0x24
#define COLOR_CYAN_FLASHING 0x25
#define COLOR_BLUE_FLASHING 0x26
#define COLOR_PURPLE_FLASHING 0x27
#define COLOR_WHITE_FLASHING 0x2F
#define COLOR_RED_WAVE 0x31
#define COLOR_ORANGE_WAVE 0x32
#define COLOR_YELLOW_WAVE 0x33
#define COLOR_GREEN_WAVE 0x34
#define COLOR_CYAN_WAVE 0x35
#define COLOR_BLUE_WAVE 0x36
#define COLOR_PURPLE_WAVE 0x37
#define COLOR_WHITE_WAVE 0x3F
#define COLOR_RAINBOW_WAVE 0xFF
namespace arm {
class InterfaceBoardBase : public BoardDriver {
 public:
  InterfaceBoardBase(uint16_t board_id, std::string can_interface);
  ~InterfaceBoardBase();
  virtual bool CheckId() override { return true; };

 private:
  void CanSendMsg(const can_frame& tx_frame);
  virtual void CanRxMsgCallback(const can_frame& rx_frame) override;

  std::shared_ptr<SocketCAN> can_;

 protected:
};
}  // namespace arm
#endif
