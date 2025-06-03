#ifndef INTERFACE_BOARD_END_HPP
#define INTERFACE_BOARD_END_HPP
#include <stdint.h>

#include <string>

#include "airbot/drivers/SocketCAN.hpp"
#include "airbot/modules/boards/board_driver.hpp"
namespace arm {
class InterfaceBoardEnd : public BoardDriver {
 public:
  InterfaceBoardEnd(uint16_t board_id, std::string can_interface);
  ~InterfaceBoardEnd();
  virtual bool CheckId() override { return true; };

 private:
  void CanSendMsg(const can_frame& tx_frame);
  virtual void CanRxMsgCallback(const can_frame& rx_frame) override;
  std::shared_ptr<SocketCAN> can_;

 protected:
};
}  // namespace arm
#endif
