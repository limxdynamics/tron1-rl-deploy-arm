#ifndef CHAIN_HPP
#define CHAIN_HPP
#include <array>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>

#include "airbot/command/command_types.hpp"
#include "airbot/utils.hpp"
using std::array;

namespace arm {

class InvalidTarget : public std::exception {
 private:
  std::string message;

 public:
  InvalidTarget(const std::string& msg) : message(msg) {}

  const char* what() const noexcept override { return message.c_str(); }
};

class ChainDescriptor {
 protected:
  int num_segments_;
  int num_joints_;
  KDL::Tree tree_;
  KDL::Chain chain_;
  std::shared_ptr<spdlog::logger> logger_;

 public:
  ChainDescriptor();
  virtual void load_model(const std::string&);
  int get_num_joints() { return num_joints_; };
};
extern double bias[6];
}  // namespace arm

#endif
