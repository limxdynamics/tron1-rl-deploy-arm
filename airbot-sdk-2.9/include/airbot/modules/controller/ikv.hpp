#ifndef IKV_HPP
#define IKV_HPP
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <string>
#include <vector>

#include "airbot/modules/controller/chain.hpp"
using std::vector;
namespace arm {

template <std::size_t DOF>
class IKVSolver : public ChainDescriptor {
 public:
  virtual Joints<DOF> CartToJnt(const Joints<DOF>&, const Twist&) = 0;
};

}  // namespace arm
#endif
