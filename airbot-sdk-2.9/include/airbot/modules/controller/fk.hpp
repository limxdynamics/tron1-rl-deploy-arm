#ifndef FK_HPP
#define FK_HPP
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <string>
#include <vector>

#include "airbot/modules/controller/chain.hpp"
using std::array;
using std::vector;
namespace arm {

template <std::size_t DOF>
class FKSolver : public ChainDescriptor {
 public:
  virtual Frame JntToCart(const Joints<DOF>&) = 0;
};
}  // namespace arm

#endif
