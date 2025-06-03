#ifndef IK_HPP
#define IK_HPP
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <string>
#include <vector>

#include "airbot/modules/controller/chain.hpp"
using std::vector;
namespace arm {

template <std::size_t DOF>
class IKSolver : public ChainDescriptor {
 public:
  virtual vector<Joints<DOF>> CartToJnt(const Frame&) = 0;
};

}  // namespace arm
#endif
