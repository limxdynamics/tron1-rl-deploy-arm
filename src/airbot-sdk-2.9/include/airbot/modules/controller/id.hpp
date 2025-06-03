#ifndef ID_HPP
#define ID_HPP
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <string>
#include <vector>

#include "airbot/modules/controller/chain.hpp"
using std::vector;
namespace arm {

template <std::size_t DOF>
class IDSolver : public ChainDescriptor {
 public:
  virtual Joints<DOF> CartToJnt(const Joints<DOF> &, const Joints<DOF> &, const Wrench &) = 0;
};

}  // namespace arm

#endif
