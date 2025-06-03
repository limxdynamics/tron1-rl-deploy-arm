#ifndef IKV_CHAIN_HPP
#define IKV_CHAIN_HPP

#include <cmath>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <string>
#include <vector>

#include "airbot/modules/controller/ikv.hpp"

using std::vector;
namespace arm {
template <std::size_t DOF>
class ChainIKVSolver : public IKVSolver<DOF> {
  KDL::JntArray joints_last_;
  KDL::ChainIkSolverVel_pinv* ik_solver_;

 public:
  ChainIKVSolver(const std::string&);
  ~ChainIKVSolver();
  virtual Joints<DOF> CartToJnt(const Joints<DOF>&, const Twist&) override;
};
}  // namespace arm
#endif
