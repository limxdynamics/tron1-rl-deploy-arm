#ifndef ID_CHAIN_RNE_HPP
#define ID_CHAIN_RNE_HPP

#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <string>
#include <vector>

#include "airbot/modules/controller/id.hpp"
namespace arm {
template <std::size_t DOF>
class ChainIDSolver : public IDSolver<DOF> {
  KDL::ChainIdSolver_RNE *id_solver_;

 public:
  ChainIDSolver(const std::string &, const std::string &);
  ~ChainIDSolver();
  virtual Joints<DOF> CartToJnt(const Joints<DOF> &, const Joints<DOF> &, const Wrench &) override;
};
}  // namespace arm

#endif
