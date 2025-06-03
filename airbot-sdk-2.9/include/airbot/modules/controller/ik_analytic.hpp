#ifndef IK_ANALYTIC_HPP
#define IK_ANALYTIC_HPP
#include <Eigen/Dense>
#include <vector>

#include "airbot/modules/controller/ik.hpp"

using std::vector;
namespace arm {
template <std::size_t DOF>
class AnalyticIKSolver : public IKSolver<DOF> {
 public:
  AnalyticIKSolver(const std::string&);
  ~AnalyticIKSolver() {};
  virtual vector<Joints<DOF>> CartToJnt(const Frame&) override;
};
}  // namespace arm

#endif
