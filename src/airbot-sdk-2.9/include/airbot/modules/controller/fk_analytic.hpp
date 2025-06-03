#ifndef FK_ANALYTIC_HPP
#define FK_ANALYTIC_HPP
#include <spdlog/spdlog.h>

#include "airbot/modules/controller/fk.hpp"

namespace arm {
template <std::size_t DOF>
class AnalyticFKSolver : public FKSolver<DOF> {
 public:
  AnalyticFKSolver(const std::string&);
  ~AnalyticFKSolver() {};
  virtual Frame JntToCart(const Joints<DOF>&) override;
};
}  // namespace arm
#endif
