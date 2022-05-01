#ifndef OBJECTIVE_H
#define OBJECTIVE_H

#include "base/base.h"
#include "robot_dynamics/functionbase.h"

struct AbstractObjective {};

// template<>
// auto length(AbstractObjective obj) {
//   return length(obj.cost);
// }

inline auto state_dim(AbstractObjective) {
  throw std::runtime_error("Not implemented");
}
inline auto control_dim(AbstractObjective) {
  throw std::runtime_error("Not implemented");
}
inline auto get_J(AbstractObjective) {
  throw std::runtime_error("Not implemented");
}

#endif
