#ifndef DISCRETE_DYNAMICS_H
#define DISCRETE_DYNAMICS_H

#include "dynamics.h"
#include "robot_dynamics/knotpoint.h"

struct DiscreteDynamics : AbstractModel {};

ABSTRACT_KNOT_POINT_TEMPLATE
auto evaluate(DiscreteDynamics model, CONST_ABSTRACT_KNOT_POINT_REF z) {
  return discrete_dynamics(model, z);
}

template <typename P, ABSTRACT_KNOT_POINT_TYPENAME>
auto evaluate(DiscreteDynamics model, P xn, CONST_ABSTRACT_KNOT_POINT_REF z) {
  discrete_dynamics(model, xn, z);
}

#endif
