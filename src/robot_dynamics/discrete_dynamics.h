#ifndef DISCRETE_DYNAMICS_H
#define DISCRETE_DYNAMICS_H

#include "dynamics.h"
#include "robot_dynamics/knotpoint.h"

struct DiscreteDynamics : AbstractModel {};

AbstractKnotPointTemplate auto evaluate(DiscreteDynamics model,
                                        const AbstractKnotPointDeclare &z) {
  return discrete_dynamics(model, z);
}

template <typename P, int Nx, int Nu, typename V, typename T>
auto evaluate(DiscreteDynamics model, P xn, const AbstractKnotPointDeclare &z) {
  discrete_dynamics(model, xn, z);
}

AbstractKnotPointTemplate auto propagate_dynamics(Inplace,
                                                  DiscreteDynamics model,
                                                  AbstractKnotPointDeclare z2,
                                                  AbstractKnotPointDeclare z1) {
  return;
}

AbstractKnotPointTemplate auto propagate_dynamics(StaticReturn,
                                                  DiscreteDynamics model,
                                                  AbstractKnotPointDeclare z2,
                                                  AbstractKnotPointDeclare z1) {
  return;
}

#endif
