#ifndef DISCRETE_DYNAMICS_H
#define DISCRETE_DYNAMICS_H

#include "dynamics.h"
#include "robot_dynamics/knotpoint.h"

struct DiscreteDynamics : AbstractModel {};

template <int Nx, int Nu, typename V, typename T>
auto evaluate(DiscreteDynamics model,
              const AbstractKnotPoint<Nx, Nu, V, T> &z) {
  return discrete_dynamics(model, z);
}

template <typename P, int Nx, int Nu, typename V, typename T>
auto evaluate(DiscreteDynamics model, P xn,
              const AbstractKnotPoint<Nx, Nu, V, T> &z) {
  discrete_dynamics(model, xn, z);
}

#endif
