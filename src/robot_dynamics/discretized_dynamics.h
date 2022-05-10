#ifndef DISCRETIZED_DYNAMICS_H
#define DISCRETIZED_DYNAMICS_H

#include "discrete_dynamics.h"

struct QuadratureRule {};

struct Explicit : QuadratureRule {};

struct Implicit : QuadratureRule {};

AbstractModelTemplate class DiscretizedDynamics
    : public DiscreteDynamicsDeclare {
  DiscretizedDynamics(const ContinuousDynamicsDeclare &dynamics_in,
                      QuadratureRule rule) {
    continuous_dynamics = dynamics_in;
    integrator = rule;
  }
  const ContinuousDynamicsDeclare &continuous_dynamics = nullptr;
  QuadratureRule integrator;
};

#endif
