#ifndef DISCRETIZED_DYNAMICS_H
#define DISCRETIZED_DYNAMICS_H

#include "discrete_dynamics.h"

struct QuadratureRule {};

struct Explicit : QuadratureRule {};

struct Implicit : QuadratureRule {};

// template <typename L = ContinuousDynamics, typename Q = QuadratureRule>
struct DiscretizedDynamics : DiscreteDynamics {
  DiscretizedDynamics(ContinuousDynamics dynamics_in, QuadratureRule rule) {
    continuous_dynamics = dynamics_in;
    integrator = rule;
  }
  ContinuousDynamics continuous_dynamics;
  QuadratureRule integrator;
};

#endif
