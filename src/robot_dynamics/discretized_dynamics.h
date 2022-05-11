#ifndef DISCRETIZED_DYNAMICS_H
#define DISCRETIZED_DYNAMICS_H

#include <functional>

#include "discrete_dynamics.h"

struct QuadratureRule {};

struct Explicit : QuadratureRule {};

struct Implicit : QuadratureRule {};

#define DiscretizedDynamicsDeclare DiscretizedDynamics<F, S>
AbstractModelTemplate class DiscretizedDynamics
    : public DiscreteDynamicsDeclare {
public:
  // Constructors
  DiscretizedDynamics(const ContinuousDynamicsDeclare &dynamics_in,
                      QuadratureRule rule): continuous_dynamics(std::cref(dynamics_in)) {
    integrator = rule;
  }

  // Overriding
  int state_dim() const override { return continuous_dynamics.get().state_dim(); }
  int control_dim() const override { return continuous_dynamics.get().control_dim(); }
  int output_dim() const override { return continuous_dynamics.get().output_dim(); }

  // Members
  std::reference_wrapper<const ContinuousDynamicsDeclare> continuous_dynamics;

  QuadratureRule integrator;
};

#endif
