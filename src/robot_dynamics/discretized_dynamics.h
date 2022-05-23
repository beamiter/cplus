#ifndef DISCRETIZED_DYNAMICS_H
#define DISCRETIZED_DYNAMICS_H

#include <functional>

#include "discrete_dynamics.h"

struct QuadratureRule {};
struct Explicit : QuadratureRule {};
struct Implicit : QuadratureRule {};

class DiscretizedDynamics : public DiscreteDynamics {
public:
  // Constructors
  DiscretizedDynamics(const ContinuousDynamics *dynamics_in,
                      QuadratureRule rule)
      : continuous_dynamics(std::ref(dynamics_in)) {
    integrator = rule;
  }

  // Overriding
  int state_dim() const override {
    return continuous_dynamics.get()->state_dim();
  }
  int control_dim() const override {
    return continuous_dynamics.get()->control_dim();
  }
  int output_dim() const override {
    return continuous_dynamics.get()->output_dim();
  }

  // Members
  std::reference_wrapper<const ContinuousDynamics *> continuous_dynamics;

  QuadratureRule integrator;
};

#endif
