#ifndef DISCRETIZED_DYNAMICS_H
#define DISCRETIZED_DYNAMICS_H

#include "discrete_dynamics.h"
#include "integration.h"

template <typename KP, template <typename> class Q>
class DiscretizedDynamics : public DiscreteDynamics<KP> {
public:
  DiscretizedDynamics() = default;
  virtual ~DiscretizedDynamics() = default;
  // Constructors
  DiscretizedDynamics(const ContinuousDynamics<KP> *dynamics_in, Q<KP> rule)
      : continuous_dynamics(dynamics_in), integrator(rule) {}
  DiscretizedDynamics(const ContinuousDynamics<KP> *dynamics_in)
      : continuous_dynamics(dynamics_in) {}

  // Overriding
  int state_dim() const override { return continuous_dynamics->state_dim(); }
  int control_dim() const override {
    return continuous_dynamics->control_dim();
  }
  int output_dim() const override { return continuous_dynamics->output_dim(); }
  void jacobian(typename KP::jacobian_type &jaco,
                const typename KP::state_type &y,
                const typename KP::state_type &x,
                const typename KP::control_type &u) const override {
    // CHECK(0);
    continuous_dynamics->jacobian(jaco, y, x, u);
  }

  // Members
  const ContinuousDynamics<KP> *continuous_dynamics = nullptr;

  Q<KP> integrator;
};

// This method is called when using the 'StaticReturn'.
template <typename KP, template <typename> class Q>
typename KP::state_type
discrete_dynamics(const DiscretizedDynamics<KP, Q> *model, const KP &z) {
  return discrete_dynamics<KP, Q>(model, z.state(), z.control(), z.time(),
                                  z.timestep());
}
template <typename KP, template <typename> class Q>
typename KP::state_type
discrete_dynamics(const DiscretizedDynamics<KP, Q> *model,
                  const typename KP::state_type &x,
                  const typename KP::control_type &u, typename KP::base_type t,
                  typename KP::base_type dt) {
  return integrate<KP>(model->integrator, model->continuous_dynamics, x, u, t,
                       dt);
}

// This method is called when using the 'InPlace'.
template <typename KP, template <typename> class Q>
void discrete_dynamics(const DiscretizedDynamics<KP, Q> *model,
                       typename KP::state_type *xn, const KP &z) {
  discrete_dynamics<KP, Q>(model, xn, z.state(), z.control(), z.time(),
                           z.timestep());
}
template <typename KP, template <typename> class Q>
void discrete_dynamics(const DiscretizedDynamics<KP, Q> *model,
                       typename KP::state_type *xn,
                       const typename KP::state_type &x,
                       const typename KP::control_type &u,
                       typename KP::base_type t, typename KP::base_type dt) {
  integrate<KP>(&const_cast<DiscretizedDynamics<KP, Q> *>(model)->integrator,
                model->continuous_dynamics, xn, x, u, t, dt);
}

// Function not support partial specialization yet.
template <typename KP, template <typename> class Q>
void discrete_dynamics(FunctionSignature sig,
                       const DiscretizedDynamics<KP, Q> *model,
                       typename KP::state_type *xn, const KP &z) {
  if (sig == FunctionSignature::Inplace) {
    discrete_dynamics(model, xn, z);
  } else {
    xn = discrete_dynamics(model, z);
  }
}

// Propagate dynamics.
template <typename KP, template <typename> class Q>
void propagate_dynamics(FunctionSignature sig,
                        const DiscretizedDynamics<KP, Q> *model, KP *z2,
                        const KP &z1) {
  if (sig == FunctionSignature::Inplace) {
    discrete_dynamics(model, z2->state(), z1);
  } else {
    z2->setstate(discrete_dynamics<KP, Q>(model, z1));
  }
}

#endif
