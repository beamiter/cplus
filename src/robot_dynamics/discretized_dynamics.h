#ifndef DISCRETIZED_DYNAMICS_H
#define DISCRETIZED_DYNAMICS_H

#include "discrete_dynamics.h"
#include "integration.h"

template <typename Q> class DiscretizedDynamics : public DiscreteDynamics {
public:
  DiscretizedDynamics() = default;
  virtual ~DiscretizedDynamics() = default;
  // Constructors
  DiscretizedDynamics(const ContinuousDynamics *dynamics_in, Q rule)
      : continuous_dynamics(dynamics_in), integrator(rule) {}
  DiscretizedDynamics(const ContinuousDynamics *dynamics_in)
      : continuous_dynamics(dynamics_in) {}

  // Overriding
  int state_dim() const override { return continuous_dynamics->state_dim(); }
  int control_dim() const override {
    return continuous_dynamics->control_dim();
  }
  int output_dim() const override { return continuous_dynamics->output_dim(); }

  // Members
  const ContinuousDynamics *continuous_dynamics = nullptr;

  Q integrator;
};

// This method is called when using the 'StaticReturn'.
template <typename Q, AbstractKnotPointTypeName>
typename AbstractKnotPointDeclare::state_type
discrete_dynamics(const DiscretizedDynamics<Q> *model,
                  const AbstractKnotPointDeclare &z) {
  return discrete_dynamics<Q, Nx, Nu, V, T>(model, z.state(), z.control(),
                                            z.time(), z.timestep());
}
template <typename Q, AbstractKnotPointTypeName>
typename AbstractKnotPointDeclare::state_type
discrete_dynamics(const DiscretizedDynamics<Q> *model,
                  const typename AbstractKnotPointDeclare::state_type &x,
                  const typename AbstractKnotPointDeclare::control_type &u, T t,
                  T dt) {
  return integrate<ContinuousDynamics, Nx, Nu, V, T>(
      model->integrator, model->continuous_dynamics, x, u, t, dt);
}

// This method is called when using the 'InPlace'.
template <typename Q, AbstractKnotPointTypeName>
void discrete_dynamics(const DiscretizedDynamics<Q> *model,
                       typename AbstractKnotPointDeclare::state_type *xn,
                       const AbstractKnotPointDeclare &z) {
  discrete_dynamics<Q, Nx, Nu, V, T>(model, xn, z.state(), z.control(),
                                     z.time(), z.timestep());
}
template <typename Q, AbstractKnotPointTypeName>
void discrete_dynamics(const DiscretizedDynamics<Q> *model,
                       typename AbstractKnotPointDeclare::state_type *xn,
                       const typename AbstractKnotPointDeclare::state_type &x,
                       const typename AbstractKnotPointDeclare::control_type &u,
                       T t, T dt) {
  // integrate<Nx, Nu, V, T>(
  //     &const_cast<DiscretizedDynamics<Q> *>(model)->integrator,
  //     model->continuous_dynamics, xn, x, u, t, dt);
}

// Function not support partial specialization yet.
template <typename Q, AbstractKnotPointTypeName>
void discrete_dynamics(FunctionSignature sig,
                       const DiscretizedDynamics<Q> *model,
                       typename AbstractKnotPointDeclare::state_type *xn,
                       const AbstractKnotPointDeclare &z) {
  if (sig == FunctionSignature::Inplace) {
    discrete_dynamics(model, xn, z);
  } else {
    xn = discrete_dynamics(model, z);
  }
}

// Propagate dynamics.
template <typename Q, AbstractKnotPointTypeName>
void propagate_dynamics(FunctionSignature sig,
                        const DiscretizedDynamics<Q> *model,
                        AbstractKnotPointDeclare *z2,
                        const AbstractKnotPointDeclare &z1) {
  if (sig == FunctionSignature::Inplace) {
    discrete_dynamics(model, z2->state(), z1);
  } else {
    z2->setstate(discrete_dynamics(model, z1));
  }
}

#endif
