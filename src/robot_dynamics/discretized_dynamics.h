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

  // Overrides.
  int state_dim() const override { return continuous_dynamics->state_dim(); }
  int control_dim() const override {
    return continuous_dynamics->control_dim();
  }
  int output_dim() const override { return continuous_dynamics->output_dim(); }
  typename KP::state_type
  dynamics(const typename KP::state_type &x,
           const typename KP::control_type &u) const override {
    return continuous_dynamics->dynamics(x, u);
  }
  void dynamics(typename KP::ref_vector_type xdot,
                const typename KP::state_type &x,
                const typename KP::control_type &u) const override {
    continuous_dynamics->dynamics(xdot, x, u);
  }
  void jacobian(typename KP::ref_matrix_type jaco,
                typename KP::ref_vector_type y,
                const typename KP::state_type &x,
                const typename KP::control_type &u) const override {
    continuous_dynamics->jacobian(jaco, y, x, u);
  }

  // Members
  const ContinuousDynamics<KP> *continuous_dynamics = nullptr;

  Q<KP> integrator;
};

// This method is called when using the 'StaticReturn'.
template <typename KP, template <typename> class Q>
typename KP::state_type
discretized_dynamics(const DiscretizedDynamics<KP, Q> *model, const KP &z) {
  return discretized_dynamics<KP, Q>(model, z.state(), z.control(), z.time(),
                                     z.timestep());
}
template <typename KP, template <typename> class Q>
typename KP::state_type
discretized_dynamics(const DiscretizedDynamics<KP, Q> *model,
                     const typename KP::state_type &x,
                     const typename KP::control_type &u,
                     typename KP::base_type t, typename KP::base_type dt) {
  return integrate<KP>(model->integrator, model->continuous_dynamics, x, u, t,
                       dt);
}

// This method is called when using the 'InPlace'.
template <typename KP, template <typename> class Q>
void discretized_dynamics(const DiscretizedDynamics<KP, Q> *model,
                          typename KP::ref_vector_type xn, const KP &z) {
  discretized_dynamics<KP, Q>(model, xn, z.state(), z.control(), z.time(),
                              z.timestep());
}
template <typename KP, template <typename> class Q>
void discretized_dynamics(const DiscretizedDynamics<KP, Q> *model,
                          typename KP::ref_vector_type xn,
                          const typename KP::state_type &x,
                          const typename KP::control_type &u,
                          typename KP::base_type t, typename KP::base_type dt) {
  integrate<KP>(&const_cast<DiscretizedDynamics<KP, Q> *>(model)->integrator,
                model->continuous_dynamics, xn, x, u, t, dt);
}

template <typename KP, template <typename> class Q,
          typename FS = FunctionSignature>
void discretized_dynamics(FS sig, const DiscretizedDynamics<KP, Q> *model,
                          typename KP::state_type *xn, const KP &z) {}
template <typename KP, template <typename> class Q>
void discretized_dynamics(Inplace, const DiscretizedDynamics<KP, Q> *model,
                          typename KP::state_type *xn, const KP &z) {
  discretized_dynamics(model, xn, z);
}
template <typename KP, template <typename> class Q>
void discretized_dynamics(StaticReturn, const DiscretizedDynamics<KP, Q> *model,
                          typename KP::state_type *xn, const KP &z) {
  xn = discretized_dynamics(model, z);
}

template <typename KP, template <typename> class Q,
          typename FS = FunctionSignature, typename DM = DiffMethod>
void jacobian(FS sig, DM diff, const DiscretizedDynamics<KP, Q> *model,
              typename KP::ref_matrix_type J, typename KP::ref_vector_type y,
              const KP &z) {}
template <typename KP, template <typename> class Q,
          typename FS = FunctionSignature>
void jacobian(FS sig, UserDefined, const DiscretizedDynamics<KP, Q> *model,
              typename KP::ref_matrix_type J, typename KP::ref_vector_type y,
              const KP &z) {
  jacobian<KP>(&const_cast<DiscretizedDynamics<KP, Q> *>(model)->integrator,
               sig, model->continuous_dynamics, J, y, z.state(), z.control(),
               z.time(), z.timestep());
}

// Propagate dynamics.
template <typename KP, template <typename> class Q,
          typename FS = FunctionSignature>
void propagate_dynamics(FS sig, const DiscretizedDynamics<KP, Q> *model, KP *z2,
                        const KP &z1) {}
template <typename KP, template <typename> class Q>
void propagate_dynamics(Inplace, const DiscretizedDynamics<KP, Q> *model,
                        KP *z2, const KP &z1) {
  discretized_dynamics(model, z2->state(), z1);
}
template <typename KP, template <typename> class Q>
void propagate_dynamics(StaticReturn, const DiscretizedDynamics<KP, Q> *model,
                        KP *z2, const KP &z1) {
  z2->setstate(discretized_dynamics<KP, Q>(model, z1));
}

#endif
