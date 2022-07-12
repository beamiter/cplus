#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <functional>

#include "robot_dynamics/functionbase.h"
#include "robot_dynamics/knotpoint.h"

// AbstractModel.
template <typename KP> class AbstractModel : public AbstractFunction<KP> {
public:
  virtual typename KP::state_type
  dynamics(const typename KP::state_type &x,
           const typename KP::control_type &u) const {
    CHECK(0);
    return x;
  }
  virtual void dynamics(typename KP::ref_vector_type xdot,
                        const typename KP::state_type &x,
                        const typename KP::control_type &u) const {
    CHECK(0);
  }
  // Overrides.
  int output_dim() const override { return this->state_dim(); }
  // virtual void jacobian(typename KP::ref_matrix_type jaco,
  //                       typename KP::ref_vector_type y,
  //                       const typename KP::state_type &x,
  //                       const typename KP::control_type &u) const override {
  //   CHECK(0);
  // }
};

// ContinuousDynamics.
template <typename KP> class ContinuousDynamics : public AbstractModel<KP> {};

// StaticReturn.
template <typename KP>
typename KP::state_type dynamics(const ContinuousDynamics<KP> *model,
                                 const KP *z) {
  return dynamics(model, z->state(), z->control(), z->time());
}
template <typename KP>
typename KP::state_type
dynamics(const ContinuousDynamics<KP> *model, const typename KP::state_type &x,
         const typename KP::control_type &u, typename KP::base_type t) {
  return model->dynamics(x, u);
}

// Inplace.
template <typename KP>
void dynamics(const ContinuousDynamics<KP> *model,
              typename KP::ref_vector_type xdot, const KP *z) {
  dynamics(model, xdot, z->state(), z->control(), z->time());
}
template <typename KP>
void dynamics(const ContinuousDynamics<KP> *model,
              typename KP::ref_vector_type xdot,
              const typename KP::state_type &x,
              const typename KP::control_type &u, typename KP::base_type t) {
  model->dynamics(xdot, x, u);
}

// Depends on the FunctionSignature.
template <typename KP, typename TP = FunctionSignature>
void dynamics(TP sig, const ContinuousDynamics<KP> *model,
              typename KP::ref_vector_type xdot, const KP &z) {
  static_assert(std::is_base_of<FunctionSignature, TP>::value,
                "TP is not derived of FunctionSignature");
}
template <typename KP>
void dynamics(Inplace, const ContinuousDynamics<KP> *model,
              typename KP::ref_vector_type xdot, const KP &z) {
  dynamics(model, xdot, z);
}
template <typename KP>
void dynamics(StaticReturn, const ContinuousDynamics<KP> *model,
              typename KP::ref_vector_type xdot, const KP &z) {
  xdot = dynamics(model, z);
}

template <typename KP>
typename KP::state_type
evaluate(const ContinuousDynamics<KP> *model, const typename KP::state_type &x,
         const typename KP::control_type &u, const typename KP::param_type &p) {
  return dynamics(model, x, u, p.first);
}
template <typename KP>
void evaluate(const ContinuousDynamics<KP> *model,
              const typename KP::ref_vector_type xdot,
              const typename KP::state_type &x,
              const typename KP::control_type &u,
              const typename KP::param_type &p) {
  dynamics(model, xdot, x, u, p.first);
}
template <typename KP, typename DM = DiffMethod>
void jacobian(FunctionSignature, DM diff, const ContinuousDynamics<KP> *model,
              typename KP::ref_matrix_type J, typename KP::ref_vector_type xdot,
              const KP &z) {}
template <typename KP>
void jacobian(FunctionSignature, UserDefined,
              const ContinuousDynamics<KP> *model,
              typename KP::ref_matrix_type J, typename KP::ref_vector_type xdot,
              const KP &z) {
  jacobian(model, J, xdot, z.state(), z.constrol(), z.time());
}
template <typename KP>
void jacobian(const ContinuousDynamics<KP> *model,
              typename KP::ref_matrix_type J, typename KP::ref_vector_type xdot,
              const typename KP::state_type &x,
              const typename KP::control_type &u, typename KP::base_type t) {
  model->jacobian(J, xdot, x, u);
}

#endif
