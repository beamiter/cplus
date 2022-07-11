#ifndef DISCRETE_DYNAMICS_H
#define DISCRETE_DYNAMICS_H

#include <memory>

#include "dynamics.h"
#include "robot_dynamics/knotpoint.h"

template <typename KP> class DiscreteDynamics : public AbstractModel<KP> {
public:
  virtual ~DiscreteDynamics() = default;
  int state_dim() const override {
    CHECK(0);
    return -1;
  }
  int control_dim() const override {
    CHECK(0);
    return -1;
  }
};

// Evaluate for static return.
template <typename KP>
typename KP::state_type evaluate(const DiscreteDynamics<KP> *model,
                                 const KP &z) {
  return discrete_dynamics(model, z);
}
// Evaluate for in-place.
template <typename KP>
void evaluate(const DiscreteDynamics<KP> *model,
              typename KP::ref_vector_type xn, const KP &z) {
  discrete_dynamics(model, xn, z);
}

// This method is called when using the 'StaticReturn'.
template <typename KP>
typename KP::state_type discrete_dynamics(const DiscreteDynamics<KP> *model,
                                          const KP &z) {
  return discrete_dynamics<KP>(model, z.state(), z.control(), z.time(),
                               z.timestep());
}
template <typename KP>
typename KP::state_type discrete_dynamics(const DiscreteDynamics<KP> *model,
                                          const typename KP::state_type &x,
                                          const typename KP::control_type &u,
                                          typename KP::base_type t,
                                          typename KP::base_type dt) {
  return x + model->dynamics(x, u) * dt;
}

// This method is called when using the 'InPlace'.
template <typename KP>
void discrete_dynamics(const DiscreteDynamics<KP> *model,
                       typename KP::ref_vector_type xn, const KP &z) {
  discrete_dynamics<KP>(model, xn, z.state(), z.control(), z.time(),
                        z.timestep());
}
template <typename KP>
void discrete_dynamics(const DiscreteDynamics<KP> *model,
                       typename KP::ref_vector_type xn,
                       const typename KP::state_type &x,
                       const typename KP::control_type &u,
                       typename KP::base_type t, typename KP::base_type dt) {
  model->dynamics(xn, x, u);
  xn *= dt;
  xn += x;
}

// Function not support partial specialization yet.
template <typename KP>
void discrete_dynamics(FunctionSignature sig, const DiscreteDynamics<KP> *model,
                       typename KP::ref_vector_type xn, const KP &z) {
  if (sig == FunctionSignature::Inplace) {
    discrete_dynamics(model, xn, z);
  } else {
    xn = discrete_dynamics(model, z);
  }
}

// Propagate dynamics.
template <typename KP>
void propagate_dynamics(FunctionSignature sig,
                        const DiscreteDynamics<KP> *model, KP *z2,
                        const KP &z1) {
  if (sig == FunctionSignature::Inplace) {
    discrete_dynamics(model, z2->state(), z1);
  } else {
    z2->setstate(discrete_dynamics(model, z1));
  }
}

template <typename KP>
void jacobian(const DiscreteDynamics<KP> *model, typename KP::ref_matrix_type J,
              const typename KP::ref_vector_type y,
              const typename KP::state_type &x,
              const typename KP::control_type &u,
              const typename KP::param_type &p) {
  jacobian(model, J, y, x, u, p.first, p.second);
}
template <typename KP, typename P, typename Q>
void jacobian(const DiscreteDynamics<KP> *model, typename KP::jacobian_type &J,
              const Q &y, const typename KP::state_type &x,
              const typename KP::control_type &u, typename KP::base_type t,
              typename KP::base_type dt) {
  model->jacobian(J, y, x, u, t, dt);
}

// TODO: dynamics_error
// TODO: dynamics_error_jacobian

// template <typename KP>
// auto dynamics_error(const DiscreteDynamics<KP> *model, const KP *z2,
// const KP *z1) {
// return discrete_dynamics(model, z1) - state(z1);
//}
// template <typename KP>
// auto dynamics_error(const DiscreteDynamics<KP> *model, V y2, V y1, const KP
// *z2, const KP *z1) {
// discrete_dynamics(model, y2, z1);
// y2 -= state(z2);
//}

// template <typename KP>
// auto dynamics_error_jacobian(FunctionSignature sig, DiffMethod diff,
// const DiscreteDynamics<KP> *model, V J2, V J1,
// V y2, V y1, const KP *z2, const KP *z1) {
// dynamics_error_jacobian(model, J2, J1, y2, y1, z2, z1);
//}
// template <typename KP>
// auto dynamics_error_jacobian(const DiscreteDynamics<KP> *model, V J2, V J1,
// V y2, V y1, const KP *z2, const KP *z1) {
// CHECK(0);
//}

template <typename Ptr>
std::tuple<std::vector<int>, std::vector<int>>
dims(const std::vector<Ptr> &models) {
  std::vector<int> nx, nu;
  if (models.empty()) {
    return std::make_tuple(nx, nu);
  }

  std::for_each(models.begin(), models.end(), [&nx, &nu](const auto &model) {
    nx.push_back(model->state_dim());
    nu.push_back(model->control_dim());
  });
  nx.push_back(nx.back());
  nu.push_back(nu.back());
  for (auto i = 0; i < models.size(); ++i) {
    const auto ny = models.at(i)->output_dim();
    const auto nx_next = nx[i + 1];
    if (nx_next != ny) {
      CHECK(0);
    }
  }
  return std::make_tuple(nx, nu);
}

#endif
