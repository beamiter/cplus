#ifndef DISCRETE_DYNAMICS_H
#define DISCRETE_DYNAMICS_H

#include "dynamics.h"
#include "robot_dynamics/knotpoint.h"

class DiscreteDynamics : public AbstractModel {
public:
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
AbstractKnotPointTemplate typename AbstractKnotPointDeclare::state_type
evaluate(const DiscreteDynamics *model, const AbstractKnotPointDeclare &z) {
  return discrete_dynamics(model, z);
}
// Evaluate for inplace.
AbstractKnotPointTemplate void
evaluate(const DiscreteDynamics *model,
         typename AbstractKnotPointDeclare::state_type *xn,
         const AbstractKnotPointDeclare &z) {
  discrete_dynamics(model, xn, z);
}

// This method is called when using the 'StaticReturn'
AbstractKnotPointTemplate typename AbstractKnotPointDeclare::state_type
discrete_dynamics(const DiscreteDynamics *model,
                  const AbstractKnotPointDeclare &z) {
  return discrete_dynamics<Nx, Nu, V, T>(model, z.state(), z.control(),
                                         z.time(), z.timestep());
}

AbstractKnotPointTemplate typename AbstractKnotPointDeclare::state_type
discrete_dynamics(const DiscreteDynamics *model,
                  const typename AbstractKnotPointDeclare::state_type &x,
                  const typename AbstractKnotPointDeclare::control_type &u,
                  double t, double dt) {
  CHECK(0);
}

// This method is called when using the 'InPlace'
AbstractKnotPointTemplate void
discrete_dynamics(const DiscreteDynamics *model,
                  typename AbstractKnotPointDeclare::state_type *xn,
                  const AbstractKnotPointDeclare &z) {
  discrete_dynamics<Nx, Nu, V, T>(model, xn, z.state(), z.control(), z.time(),
                                  z.timestep());
}
AbstractKnotPointTemplate void
discrete_dynamics(const DiscreteDynamics *model,
                  typename AbstractKnotPointDeclare::state_type *xn,
                  const typename AbstractKnotPointDeclare::state_type &x,
                  const typename AbstractKnotPointDeclare::control_type &u, T t,
                  T dt) {
  assert(0);
}

AbstractKnotPointTemplate void
discrete_dynamics(FunctionSignature sig, const DiscreteDynamics *model,
                  typename AbstractKnotPointDeclare::state_type *xn,
                  const AbstractKnotPointDeclare &z) {
  if (sig == FunctionSignature::Inplace) {
    discrete_dynamics(model, xn, z);
  } else {
    xn = discrete_dynamics(model, z);
  }
}

// Function not support partial specialization yet
AbstractKnotPointTemplate void
propagate_dynamics(FunctionSignature sig, const DiscreteDynamics *model,
                   AbstractKnotPointDeclare *z2,
                   const AbstractKnotPointDeclare &z1) {
  if (sig == FunctionSignature::Inplace) {
    discrete_dynamics(model, z2->state(), z1);
  } else {
    z2->setstate(discrete_dynamics(model, z1));
  }
}

// TODO: jacobian
// TODO: dynamics_error
// TODO: dynamics_error_jacobian

template <typename V, typename T, typename P>
void jacobian(const DiscreteDynamics *model, V J, V y, V x, V u, P p) {
  jacobian(model, J, y, x, u, p.t, p.dt);
}
template <typename V, typename T, typename P>
void jacobian(const DiscreteDynamics *model, V J, V y, V x, V u, T t, T dt) {
  throw std::runtime_error(
      "user-defined discrete dynamics jacobian not defined.");
  assert(0);
}

AbstractKnotPointTemplate auto
dynamics_error(const DiscreteDynamics *model,
               const AbstractKnotPointDeclare *z2,
               const AbstractKnotPointDeclare *z1) {
  return discrete_dynamics(model, z1) - state(z1);
}
AbstractKnotPointTemplate auto
dynamics_error(const DiscreteDynamics *model, V y2, V y1,
               const AbstractKnotPointDeclare *z2,
               const AbstractKnotPointDeclare *z1) {
  discrete_dynamics(model, y2, z1);
  y2 -= state(z2);
}

AbstractKnotPointTemplate auto
dynamics_error_jacobian(FunctionSignature sig, DiffMethod diff,
                        const DiscreteDynamics *model, V J2, V J1, V y2, V y1,
                        const AbstractKnotPointDeclare *z2,
                        const AbstractKnotPointDeclare *z1) {
  dynamics_error_jacobian(model, J2, J1, y2, y1, z2, z1);
}
AbstractKnotPointTemplate auto
dynamics_error_jacobian(const DiscreteDynamics *model, V J2, V J1, V y2, V y1,
                        const AbstractKnotPointDeclare *z2,
                        const AbstractKnotPointDeclare *z1) {
  throw std::runtime_error("user-defined discrete error jacobian not defined.");
  assert(0);
}

template <typename Ptr> inline auto dims(const std::vector<Ptr> &models) {
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
      throw std::runtime_error("Model mismatch at time step");
    }
  }
  return std::make_tuple(nx, nu);
}

#endif
