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

// using DiscreteDynamicsRef = std::reference_wrapper<DiscreteDynamics>;
// using DiscreteDynamicsConstRef = std::reference_wrapper<const
// DiscreteDynamics>; using DiscreteDynamicsRefVec =
// std::vector<std::reference_wrapper<DiscreteDynamics>>;
// using DiscreteDynamicsConstRefVec =
// std::vector<std::reference_wrapper<const DiscreteDynamics>>;

AbstractKnotPointTemplate auto evaluate(const DiscreteDynamics *model,
                                        const AbstractKnotPointDeclare *z) {
  return discrete_dynamics(model, z);
}
template <typename P, AbstractKnotPointTypeName>
auto evaluate(const DiscreteDynamics *model, P *xn,
              const AbstractKnotPointDeclare *z) {
  return discrete_dynamics(model, xn, z);
}

template <AbstractKnotPointTypeName>
V discrete_dynamics(const DiscreteDynamics *model,
                    const AbstractKnotPointDeclare *z) {
  return discrete_dynamics(model, z->state(), z->control(), z->time(),
                           z->timestep());
}
template <AbstractKnotPointTypeName>
V discrete_dynamics(const DiscreteDynamics *model, V x, V u, double t,
                    double dt) {
  throw std::runtime_error("discrete dynamics not defined yet");
}

template <typename P, AbstractKnotPointTypeName>
auto discrete_dynamics(const DiscreteDynamics *model, P *xn,
                       const AbstractKnotPointDeclare *z) {
  discrete_dynamics(model, xn, z->state(), z->control(), z->time(),
                    z->timestep());
}
template <typename P, AbstractKnotPointTypeName>
void discrete_dynamics(const DiscreteDynamics *model, P *xn, V x, V u, double t,
                       double dt) {
  throw std::runtime_error("discrete dynamics not defined yet");
  assert(0);
}
template <typename P, AbstractKnotPointTypeName>
void discrete_dynamics(FunctionSignature sig, const DiscreteDynamics *model,
                       P *xn, const AbstractKnotPointDeclare *z) {
  if (sig == FunctionSignature::Inplace) {
    discrete_dynamics(model, xn, z);
  } else {
    xn = discrete_dynamics(model, z);
  }
}

// Function not support partial specialization
AbstractKnotPointTemplate auto
propagate_dynamics(FunctionSignature sig, const DiscreteDynamics *model,
                   const AbstractKnotPointDeclare *z2,
                   const AbstractKnotPointDeclare *z1) {
  if (sig == FunctionSignature::Inplace) {
    discrete_dynamics(model, z2->state(), z1);
  } else {
    z2->setstate(discrete_dynamics(model, z1));
  }
}

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
