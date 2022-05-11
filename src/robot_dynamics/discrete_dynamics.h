#ifndef DISCRETE_DYNAMICS_H
#define DISCRETE_DYNAMICS_H

#include "dynamics.h"
#include "robot_dynamics/knotpoint.h"

AbstractModelTemplate class DiscreteDynamics : public AbstractModelDeclare {};
#define DiscreteDynamicsDeclare DiscreteDynamics<F, S>

AbstractModelTemplate using DiscreteDynamicsRef =
    std::reference_wrapper<DiscreteDynamicsDeclare>;
AbstractModelTemplate using DiscreteDynamicsConstRef =
    std::reference_wrapper<const DiscreteDynamicsDeclare>;
AbstractModelTemplate using DiscreteDynamicsRefVec =
    std::vector<std::reference_wrapper<DiscreteDynamicsDeclare>>;
AbstractModelTemplate using DiscreteDynamicsConstRefVec =
    std::vector<std::reference_wrapper<const DiscreteDynamicsDeclare>>;

// AbstractKnotPointTemplate auto evaluate(const DiscreteDynamics *model,
// const AbstractKnotPointDeclare &z) {
// return discrete_dynamics(model, z);
//}

template <AbstractKnotPointTypeName, AbstractModelTypeName>
V discrete_dynamics(const DiscreteDynamicsDeclare &model,
                    const AbstractKnotPointDeclare &z) {
  return discrete_dynamics(model, z->state(), z->control(), z->time(),
                           z->timestep());
}
template <AbstractKnotPointTypeName, AbstractModelTypeName>
V discrete_dynamics(const DiscreteDynamicsDeclare &model, V x, V u, double t,
                    double dt) {
  throw std::runtime_error("discrete dynamics not defined yet");
}

// template <typename P, AbstractKnotPointTypeName>
// auto evaluate(const DiscreteDynamics *model, P xn,
// const AbstractKnotPointDeclare &z) {
// discrete_dynamics(model, xn, z);
//}

// Function not support partial specialization
// AbstractKnotPointTemplate auto propagate_dynamics(FunctionSignature::Inplace,
//                                                   DiscreteDynamics model,
//                                                   AbstractKnotPointDeclare
//                                                   z2,
//                                                   AbstractKnotPointDeclare
//                                                   z1) {
//   return;
// }
//
// AbstractKnotPointTemplate auto
// propagate_dynamics(FunctionSignature::StaticReturn,
//                                                   DiscreteDynamics model,
//                                                   AbstractKnotPointDeclare
//                                                   z2,
//                                                   AbstractKnotPointDeclare
//                                                   z1) {
//   return;
// }

AbstractModelTemplate inline auto
dims(std::vector<const DiscreteDynamicsDeclare *> models) {
  std::vector<int> nx, nu;
  if (models.empty()) {
    return std::make_tuple(nx, nu);
  }

  std::for_each(models.begin(), models.end(), [&nx, &nu](const auto model) {
    nx.push_back(model->state_dim());
    nu.push_back(model->control_dim());
  });
  nx.push_back(nx.back());
  nu.push_back(nu.back());
  for (auto i = 0; i < models.size(); ++i) {
    const auto ny = models[i]->output_dim();
    const auto nx_next = nx[i + 1];
    if (nx_next != ny) {
      throw std::runtime_error("Model mismatch at time step");
    }
  }
  return std::make_tuple(nx, nu);
}

#endif
