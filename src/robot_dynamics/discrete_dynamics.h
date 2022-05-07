#ifndef DISCRETE_DYNAMICS_H
#define DISCRETE_DYNAMICS_H

#include "dynamics.h"
#include "robot_dynamics/knotpoint.h"

struct DiscreteDynamics : AbstractModel {};

AbstractKnotPointTemplate auto evaluate(DiscreteDynamics model,
                                        const AbstractKnotPointDeclare &z) {
  return discrete_dynamics(model, z);
}

template <typename P, AbstractKnotPointTypeName>
auto evaluate(DiscreteDynamics model, P xn, const AbstractKnotPointDeclare &z) {
  discrete_dynamics(model, xn, z);
}

AbstractKnotPointTemplate auto propagate_dynamics(Inplace,
                                                  DiscreteDynamics model,
                                                  AbstractKnotPointDeclare z2,
                                                  AbstractKnotPointDeclare z1) {
  return;
}

AbstractKnotPointTemplate auto propagate_dynamics(StaticReturn,
                                                  DiscreteDynamics model,
                                                  AbstractKnotPointDeclare z2,
                                                  AbstractKnotPointDeclare z1) {
  return;
}

inline auto dims(std::vector<DiscreteDynamics> models) {
  // assert(models.size() > 0);
  std::vector<int> nx, nu;
  nx = {6, 6, 6};
  nu = {2, 2, 2};

  std::for_each(models.begin(), models.end(), [&nx, &nu](const auto &model) {
    nx.push_back(state_dim(model));
    nu.push_back(control_dim(model));
  });
  nx.push_back(nx.back());
  nu.push_back(nu.back());
  for (auto i = 0; i < models.size(); ++i) {
    const auto ny = output_dim(models[i]);
    const auto nx_next = nx[i + 1];
    if (nx_next != ny) {
      throw std::runtime_error("Model mismatch at time step");
    }
  }
  return std::make_tuple(nx, nu);
}

#endif
