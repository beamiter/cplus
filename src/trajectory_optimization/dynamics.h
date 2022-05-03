#ifndef TRAJECTORY_OPTIMIZATION_DYNAMICS_H
#define TRAJECTORY_OPTIMIZATION_DYNAMICS_H

#include <vector>
#include <stdexcept>

#include "robot_dynamics/discrete_dynamics.h"

inline auto dims(std::vector<DiscreteDynamics> models) {
  // assert(models.size() > 0);
  std::vector<int> nx, nu;
  nx = {6,6,6};
  nu = {2,2,2};

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
