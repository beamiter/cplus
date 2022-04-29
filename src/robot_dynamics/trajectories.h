#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include <stdexcept>
#include <vector>

#include "knotpoint.h"

struct AbstractTrajectory {};

inline auto getstate(AbstractTrajectory Z, double t) {
  throw std::runtime_error("Not implemented");
}
inline auto getcontrol(AbstractTrajectory Z, double t) {
  throw std::runtime_error("Not implemented");
}
inline auto getinitialtime(AbstractTrajectory Z, double t) {
  throw std::runtime_error("Not implemented");
}
inline auto getfinaltime(AbstractTrajectory Z, double t) {
  throw std::runtime_error("Not implemented");
}

template <int n, int m, typename T, typename KP>
struct SampledTrajectory : AbstractTrajectory {
  std::vector<KP> data;
  std::vector<T> times;
};

ABSTRACT_KNOT_POINT_TEMPLATE
struct SampledTrajectoryHelper {
  static auto init(std::vector<ABSTRACT_KNOT_POINT> Z) {
    SampledTrajectory<Nx, Nu, T, ABSTRACT_KNOT_POINT> traj;
    traj.times.resize(length(Z));
    for (auto k = 0; k < length(Z); ++Z) {
      traj.times[k] = time(Z[k]);
    }
    traj.data = Z;
    return traj;
  }
};

#endif
