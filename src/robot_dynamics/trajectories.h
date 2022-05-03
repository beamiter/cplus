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

template <int n, int m, typename V, typename T,
          template <int, int, typename, typename> class KP>
struct SampledTrajectory : AbstractTrajectory {
  std::vector<KP<n, m, V, T>> data;
  std::vector<T> times;
};

ABSTRACT_KNOT_POINT_TEMPLATE
struct SampledTrajectoryHelper {
  static auto init(std::vector<ABSTRACT_KNOT_POINT> Z) {
    SampledTrajectory<Nx, Nu, V, T, AbstractKnotPoint> traj;
    traj.times.resize(length(Z));
    for (auto k = 0; k < length(Z); ++Z) {
      traj.times[k] = time(Z[k]);
    }
    traj.data = Z;
    return traj;
  }
};

#endif
