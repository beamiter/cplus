#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include <cmath>
#include <numeric>
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

inline auto gettimeinfo(double t0 = 0.,
                        double tf = std::numeric_limits<double>::quiet_NaN(),
                        double dt = std::numeric_limits<double>::quiet_NaN(),
                        int N = 0) {
  auto Dt = zero(t0);
  if (std::isnan(tf) + std::isnan(dt) + (N == 0) > 1) {
    throw std::runtime_error("must specify at least two of the following");
  }
  if (N == 0 && !std::isnan(tf) && !std::isnan(dt)) {
    N = std::round(tf / dt + 1);
    Dt = tf - t0;
    dt = Dt / (N - 1);
  }
  if (!std::isnan(tf) && std::isnan(dt)) {
    dt = tf / (N - 1);
  }
  if (!std::isnan(dt) && std::isnan(tf)) {
    tf = dt * (N - 1);
  }
  Dt = tf - t0;
  std::vector<double> rtn;
  for (int i = 0; i < N; ++i) {
    rtn.push_back(t0);
    t0 += dt;
  }
  return rtn;
}

inline auto gettimeinfo(std::vector<double> dt, double t0 = 0.,
                        double tf = std::numeric_limits<double>::quiet_NaN(),
                        int N = 0) {
  assert(
      std::all_of(dt.begin(), dt.end(), [](double t) { return t > zero(t); }));
  if (N == 0) {
    N = dt.size() + 1;
  }
  if (std::isnan(tf)) {
    tf = std::accumulate(dt.begin(), dt.end(), t0);
  }
  assert(dt.size() == N - 1);
  dt.insert(dt.begin(), t0);
  return std::partial_sum(dt.begin(), dt.end(), dt.begin());
}

#define SAMPLED_TRAJECTORY_TEMPLATE                                            \
  template <int n, int m, typename V, typename T,                              \
            template <int, int, typename, typename> class KP>
#define SAMPLED_TRAJECTORY SampledTrajectory<n, m, V, T, AbstractKnotPoint>

SAMPLED_TRAJECTORY_TEMPLATE
struct SampledTrajectory : AbstractTrajectory {
#define KP_TYPENAME KP<n, m, V, T>
  SampledTrajectory() {}
  SampledTrajectory(std::vector<KP_TYPENAME> data_in, std::vector<T> times_in)
      : data(std::move(data_in)), times(std::move(times_in)) {}

  auto begin() { return data.begin(); }
  auto end() { return data.end(); }
  auto front() { return data.front(); }
  auto back() { return data.back(); }
  auto size() { return data.size(); }
  const auto &operator[](int i) const { return data[i]; }
  KP_TYPENAME &operator[](int i) { return data[i]; }
  std::vector<KP_TYPENAME> data;
  std::vector<T> times;
};

struct SampledTrajectoryHelper {
  template <int Nx, int Nu, typename V, typename T>
  static auto init(std::vector<AbstractKnotPoint<Nx, Nu, V, T>> Z) {
    SampledTrajectory<Nx, Nu, V, T, AbstractKnotPoint> traj;
    traj.times.resize(length(Z));
    for (auto k = 0; k < length(Z); ++Z) {
      traj.times[k] = time(Z[k]);
    }
    traj.data = Z;
    return traj;
  }

  template <int Nx, int Nu>
  static auto init(int n, int m, double t0, double tf, double dt, double N,
                   bool equal = false) {
    auto times = gettimeinfo(t0, tf, dt, N);
    std::adjacent_difference(times.begin(), times.end(), times.begin());
    times.push_back(0.0);
    KnotPoint<Nx, Nu, VectorX<double>, double> Z;
    KnotPointX<Nx, Nu, double> kp;
    KnotPointXd<Nx, Nu> kp0;
  }
};

SAMPLED_TRAJECTORY_TEMPLATE
auto getk(SAMPLED_TRAJECTORY Z, double t) {
  auto iter = std::lower_bound(Z.times.begin(), Z.times.end(), t);
  return std::distance(iter, Z.times.begin());
}

SAMPLED_TRAJECTORY_TEMPLATE
auto getstate(SAMPLED_TRAJECTORY Z, double t) {
  return state(Z.data[getk(Z, t)]);
}
SAMPLED_TRAJECTORY_TEMPLATE
auto getcontrol(SAMPLED_TRAJECTORY Z, double t) {
  return control(Z.data[getk(Z, t)]);
}
SAMPLED_TRAJECTORY_TEMPLATE
auto getinitialtime(SAMPLED_TRAJECTORY Z) { return time(Z.data.front()); }
SAMPLED_TRAJECTORY_TEMPLATE
auto getfinaltime(SAMPLED_TRAJECTORY Z) { return time(Z.data.back()); }

#endif
