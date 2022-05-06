#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>

#include "discrete_dynamics.h"
#include "knotpoint.h"

using Eigen::all;
using Eigen::MatrixX;

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

struct TrivialParam {
  double t0 = std::numeric_limits<double>::quiet_NaN();
  double tf = std::numeric_limits<double>::quiet_NaN();
  double dt = std::numeric_limits<double>::quiet_NaN();
  int N = std::numeric_limits<int>::quiet_NaN();
};

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

inline auto gettimeinfo(TrivialParam param) {
  return gettimeinfo(param.t0, param.tf, param.dt, param.N);
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

#define SampledTrajectoryTemplate                                              \
  template <int n, int m, typename V, typename T,                              \
            template <int, int, typename, typename> class KP>
#define SampledTrajectoryDeclare                                               \
  SampledTrajectory<n, m, V, T, AbstractKnotPoint>

template <int n, int m, typename V, typename T,
          template <int, int, typename, typename> class KP>
struct SampledTrajectory : AbstractTrajectory {
  typedef V vectype;
  SampledTrajectory() {}
  SampledTrajectory(std::vector<KP<n, m, V, T>> data_in,
                    std::vector<T> times_in)
      : data(std::move(data_in)), times(std::move(times_in)) {}

  auto begin() { return data.begin(); }
  auto end() { return data.end(); }
  auto front() { return data.front(); }
  auto back() { return data.back(); }
  auto size() { return data.size(); }
  const auto &operator[](int i) const { return data[i]; }
  KP<n, m, V, T> &operator[](int i) { return data[i]; }
  std::vector<KP<n, m, V, T>> data;
  std::vector<T> times;
};

template <int n, int m, typename T>
using SampledTrajectoryX = SampledTrajectory<n, m, VectorX<T>, T, KnotPoint>;

template <int n, int m>
using SampledTrajectoryXd =
    SampledTrajectory<n, m, VectorX<double>, double, KnotPoint>;

struct SampledTrajectoryHelper {
  KnotPointTemplate static auto init(std::vector<KnotPointX<Nx, Nu, T>> Z) {
    SampledTrajectoryX<Nx, Nu, T> traj;
    traj.times.resize(length(Z));
    for (auto k = 0; k < length(Z); ++k) {
      traj.times[k] = time(Z[k]);
    }
    traj.data = Z;
    return traj;
  }

  template <int Nx, int Nu>
  static auto init(int n, int m, TrivialParam param = TrivialParam(),
                   bool equal = false) {
    auto times = gettimeinfo(param);
    std::vector<double> dt_vec;
    std::adjacent_difference(times.begin(), times.end(), dt_vec.begin());
    dt_vec.push_back(0.0);
    std::vector<KnotPointXd<Nx, Nu>> Z;
    for (int k = 0; k < Z.size(); ++k) {
      Z.emplace_back(n, m, VectorXd::Zero(n + m), times[k], dt_vec[k]);
    }
    return init<Nx, Nu, double>(Z);
  }

  template <int Nx, int Nu>
  static auto init(const std::vector<VectorXd> &X,
                   const std::vector<VectorXd> &U,
                   TrivialParam param = TrivialParam()) {
    auto N = X.size();
    if (std::isnan(param.tf) && std::isnan(param.dt)) {
      assert(0);
    }
    if (!std::isnan(param.N) && param.N != N) {
      assert(0);
    }
    auto times = gettimeinfo(param);
    std::vector<double> dt_vec;
    std::adjacent_difference(times.begin(), times.end(), dt_vec.begin());
    std::vector<KnotPointXd<Nx, Nu>> Z;
    for (int k = 0; k < N - 1; ++k) {
      VectorXd joined;
      joined << X[k], U[k];
      Z.emplace_back(length(X[k]), length(U[k]), joined, times[k], dt_vec[k]);
    }
    if (length(X) == length(U)) {
      VectorXd joined;
      joined << X.back(), U.back();
      Z.emplace_back(length(X.back()), length(U.back()), joined, times.back(),
                     std::numeric_limits<double>::infinity());
    } else {
      VectorXd joined;
      joined << X.back(), U.back() * 0.0;
      Z.emplace_back(length(X.back()), length(U.back()), joined, times.back(),
                     0.0);
    }
    return init<Nx, Nu, double>(Z);
  }
};

SampledTrajectoryTemplate auto getk(SampledTrajectoryDeclare Z, double t) {
  auto iter = std::lower_bound(Z.times.begin(), Z.times.end(), t);
  return std::distance(iter, Z.times.begin());
}

SampledTrajectoryTemplate auto getstate(SampledTrajectoryDeclare Z, double t) {
  return state(Z.data[getk(Z, t)]);
}

SampledTrajectoryTemplate auto getcontrol(SampledTrajectoryDeclare Z,
                                          double t) {
  return control(Z.data[getk(Z, t)]);
}

SampledTrajectoryTemplate auto getinitialtime(SampledTrajectoryDeclare Z) {
  return time(Z.data.front());
}

SampledTrajectoryTemplate auto getfinaltime(SampledTrajectoryDeclare Z) {
  return time(Z.data.back());
}

SampledTrajectoryTemplate auto
has_terminal_control(SampledTrajectoryDeclare Z) {
  return !is_terminal(Z.back());
}
SampledTrajectoryTemplate auto state_dim(SampledTrajectoryDeclare Z) {
  return n;
}
SampledTrajectoryTemplate auto control_dim(SampledTrajectoryDeclare Z) {
  return m;
}
SampledTrajectoryTemplate auto state_dim(SampledTrajectoryDeclare Z, int k) {
  return state_dim(Z[k]);
}
SampledTrajectoryTemplate auto control_dim(SampledTrajectoryDeclare Z, int k) {
  return control_dim(Z[k]);
}

SampledTrajectoryTemplate auto dims(SampledTrajectoryDeclare Z) {
  std::vector<int> nx;
  std::vector<int> nu;
  for (const auto z : Z) {
    nx.push_back(state_dim(z));
    nu.push_back(control_dim(z));
  }
  return std::make_tuple(nx, nu, length(Z));
}
SampledTrajectoryTemplate auto num_vars(SampledTrajectoryDeclare Z) {
  int num = 0;
  for (const auto z : Z) {
    num += state_dim(z) + (is_terminal(z) ? 0 : control_dim(z));
  }
  return num;
}

inline auto num_vars(int n, int m, int N, bool equal = false) {
  auto Nu = equal ? N : N - 1;
  return N * n + Nu * m;
}

SampledTrajectoryTemplate auto eachcontrol(SampledTrajectoryDeclare Z) {
  std::vector<int> rtn;
  has_terminal_control(Z) ? rtn.resize(length(Z) - 1)
                          : rtn.resize(length(Z) - 2);
  std::iota(rtn.begin(), rtn.end(), 0);
  return rtn;
}

SampledTrajectoryTemplate auto states(SampledTrajectoryDeclare Z) {
  std::vector<V> rtn;
  for (int k = 0; k < Z.size(); ++k) {
    rtn.push_back(state(Z[k]));
  }
  return rtn;
}

SampledTrajectoryTemplate auto states(SampledTrajectoryDeclare Z, int ind) {
  std::vector<T> rtn;
  for (int k = 0; k < Z.size(); ++k) {
    rtn.push_back(state(Z[k])(ind));
  }
  return rtn;
}

SampledTrajectoryTemplate auto states(SampledTrajectoryDeclare Z,
                                      std::vector<int> inds) {
  std::vector<std::vector<T>> rtn;
  for (auto k = 0; k < inds.size(); ++k) {
    rtn.push_back(states(Z, k));
  }
  return rtn;
}

SampledTrajectoryTemplate auto controls(SampledTrajectoryDeclare Z) {
  std::vector<V> rtn;
  for (int k = 0; k < eachcontrol(Z).size(); ++k) {
    rtn.push_back(control(Z[k]));
  }
  return rtn;
}

SampledTrajectoryTemplate auto controls(SampledTrajectoryDeclare Z, int ind) {
  std::vector<T> rtn;
  for (int k = 0; k < eachcontrol(Z).size(); ++k) {
    rtn.push_back(control(Z[k](ind)));
  }
  return rtn;
}

SampledTrajectoryTemplate auto controls(SampledTrajectoryDeclare Z,
                                        std::vector<int> inds) {
  std::vector<std::vector<T>> rtn;
  for (auto k = 0; k < inds.size(); ++k) {
    rtn.push_back(controls(Z, k));
  }
  return rtn;
}

SampledTrajectoryTemplate auto gettimes(SampledTrajectoryDeclare Z) {
  std::vector<T> rtn;
  for (const auto z : Z) {
    rtn.push_back(time(z));
  }
  return rtn;
}

SampledTrajectoryTemplate auto getdata(SampledTrajectoryDeclare Z) {
  std::vector<V> rtn;
  for (const auto z : Z) {
    rtn.push_back(get_data(z));
  }
  return rtn;
}

template <int n, int m, typename V, typename T, typename Q,
          template <int, int, typename, typename> class KP>
auto setstates(SampledTrajectoryDeclare Z, Q X) {
  for (int k = 0; k < Z.size(); ++k) {
    setstate(Z[k], X[k]);
  }
}

SampledTrajectoryTemplate auto setstates(SampledTrajectoryDeclare Z,
                                         MatrixX<T> X) {
  for (int k = 0; k < Z.size(); ++k) {
    setstate(Z[k], X(all, k));
  }
}

template <int n, int m, typename V, typename T, typename Q,
          template <int, int, typename, typename> class KP>
auto setcontrols(SampledTrajectoryDeclare Z, Q U) {
  for (int k = 0; k < Z.size() - 1; ++k) {
    setcontrol(Z[k], U[k]);
  }
}

SampledTrajectoryTemplate auto setcontrols(SampledTrajectoryDeclare Z,
                                           MatrixX<T> U) {
  for (int k = 0; k < Z.size() - 1; ++k) {
    setcontrol(Z[k], U(all, k));
  }
}

SampledTrajectoryTemplate auto setcontrols(SampledTrajectoryDeclare Z,
                                           VectorX<T> u) {
  for (int k = 0; k < Z.size() - 1; ++k) {
    setcontrol(Z[k], u);
  }
}

template <int n, int m, typename V, typename T, typename Q,
          template <int, int, typename, typename> class KP>
auto setdata(SampledTrajectoryDeclare Z, Q data) {
  for (int k = 0; k < Z.size() - 1; ++k) {
    setdata(Z[k], data[k]);
  }
}

SampledTrajectoryTemplate auto setdata(SampledTrajectoryDeclare Z,
                                       MatrixX<T> data) {
  for (int k = 0; k < Z.size() - 1; ++k) {
    setdata(Z[k], data(all, k));
  }
}

SampledTrajectoryTemplate auto settimes(SampledTrajectoryDeclare Z,
                                        std::vector<double> ts) {
  for (auto k = 0; k < ts.size(); ++k) {
    Z[k].t = ts[k];
    k < ts.size() - 1 && (Z[k].dt = ts[k + 1] - ts[k]);
  }
}

SampledTrajectoryTemplate auto set_dt(SampledTrajectoryDeclare Z, double dt) {
  double t = Z[0].t;
  for (auto &z : Z) {
    z.t = t;
    if (!is_terminal(z)) {
      z.dt = dt;
      t += dt;
    }
  }
  return t;
}

SampledTrajectoryTemplate auto setinitialtime(SampledTrajectoryDeclare Z,
                                              double t0) {
  double t0_prev = time(Z[0]);
  double Dt = t0 - t0_prev;
  for (auto &z : Z) {
    double t = time(z);
    settime(z, t + Dt);
  }
  return Z;
}

SampledTrajectoryTemplate auto rollout(FunctionSignature sig,
                                       DiscreteDynamics model,
                                       SampledTrajectoryDeclare Z, V x0) {
  setstate(Z[0], x0);
  for (auto k = 1; k < length(Z); ++k) {
    propagate_dynamics(sig, model, Z[k], Z[k - 1]);
  }
}

#endif
