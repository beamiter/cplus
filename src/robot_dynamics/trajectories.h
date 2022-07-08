#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>

#include "discretized_dynamics.h"
#include "knotpoint.h"
#include "robot_dynamics/integration.h"

using Eigen::all;
using Eigen::MatrixX;

struct TrivialParam {
  TrivialParam() {
    N = 0;
    t0 = 0.0;
    tf = std::numeric_limits<double>::quiet_NaN();
    dt = std::numeric_limits<double>::quiet_NaN();
  }
  int N = 0;
  double t0 = 0.0;
  double tf = std::numeric_limits<double>::quiet_NaN();
  double dt = std::numeric_limits<double>::quiet_NaN();
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

template <typename KP> class AbstractTrajectory {
public:
  virtual ~AbstractTrajectory() = default;
  virtual const typename KP::state_type getstate(double t) const = 0;
  virtual const typename KP::control_type getcontrol(double t) const = 0;
  virtual typename KP::base_type getinitialtime() const = 0;
  virtual typename KP::base_type getfinaltime() const = 0;
};

template <typename KP> class SampledTrajectory : public AbstractTrajectory<KP> {
public:
  using state_type = typename KP::state_type;
  using control_type = typename KP::control_type;
  using value_type = typename KP::value_type;
  using base_type = typename KP::base_type;
  using vectype = value_type;

  // Constructors.
  SampledTrajectory() = default;
  SampledTrajectory(std::vector<KP> data_in, std::vector<base_type> times_in)
      : data(std::move(data_in)), times(std::move(times_in)) {}
  SampledTrajectory(const SampledTrajectory<KP> &traj)
      : data(traj.data), times(traj.times) {}
  SampledTrajectory(const std::vector<KP> &Z) {
    times.resize(length(Z));
    for (auto k = 0; k < length(Z); ++k) {
      times[k] = Z[k].time();
    }
    data = Z;
  }
  SampledTrajectory(TrivialParam param, bool equal = false) {
    times = gettimeinfo(param);
    std::vector<double> dt_vec;
    dt_vec.reserve(times.size() + 1);
    std::adjacent_difference(times.begin(), times.end(), dt_vec.begin());
    // Since the first value is not adjacent_difference.
    dt_vec[0] = dt_vec[1];
    dt_vec.push_back(0.0);
    data.clear();
    for (int k = 0; k < times.size(); ++k) {
      data.emplace_back(Vector<typename KP::base_type, KP::N + KP::M>::Zero(),
                        times[k], dt_vec[k]);
    }
  }
  SampledTrajectory(const std::vector<state_type> &X,
                    const std::vector<control_type> &U, TrivialParam param) {
    auto N = X.size();
    if (std::isnan(param.tf) && std::isnan(param.dt)) {
      CHECK(0);
    }
    if (!std::isnan(param.N) && param.N != N) {
      CHECK(0);
    }
    times = gettimeinfo(param);
    std::vector<double> dt_vec;
    dt_vec.reserve(times.size());
    std::adjacent_difference(times.begin(), times.end(), dt_vec.begin());
    // Since the first value is not adjacent_difference.
    dt_vec[0] = dt_vec[1];
    data.clear();
    for (int k = 0; k < N - 1; ++k) {
      VectorXd joined(X[k].size() + U[k].size());
      // LOG(INFO) << X[k] << ", " << U[k] << ", " << dt_vec[k];
      joined << X[k], U[k];
      // LOG(INFO) << joined;
      data.emplace_back(joined, times[k], dt_vec[k]);
    }
    if (length(X) == length(U)) {
      VectorXd joined(X.back().size() + U.back().size());
      joined << X.back(), U.back();
      data.emplace_back(joined, times.back(),
                        std::numeric_limits<double>::infinity());
    } else {
      VectorXd joined(X.back().size() + U.back().size());
      joined << X.back(), U.back() * 0.0;
      data.emplace_back(joined, times.back(), 0.0);
    }
  }

  // Overrides.
  const state_type getstate(double t) const final {
    CHECK(!data.empty());
    return data[getk(t)].state();
  }
  const control_type getcontrol(double t) const final {
    CHECK(!data.empty());
    return data[getk(t)].control();
  }
  base_type getinitialtime() const final {
    CHECK(!data.empty());
    return data.front().time();
  }
  base_type getfinaltime() const final {
    CHECK(!data.empty());
    return data.back().time();
  }

  // Operators.
  SampledTrajectory<KP> &operator=(const SampledTrajectory<KP> &traj) {
    this->data = traj.data;
    this->times = traj.times;
    return *this;
  }

  // Iterators.
  auto begin() { return data.begin(); }
  auto end() { return data.end(); }
  auto front() { return data.front(); }
  auto back() { return data.back(); }
  int size() const { return data.size(); }
  const KP &operator[](int i) const { return data[i]; }
  KP &operator[](int i) { return data[i]; }

  // Methods.

  // Members.
  std::vector<KP> data;
  std::vector<base_type> times;

protected:
  int getk(double t) const {
    auto iter = std::lower_bound(times.begin(), times.end(), t);
    if (iter == times.end()) {
      return times.size() - 1;
    }
    return std::distance(iter, times.begin());
  }
};
template <int n, int m, typename T>
using SampledTrajectoryS = SampledTrajectory<KnotPointS<n, m, T>>;
template <int n, int m>
using SampledTrajectorySd = SampledTrajectory<KnotPointSd<n, m>>;

template <typename KP> auto has_terminal_control(SampledTrajectory<KP> Z) {
  return !is_terminal(Z.back());
}
template <typename KP> auto state_dim(SampledTrajectory<KP> Z) {
  return KP::Nx;
}
template <typename KP> auto control_dim(SampledTrajectory<KP> Z) {
  return KP::Nu;
}
template <typename KP> auto state_dim(SampledTrajectory<KP> Z, int k) {
  return state_dim(Z[k]);
}
template <typename KP> auto control_dim(SampledTrajectory<KP> Z, int k) {
  return control_dim(Z[k]);
}

template <typename KP>
std::tuple<std::vector<int>, std::vector<int>, int>
dims(SampledTrajectory<KP> Z) {
  std::vector<int> nx;
  std::vector<int> nu;
  for (const auto z : Z) {
    nx.push_back(state_dim(z));
    nu.push_back(control_dim(z));
  }
  return std::make_tuple(nx, nu, length(Z));
}
template <typename KP> auto num_vars(SampledTrajectory<KP> Z) {
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

template <typename KP> auto eachcontrol(SampledTrajectory<KP> Z) {
  std::vector<int> rtn;
  has_terminal_control(Z) ? rtn.resize(length(Z) - 1)
                          : rtn.resize(length(Z) - 2);
  std::iota(rtn.begin(), rtn.end(), 0);
  return rtn;
}

template <typename KP> auto states(SampledTrajectory<KP> Z) {
  std::vector<typename KP::state_type> rtn;
  for (int k = 0; k < Z.size(); ++k) {
    rtn.push_back(state(Z[k]));
  }
  return rtn;
}

template <typename KP> auto states(SampledTrajectory<KP> Z, int ind) {
  std::vector<typename KP::base_type> rtn;
  for (int k = 0; k < Z.size(); ++k) {
    rtn.push_back(state(Z[k])(ind));
  }
  return rtn;
}

template <typename KP>
auto states(SampledTrajectory<KP> Z, std::vector<int> inds) {
  std::vector<std::vector<typename KP::state_type>> rtn;
  for (auto k = 0; k < inds.size(); ++k) {
    rtn.push_back(states(Z, k));
  }
  return rtn;
}

template <typename KP> auto controls(SampledTrajectory<KP> Z) {
  std::vector<typename KP::control_type> rtn;
  for (int k = 0; k < eachcontrol(Z).size(); ++k) {
    rtn.push_back(control(Z[k]));
  }
  return rtn;
}

template <typename KP> auto controls(SampledTrajectory<KP> Z, int ind) {
  std::vector<typename KP::base_type> rtn;
  for (int k = 0; k < eachcontrol(Z).size(); ++k) {
    rtn.push_back(control(Z[k](ind)));
  }
  return rtn;
}

template <typename KP>
auto controls(SampledTrajectory<KP> Z, std::vector<int> inds) {
  std::vector<std::vector<typename KP::control_type>> rtn;
  for (auto k = 0; k < inds.size(); ++k) {
    rtn.push_back(controls(Z, k));
  }
  return rtn;
}

template <typename KP>
std::vector<typename KP::base_type> gettimes(SampledTrajectory<KP> Z) {
  std::vector<typename KP::base_type> rtn;
  for (const auto z : Z) {
    rtn.push_back(time(z));
  }
  return rtn;
}

template <typename KP> auto getdata(SampledTrajectory<KP> Z) {
  std::vector<typename KP::value_type> rtn;
  for (const auto z : Z) {
    rtn.push_back(get_data(z));
  }
  return rtn;
}

template <typename Q, typename KP>
auto setstates(SampledTrajectory<KP> Z, Q X) {
  for (int k = 0; k < Z.size(); ++k) {
    Z[k].setstate(X[k]);
  }
}

template <typename KP>
auto setstates(SampledTrajectory<KP> Z, MatrixX<typename KP::base_type> X) {
  for (int k = 0; k < Z.size(); ++k) {
    Z[k].setstate(X(all, k));
  }
}

template <typename Q, typename KP>
auto setcontrols(SampledTrajectory<KP> Z, Q U) {
  for (int k = 0; k < Z.size() - 1; ++k) {
    Z[k].setcontrol(U[k]);
  }
}

template <typename KP>
auto setcontrols(SampledTrajectory<KP> Z, MatrixX<typename KP::base_type> U) {
  for (int k = 0; k < Z.size() - 1; ++k) {
    Z[k].setcontrol(U(all, k));
  }
}

template <typename KP>
auto setcontrols(SampledTrajectory<KP> Z, VectorX<typename KP::base_type> u) {
  for (int k = 0; k < Z.size() - 1; ++k) {
    setcontrol(Z[k], u);
  }
}

template <typename Q, typename KP>
auto setdata(SampledTrajectory<KP> Z, Q data) {
  for (int k = 0; k < Z.size() - 1; ++k) {
    setdata(Z[k], data[k]);
  }
}

template <typename KP>
auto setdata(SampledTrajectory<KP> Z, MatrixX<typename KP::base_type> data) {
  for (int k = 0; k < Z.size() - 1; ++k) {
    setdata(Z[k], data(all, k));
  }
}

template <typename KP>
auto settimes(SampledTrajectory<KP> Z, std::vector<typename KP::base_type> ts) {
  for (auto k = 0; k < ts.size(); ++k) {
    Z[k].t = ts[k];
    k < ts.size() - 1 && (Z[k].dt = ts[k + 1] - ts[k]);
  }
}

template <typename KP>
auto set_dt(SampledTrajectory<KP> Z, typename KP::base_type dt) {
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

template <typename KP>
void setinitialtime(SampledTrajectory<KP> Z, typename KP::base_type t0) {
  double t0_prev = Z[0].time();
  double Dt = t0 - t0_prev;
  for (auto &z : Z) {
    double t = z.time();
    z.settime(t + Dt);
  }
}

template <typename KP> int length(const SampledTrajectory<KP> &z) {
  return length(z.data);
}

template <typename KP>
void rollout(FunctionSignature sig, const DiscreteDynamics<KP> *model,
             SampledTrajectory<KP> &Z, typename KP::state_type x0) {
  Z[0].setstate(x0);
  for (auto k = 1; k < length(Z); ++k) {
    propagate_dynamics(sig, model, &Z[k], Z[k - 1]);
  }
}

// DiscretizedDynamics need to specify the Quadrature rule.
template <template <typename> class Q, typename KP>
void rollout(FunctionSignature sig, const DiscretizedDynamics<KP, Q> *model,
             SampledTrajectory<KP> &Z, typename KP::state_type x0) {
  Z[0].setstate(x0);
  for (auto k = 1; k < length(Z); ++k) {
    propagate_dynamics(sig, model, &Z[k], Z[k - 1]);
  }
}

#endif
