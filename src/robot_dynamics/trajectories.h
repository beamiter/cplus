#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include <cmath>
#include <iostream>
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
    if (X.size() == U.size()) {
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
  typename std::vector<KP>::iterator begin() { return data.begin(); }
  typename std::vector<KP>::iterator end() { return data.end(); }
  typename std::vector<KP>::const_iterator begin() const {
    return data.begin();
  }
  typename std::vector<KP>::const_iterator end() const { return data.end(); }
  KP front() const { return data.front(); }
  KP back() const { return data.back(); }
  KP &front() { return data.front(); }
  KP &back() { return data.back(); }
  const KP &operator[](int i) const { return data[i]; }
  KP &operator[](int i) { return data[i]; }
  KP &at(int i) {
    CHECK(i >= 0 && i < data.size());
    return (*this)[i];
  }
  int size() const { return data.size(); }
  int length() { return data.size(); }

  // Methods.
  bool has_terminal_control(SampledTrajectory<KP> Z) {
    return !is_terminal(Z.back());
  }
  int state_dim() const { return KP::Nx; }
  int control_dim() const { return KP::Nu; }
  int state_dim(int k) const { return data[k].state_dim(); }
  int control_dim(int k) const { return data[k].control_dim(); }
  std::tuple<std::vector<int>, std::vector<int>, int> dims() const {
    std::vector<int> nx;
    std::vector<int> nu;
    for (const auto &z : data) {
      nx.push_back(z.state_dim());
      nu.push_back(z.control_dim());
    }
    return std::make_tuple(nx, nu, length(data));
  }
  int num_vars() const {
    int num = 0;
    for (const auto &z : data) {
      num += z.state_dim() + (is_terminal(z) ? 0 : z.control_dim());
    }
    return num;
  }
  std::vector<int> eachcontrol() const {
    std::vector<int> rtn;
    has_terminal_control() ? rtn.resize(length(data))
                           : rtn.resize(length(data) - 1);
    std::iota(rtn.begin(), rtn.end(), 0);
    return rtn;
  }
  std::vector<typename KP::state_type> states() const {
    std::vector<typename KP::state_type> rtn;
    for (int k = 0; k < data.size(); ++k) {
      rtn.push_back(data[k].state());
    }
    return rtn;
  }
  std::vector<typename KP::base_type> states(int ind) const {
    std::vector<typename KP::base_type> rtn;
    for (int k = 0; k < data.size(); ++k) {
      rtn.push_back(data[k].state()[ind]);
    }
    return rtn;
  }
  std::vector<std::vector<typename KP::base_type>>
  states(const std::vector<int> &inds) const {
    std::vector<std::vector<typename KP::base_type>> rtn;
    for (auto k = 0; k < inds.size(); ++k) {
      rtn.push_back(states(k));
    }
    return rtn;
  }
  std::vector<typename KP::control_type> controls() const {
    std::vector<typename KP::control_type> rtn;
    for (int k = 0; k < eachcontrol().size(); ++k) {
      rtn.push_back(data[k].control());
    }
    return rtn;
  }
  std::vector<typename KP::base_type> controls(int ind) const {
    std::vector<typename KP::base_type> rtn;
    for (int k = 0; k < eachcontrol().size(); ++k) {
      rtn.push_back(data[k].control()[ind]);
    }
    return rtn;
  }
  std::vector<std::vector<typename KP::base_type>>
  controls(std::vector<int> inds) const {
    std::vector<std::vector<typename KP::base_type>> rtn;
    for (auto k = 0; k < inds.size(); ++k) {
      rtn.push_back(controls(k));
    }
    return rtn;
  }
  std::vector<typename KP::base_type> gettimes() const {
    std::vector<typename KP::base_type> rtn;
    for (const auto &z : data) {
      rtn.push_back(z.time());
    }
    return rtn;
  }
  std::vector<typename KP::value_type> getdata() const {
    std::vector<typename KP::value_type> rtn;
    for (const auto z : data) {
      rtn.push_back(z.data());
    }
    return rtn;
  }
  void setstates(const std::vector<typename KP::state_type> &X) {
    CHECK(X.size() == data.size());
    for (int k = 0; k < X.size(); ++k) {
      data[k].setstate(X[k]);
    }
  }
  void setstates(const MatrixX<typename KP::base_type> &X) {
    CHECK(X.rows() == KP::N);
    CHECK(X.cols() == data.size());
    for (int k = 0; k < data.size(); ++k) {
      data[k].setstate(X(all, k));
    }
  }
  void setcontrols(const std::vector<typename KP::control_type> &U) {
    for (int k = 0; k < data.size() - 1; ++k) {
      data[k].setcontrol(U[k]);
    }
  }
  void setcontrols(const MatrixX<typename KP::base_type> &U) {
    CHECK(U.rows() == KP::M);
    CHECK(U.cols() == data.size());
    for (int k = 0; k < data.size() - 1; ++k) {
      data[k].setcontrol(U(all, k));
    }
  }
  void setcontrols(const typename KP::control_type &u) {
    for (int k = 0; k < data.size() - 1; ++k) {
      data[k].setcontrol(u);
    }
  }
  void setdata(const std::vector<KP> &data_in) {
    for (int k = 0; k < data.size() - 1; ++k) {
      data[k].setdata(data_in[k]);
    }
  }
  void setdata(const MatrixX<typename KP::base_type> &data_in) {
    CHECK(data_in.rows() == KP::N + KP::M);
    CHECK(data_in.cols() == data.size());
    for (int k = 0; k < data.size() - 1; ++k) {
      data[k].setdata(data_in(all, k));
    }
  }
  void settimes(const std::vector<typename KP::base_type> &ts) {
    for (auto k = 0; k < ts.size(); ++k) {
      data[k].t = ts[k];
      k < ts.size() - 1 && (data[k].dt = ts[k + 1] - ts[k]);
    }
  }
  typename KP::base_type set_dt(typename KP::base_type dt) {
    double t = data[0].t;
    for (auto &z : data) {
      z.t = t;
      if (!is_terminal(z)) {
        z.dt = dt;
        t += dt;
      }
    }
    return t;
  }
  void setinitialtime(typename KP::base_type t0) {
    double t0_prev = data[0].time();
    double Dt = t0 - t0_prev;
    for (auto &z : data) {
      double t = z.time();
      z.settime(t + Dt);
    }
  }
  friend std::ostream &operator<<(std::ostream &output,
                                  const SampledTrajectory<KP> &Z) {
    for (const auto &pt : Z.data) {
      output << "Point : " << pt[0] << "\n";
    }
    return output;
  }

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
  int num_vars(int n, int m, int N, bool equal = false) {
    auto Nu = equal ? N : N - 1;
    return N * n + Nu * m;
  }
};
template <int n, int m, typename T>
using SampledTrajectoryS = SampledTrajectory<KnotPointS<n, m, T>>;
template <int n, int m>
using SampledTrajectorySd = SampledTrajectory<KnotPointSd<n, m>>;

template <typename KP, typename FS = FunctionSignature>
void rollout(FS sig, const DiscreteDynamics<KP> *model,
             SampledTrajectory<KP> *Z, const typename KP::state_type &x0) {}
template <typename KP>
void rollout(Inplace sig, const DiscreteDynamics<KP> *model,
             SampledTrajectory<KP> *Z, const typename KP::state_type &x0) {
  Z->at(0).setstate(x0);
  for (auto k = 1; k < Z->length(); ++k) {
    propagate_dynamics(sig, model, &Z->at(k), Z->at(k - 1));
  }
}

// DiscretizedDynamics need to specify the Quadrature rule.
template <typename KP, template <typename> class Q,
          typename FS = FunctionSignature>
void rollout(FS sig, const DiscretizedDynamics<KP, Q> *model,
             SampledTrajectory<KP> *Z, const typename KP::state_type &x0) {}
template <typename KP, template <typename> class Q>
void rollout(Inplace sig, const DiscretizedDynamics<KP, Q> *model,
             SampledTrajectory<KP> *Z, const typename KP::state_type &x0) {
  Z->at(0).setstate(x0);
  for (auto k = 1; k < Z->length(); ++k) {
    propagate_dynamics(sig, model, &Z->at(k), Z->at(k - 1));
  }
}

#endif
