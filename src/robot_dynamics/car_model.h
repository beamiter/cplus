#ifndef CAR_MODEL_
#define CAR_MODEL_

#include <Eigen/Dense>
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/util/Constants.h>

#include "base/base.h"
#include "dynamics.h"
#include "robot_dynamics/discrete_dynamics.h"
#include "trajectory_optimization/problem.h"

using Eigen::DiagonalMatrix;
using Eigen::Map;

enum class RefPos {
  rear,
  cg,
  fg,
};

template <typename KP> class CarModel : public ContinuousDynamics<KP> {
  using state_type = typename KP::state_type;
  using control_type = typename KP::control_type;

public:
  virtual ~CarModel() = default;
  static constexpr int N = 6;
  static constexpr int M = 2;
  CarModel(RefPos ref_in = RefPos::rear, double L_in = 2.7,
           double lr_in = 1.5) {
    this->ref = ref_in;
    this->L = L_in;
    this->lr = lr_in;
  }
  int state_dim() const final { return N; }
  int control_dim() const final { return M; }
  state_type dynamics(const state_type &x,
                      const control_type &u) const override {
    CHECK(0);
  }
  state_type dynamics(state_type *xdot, const state_type &x,
                      const control_type &u) const override {
    CHECK(0);
  }

  RefPos ref;
  double L = 0.0;
  double lr = 0.0;
};

template <typename KP> class CarProblem : public Problem<KP> {
  using base_type = typename KP::base_type;
  static constexpr int Nx = KP::N;
  static constexpr int Nu = KP::M;

public:
  CarProblem(std::vector<base_type> x0_in, std::vector<base_type> xf_in,
             std::vector<base_type> uf_in, int N, double tf) {
    VectorXd x0 = Map<VectorXd>(x0_in.data(), x0_in.size());
    VectorXd xf = Map<VectorXd>(xf_in.data(), xf_in.size());
    VectorXd uf = Map<VectorXd>(uf_in.data(), uf_in.size());
    init(x0, xf, uf, N, tf);
  }
  void init(const VectorX<base_type> &x0, const VectorX<base_type> &xf,
            const VectorX<base_type> &uf, int N, double tf) {
    DiagonalMatrix<double, Nx> Q(10, 10, 50, 1, 1, 1);
    auto R = DiagonalMatrix<double, Nu>(1, 1);
    const double rho = 1.0;
    R = rho * R;
    DiagonalMatrix<double, Nx> Qf(10, 10, 60, 1, 1, 1);

    obj_ = std::make_unique<Objective<DiagonalCost<KP::N, KP::M, base_type>>>(
        LQRObjective<KP::N, KP::M, base_type>(Q, R, Qf, xf, uf, N));
    // Model initialize.
    car_ = std::make_unique<CarModel<KP>>(CarModel<KP>());
    Problem<KP>::init(car_.get(), obj_.get(), x0, tf);

    // initial_controls, initial_states, rollout
  }

  // Members.
  std::unique_ptr<CarModel<KP>> car_;
  std::unique_ptr<Objective<DiagonalCost<Nx, Nu, base_type>>> obj_;
};

#endif
