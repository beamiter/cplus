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
using Eigen::Dynamic;
using Eigen::Map;

enum class RefPos {
  rear,
  cg,
  fg,
};

class CarModel : public ContinuousDynamics {
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
  int state_dim() const final { return 6; }
  int control_dim() const final { return 2; }

  RefPos ref;
  double L = 0.0;
  double lr = 0.0;
};

ProblemTemplate class CarProblem : public ProblemDeclare {
public:
  CarProblem(std::vector<T> x0_in, std::vector<T> xf_in, std::vector<T> uf_in,
             int N, double tf) {
    VectorXd x0 = Map<VectorXd>(x0_in.data(), x0_in.size());
    VectorXd xf = Map<VectorXd>(xf_in.data(), xf_in.size());
    VectorXd uf = Map<VectorXd>(uf_in.data(), uf_in.size());
    init(x0, xf, uf, N, tf);
  }
  void init(const VectorX<T> &x0, const VectorX<T> &xf, const VectorX<T> &uf,
            int N, double tf) {
    DiagonalMatrix<double, Nx> Q(10, 10, 50, 1, 1, 1);
    double rho = 1.0;
    auto R = rho * DiagonalMatrix<double, Nu>(1, 1);
    DiagonalMatrix<double, Nx> Qf(10, 10, 60, 1, 1, 1);

    auto obj = LQRObjective<Nx, Nu, T>(Q, R, Qf, xf, uf, N);
    // Model initialize.
    car_ = std::make_unique<CarModel>(CarModel());
    ProblemDeclare::init(car_.get(), &obj, x0, tf);

    // initial_controls, initial_states, rollout
  }

  // Members.
  std::unique_ptr<CarModel> car_;
};

#endif
