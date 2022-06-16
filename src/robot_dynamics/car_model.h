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
public:
  virtual ~CarModel() = default;
  static constexpr int N = 6;
  static constexpr int M = 2;
  using state_type = Eigen::Vector<double, N>;
  using control_type = Eigen::Vector<double, M>;
  CarModel(RefPos ref_in = RefPos::rear, double L_in = 2.7,
           double lr_in = 1.5) {
    this->ref = ref_in;
    this->L = L_in;
    this->lr = lr_in;
  }
  int state_dim() const final { return N; }
  int control_dim() const final { return M; }

  RefPos ref;
  double L = 0.0;
  double lr = 0.0;
};

template <typename KP> class CarProblem : public Problem<KP> {
public:
  CarProblem(std::vector<typename KP::base_type> x0_in,
             std::vector<typename KP::base_type> xf_in,
             std::vector<typename KP::base_type> uf_in, int N, double tf) {
    VectorXd x0 = Map<VectorXd>(x0_in.data(), x0_in.size());
    VectorXd xf = Map<VectorXd>(xf_in.data(), xf_in.size());
    VectorXd uf = Map<VectorXd>(uf_in.data(), uf_in.size());
    init(x0, xf, uf, N, tf);
  }
  void init(const VectorX<typename KP::base_type> &x0,
            const VectorX<typename KP::base_type> &xf,
            const VectorX<typename KP::base_type> &uf, int N, double tf) {
    DiagonalMatrix<double, KP::N> Q(10, 10, 50, 1, 1, 1);
    auto R = DiagonalMatrix<double, KP::M>(1, 1);
    const double rho = 1.0;
    R = rho * R;
    DiagonalMatrix<double, KP::N> Qf(10, 10, 60, 1, 1, 1);

    obj_ = std::make_unique<
        Objective<DiagonalCost<KP::N, KP::M, typename KP::base_type>>>(
        LQRObjective<KP::N, KP::M, typename KP::base_type>(Q, R, Qf, xf, uf,
                                                           N));
    // Model initialize.
    car_ = std::make_unique<CarModel<KP>>(CarModel<KP>());
    Problem<KP>::init(car_.get(), obj_.get(), x0, tf);

    // initial_controls, initial_states, rollout
  }

  // Members.
  std::unique_ptr<CarModel<KP>> car_;
  std::unique_ptr<Objective<DiagonalCost<KP::N, KP::M, typename KP::base_type>>>
      obj_;
};

#endif
