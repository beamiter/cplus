#ifndef COST_FUNTION_H
#define COST_FUNTION_H

#include <Eigen/Dense>
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <algorithm>
#include <stdexcept>

#include "base/base.h"
#include "robot_dynamics/knotpoint.h"
#include "robot_dynamics/scalar_function.h"

using Eigen::DiagonalMatrix;
using Eigen::Matrix;
using Eigen::MatrixX;
using Eigen::Vector;

class CostFunction : public ScalarFunction {
public:
};

#define QuadraticCostFunctionTypeName typename KP
#define QuadraticCostFunctionTemplate template <typename KP>
#define QuadraticCostFunctionDeclare QuadraticCostFunction<KP>
QuadraticCostFunctionTemplate class QuadraticCostFunction
    : public CostFunction {
  static constexpr int Nx = KP::N;
  static constexpr int Nu = KP::M;
  using T = typename KP::base_type;
  using state_type = typename KP::state_type;
  using control_type = typename KP::control_type;

public:
  // Pure virtual functions.
  // virtual double stage_cost(const state_type &x, const control_type &u) = 0;

  // Virtual functions.
  virtual bool is_blockdiag() const { return false; }
  // TODO: cost.Q isa Diagonal && cost.R isa Diagonal.
  virtual bool is_diag() const { return is_blockdiag(); }

  // Functions.
  int state_dim() const override { return Nx; }
  int control_dim() const override { return Nu; }
};

QuadraticCostFunctionTemplate class DiagonalCost
    : public QuadraticCostFunctionDeclare {
  static constexpr int Nx = KP::N;
  static constexpr int Nu = KP::M;
  using T = typename KP::base_type;
  using state_type = typename KP::state_type;
  using control_type = typename KP::control_type;

public:
  DiagonalMatrix<T, Nx> Q;
  DiagonalMatrix<T, Nu> R;
  Vector<T, Nx> q;
  Vector<T, Nu> r;
  T c;
  bool terminal = false;
  DiagonalCost(DiagonalMatrix<T, Nx> Q_in, DiagonalMatrix<T, Nu> R_in,
               Vector<T, Nx> q_in, Vector<T, Nu> r_in, T c_in,
               bool terminal = false, bool checks = true)
      : Q(Q_in), R(R_in), q(q_in), r(r_in), c(c_in) {
    if (checks) {
      if (std::any_of(Q_in.diagonal().begin(), Q_in.diagonal().end(),
                      [](const auto &p) { return p < 0; })) {
        CHECK(0);
      } else if (std::any_of(Q_in.diagonal().begin(), Q_in.diagonal().end(),
                             [](const auto &p) { return p <= 0.0001; }) &&
                 !terminal) {
        CHECK(0);
      }
    }
  }
  DiagonalCost(Vector<T, Nx> Q_in, Vector<T, Nu> R_in, MatrixXd H,
               Vector<T, Nx> q_in, Vector<T, Nu> r_in, T c_in)
      : Q(Q_in), R(R_in), q(q_in), r(r_in), c(c_in) {}

  bool is_blockdiag() const final { return true; }

  bool is_diag() const final { return true; }
};
template <int n, int m, typename T>
using DiagonalCostS = DiagonalCost<KnotPointS<n, m, T>>;
template <int n, int m> using DiagonalCostSd = DiagonalCost<KnotPointSd<n, m>>;
template <int n, int m, typename T>
using DiagonalCostX = DiagonalCost<KnotPointX<n, m, T>>;
template <int n, int m> using DiagonalCostXd = DiagonalCost<KnotPointXd<n, m>>;

QuadraticCostFunctionTemplate class QuadraticCost
    : public QuadraticCostFunctionDeclare {
  static constexpr int Nx = KP::N;
  static constexpr int Nu = KP::M;
  using T = typename KP::base_type;
  using state_type = typename KP::state_type;
  using control_type = typename KP::control_type;

public:
  Matrix<T, Nx, Nx> Q;
  Matrix<T, Nu, Nu> R;
  Matrix<T, Nu, Nx> H;
  Vector<T, Nx> q;
  Vector<T, Nu> r;
  T c;
  bool terminal = false;
  bool zeroH;
  // A copy of r.
  Vector<T, Nu> tmpu;
  QuadraticCost(Matrix<T, Nx, Nx> Q_in, Matrix<T, Nu, Nu> R_in,
                Matrix<T, Nu, Nx> H_in, Vector<T, Nx> q_in, Vector<T, Nu> r_in,
                T c_in, bool checkes = true, bool terminal_in = false)
      : Q(Q_in), R(R_in), H(H_in), q(q_in), r(r_in), c(c_in),
        terminal(terminal_in), tmpu(r_in) {
    // check size.
    if (checkes) {
      if (!isPd<MatrixX<T>>(R_in)) {
        throw std::runtime_error("R is not possitive definite");
      } else if (!isPsd<MatrixX<T>>(Q_in)) {
        throw std::runtime_error("Q is not possitive semidefinite");
      }
    }
    zeroH = (H.norm() < 0.001);
  }
  QuadraticCost(bool terminal_in = false) {
    Q = MatrixX<T>::Zero(Nx, Nx);
    R = MatrixX<T>::Zero(Nu, Nu);
    H = MatrixX<T>::Zero(Nu, Nx);
    q = VectorX<T>::Zero(Nx);
    r = VectorX<T>::Zero(Nu);
    c = zero<T>();
    terminal = terminal_in;
    zeroH = (H.norm() < 0.001);
    tmpu = r;
  }

  int state_dim() const override { return q.size(); }
  int control_dim() const override { return r.size(); }
  bool is_blockdiag() const override { return zeroH; }
};
template <int n, int m, typename T>
using QuadraticCostS = QuadraticCost<KnotPointS<n, m, T>>;
template <int n, int m>
using QuadraticCostSd = QuadraticCost<KnotPointSd<n, m>>;
template <int n, int m, typename T>
using QuadraticCostX = QuadraticCost<KnotPointX<n, m, T>>;
template <int n, int m>
using QuadraticCostXd = QuadraticCost<KnotPointXd<n, m>>;

// QuadraticCost or DiagonalCost
QuadraticCostFunctionTemplate DiagonalCost<KP>
LQRCost(DiagonalMatrix<typename KP::base_type, KP::Nx> Q,
        DiagonalMatrix<typename KP::base_type, KP::Nu> R,
        Vector<typename KP::base_type, KP::Nx> xf,
        Vector<typename KP::base_type, KP::Nu> uf) {
  auto q = -Q * xf;
  auto r = -R * xf;
  double c = 0.5 * xf.adjoint() * Q * xf + 0.5 * uf.adjoint * uf;
  return DiagonalCost<KP>(Q, R, q, r, c);
}

#endif
