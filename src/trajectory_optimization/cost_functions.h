#ifndef COST_FUNTION_H
#define COST_FUNTION_H

#include <Eigen/Dense>
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <algorithm>
#include <stdexcept>

#include "base/base.h"
#include "robot_dynamics/scalar_function.h"

using Eigen::DiagonalMatrix;
using Eigen::Matrix;
using Eigen::MatrixX;
using Eigen::Vector;

class CostFunction : public ScalarFunction {
public:
};

#define QuadraticCostFunctionTypeName int n, int m, typename T
#define QuadraticCostFunctionTemplate template <int n, int m, typename T>
#define QuadraticCostFunctionDeclare QuadraticCostFunction<n, m, T>
QuadraticCostFunctionTemplate class QuadraticCostFunction
    : public CostFunction {
public:
  virtual bool is_blockdiag() const { return false; }
  int state_dim() const override { return n; }
  int control_dim() const override { return m; }

  virtual bool is_diag() const { return is_blockdiag(); }
};

#define DiagonalCostTypeName int n, int m, typename T
#define DiagonalCostTemplate template <int n, int m, typename T>
#define DiagonalCostDeclare DiagonalCost<n, m, T>
DiagonalCostTemplate class DiagonalCost : public QuadraticCostFunctionDeclare {
public:
  DiagonalMatrix<T, n> Q;
  DiagonalMatrix<T, m> R;
  Vector<T, n> q;
  Vector<T, m> r;
  T c;
  bool terminal = false;
  DiagonalCost(DiagonalMatrix<T, n> Q_in, DiagonalMatrix<T, m> R_in,
               Vector<T, n> q_in, Vector<T, m> r_in, T c_in,
               bool terminal = false, bool checks = true)
      : Q(Q_in), R(R_in), q(q_in), r(r_in), c(c_in) {
    if (checks) {
      if (std::any_of(Q_in.diagonal().begin(), Q_in.diagonal().end(),
                      [](const auto &p) { return p < 0; })) {
        CHECK(0);
        // throw std::runtime_error("nees to be positive semi-definite");
      } else if (std::any_of(Q_in.diagonal().begin(), Q_in.diagonal().end(),
                             [](const auto &p) { return p <= 0.0001; }) &&
                 !terminal) {
        // throw std::runtime_error("nees to be positive definite");
        CHECK(0);
      }
    }
  }
  DiagonalCost(Vector<T, n> Q_in, Vector<T, m> R_in, MatrixXd H,
               Vector<T, n> q_in, Vector<T, m> r_in, T c_in)
      : Q(Q_in), R(R_in), q(q_in), r(r_in), c(c_in) {}

  bool is_blockdiag() const final { return true; }

  bool is_diag() const final { return true; }
};

#define QuadraticCostTypeName int n, int m, typename T
#define QuadraticCostTemplate template <int n, int m, typename T>
#define QuadraticCostDeclare DianonalCost<n, m, T>
QuadraticCostTemplate class QuadraticCost
    : public QuadraticCostFunctionDeclare {
public:
  Matrix<T, n, n> Q;
  Matrix<T, m, m> R;
  Matrix<T, m, n> H;
  Vector<T, n> q;
  Vector<T, m> r;
  T c;
  bool terminal = false;
  bool zeroH;
  // A copy of r.
  Vector<T, m> tmpu;
  QuadraticCost(Matrix<T, n, n> Q_in, Matrix<T, m, m> R_in,
                Matrix<T, m, n> H_in, Vector<T, n> q_in, Vector<T, m> r_in,
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
    Q = MatrixX<T>::Zero(n, n);
    R = MatrixX<T>::Zero(m, m);
    H = MatrixX<T>::Zero(m, n);
    q = VectorX<T>::Zero(n);
    r = VectorX<T>::Zero(m);
    c = zero<T>();
    terminal = terminal_in;
    zeroH = (H.norm() < 0.001);
    tmpu = r;
  }

  int state_dim() const override { return q.size(); }
  int control_dim() const override { return r.size(); }
  bool is_blockdiag() const override { return zeroH; }
};

// QuadraticCost or DiagonalCost
DiagonalCostTemplate DiagonalCostDeclare LQRCost(DiagonalMatrix<T, n> Q,
                                                 DiagonalMatrix<T, m> R,
                                                 Vector<T, n> xf,
                                                 Vector<T, m> uf) {
  auto q = -Q * xf;
  auto r = -R * xf;
  double c = 0.5 * xf.adjoint() * Q * xf + 0.5 * uf.adjoint * uf;
  return DiagonalCostDeclare(Q, R, q, r, c);
}

#endif
