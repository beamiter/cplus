#ifndef COST_FUNTION_H
#define COST_FUNTION_H

#include <Eigen/Dense>
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <stdexcept>

#include "base/base.h"
#include "robot_dynamics/scalar_function.h"

using Eigen::DiagonalMatrix;
using Eigen::Dynamic;
using Eigen::Matrix;
using Eigen::MatrixX;
using Eigen::Vector;

#define CostFunctionTypeName typename F, typename S
#define CostFunctionTemplate template <typename F, typename S>
#define CostFunctionDeclare CostFunction<F, S>
CostFunctionTemplate class CostFunction : public ScalarFunctionDeclare {
public:
};

#define QuadraticCostFunctionTypeName typename F, typename S
#define QuadraticCostFunctionTemplate                                          \
  template <int n, int m, typename T, CostFunctionTypeName>
#define QuadraticCostFunctionDeclare QuadraticCostFunction<n, m, T, F, S>
QuadraticCostFunctionTemplate class QuadraticCostFunction
    : public CostFunctionDeclare {
public:
  virtual bool is_blockdiag() const { return false; }
  int state_dim() const override { return n; }
  int control_dim() const override { return m; }

  virtual bool is_diag() const { return is_blockdiag(); }
};

#define DiagonalCostTypeName typename F, typename S
#define DiagonalCostTemplate                                                   \
  template <int n, int m, typename T, CostFunctionTypeName>
#define DiagonalCostDeclare DiagonalCost<n, m, T, F, S>
DiagonalCostTemplate class DiagonalCost : public QuadraticCostFunctionDeclare {
public:
  DiagonalMatrix<T, n> Q;
  DiagonalMatrix<T, m> R;
  Vector<T, n> q;
  Vector<T, m> r;
  T c;
  bool terminal;
  DiagonalCost(Vector<T, n> Q_in, Vector<T, m> R_in, Vector<T, n> q_in,
               Vector<T, m> r_in, T c_in, bool terminal = false,
               bool checks = true)
      : Q(Q_in), R(R_in), q(q_in), r(r_in), c(c_in) {
    if (checks) {
      if ((Q_in < 0).any()) {
        throw std::runtime_error("nees to be positive semi-definite");
      }
      elseif((R_in <= 0).any() && !terminal) {
        throw std::runtime_error("nees to be positive definite");
      }
    }
  }
  DiagonalCost(Vector<T, n> Q_in, Vector<T, m> R_in, MatrixXd H,
               Vector<T, n> q_in, Vector<T, m> r_in, T c_in)
      : Q(Q_in), R(R_in), q(q_in), r(r_in), c(c_in) {}

  bool is_blockdiag() const final { return true; }

  bool is_diag() const final { return true; }
};

#define QuadraticCostTypeName typename F, typename S
#define QuadraticCostTemplate                                                  \
  template <int n, int m, typename T, CostFunctionTypeName>
#define QuadraticCostDeclare DianonalCost<n, m, T, F, S>
QuadraticCostTemplate class QuadraticCost
    : public QuadraticCostFunctionDeclare {
public:
  Matrix<T, n, n> Q;
  Matrix<T, m, m> R;
  Matrix<T, m, n> H;
  Vector<T, n> q;
  Vector<T, m> r;
  T c;
  bool terminal;
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
  double c = xf.adjoint() * Q * xf + 0.5 * uf.adjoint * uf;
  return DiagonalCostDeclare(Q, R, q, r, c);
}

#endif
