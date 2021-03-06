#ifndef INTEGRATION_H
#define INTEGRATION_H

#include <tuple>

#include "dynamics.h"

struct QuadratureRule {};
struct Explicit : QuadratureRule {};
struct Implicit : QuadratureRule {};

// Speedup policy
template <typename T> struct ADVecotor {};

/*
 * Explicit Methods
 */

template <typename KP> struct Euler : Explicit {};
template <typename KP>
typename KP::state_type
integrate(Euler<KP>, const ContinuousDynamics<KP> *model,
          const typename KP::state_type &x, const typename KP::control_type &u,
          typename KP::base_type t, typename KP::base_type h) {
  auto xdot = dynamics<KP>(model, x, u, t);
  return x + h * xdot;
}
template <typename KP>
void integrate(Euler<KP>, const ContinuousDynamics<KP> *model,
               typename KP::ref_vector_type xn,
               const typename KP::state_type &x,
               const typename KP::control_type &u, typename KP::base_type t,
               typename KP::base_type h) {
  dynamics<KP>(model, xn, x, u, t);
  xn *= h;
  xn += x;
}
template <typename KP, typename FS = FunctionSignature>
void jacobian(Euler<KP> *, FS, const ContinuousDynamics<KP> *model,
              typename KP::ref_matrix_type J, typename KP::ref_vector_type xn,
              const typename KP::state_type &x,
              const typename KP::control_type &u, typename KP::base_type t,
              typename KP::base_type h) {
  static_assert(std::is_base_of<FunctionSignature, FS>::value,
                "FS is not derived of FunctionSignature");
}
template <typename KP>
void jacobian(Euler<KP> *, StaticReturn, const ContinuousDynamics<KP> *model,
              typename KP::ref_matrix_type J, typename KP::ref_vector_type xn,
              const typename KP::state_type &x,
              const typename KP::control_type &u, typename KP::base_type t,
              typename KP::base_type h) {
  jacobian(model, J, xn, x, u, t);
  J *= h;
  for (auto i = 0; i < model->state_dim(); ++i) {
    J(i, i) += 1.0;
  }
}
template <typename KP>
void jacobian(Euler<KP> *, Inplace, const ContinuousDynamics<KP> *model,
              typename KP::ref_matrix_type J, typename KP::ref_vector_type xn,
              const typename KP::state_type &x,
              const typename KP::control_type &u, typename KP::base_type t,
              typename KP::base_type h) {
  jacobian(model, J, xn, x, u, t);
  J *= h;
  for (auto i = 0; i < model->state_dim(); ++i) {
    J(i, i) += 1.0;
  }
}

template <typename KP> struct RK3 : Explicit {
  typename KP::state_type k1;
  typename KP::state_type k2;
  typename KP::state_type k3;
  std::array<Matrix<typename KP::base_type, KP::N, KP::N>, 3> A;
  std::array<Matrix<typename KP::base_type, KP::N, KP::M>, 3> B;
  std::array<Matrix<typename KP::base_type, KP::N, KP::N>, 3> dA;
  std::array<Matrix<typename KP::base_type, KP::N, KP::M>, 3> dB;
  RK3() {
    k1.setZero();
    k2.setZero();
    k3.setZero();
    loop(0, 3, [this](const int k) {
      this->A.at(k).setZero();
      this->B.at(k).setZero();
      this->dA.at(k).setZero();
      this->dB.at(k).setZero();
    });
  }
};
template <typename KP>
inline auto getks(const RK3<KP> &inte)
    -> decltype(std::make_tuple(inte.k1, inte.k2, inte.k3)) {
  return std::make_tuple(inte.k1, inte.k2, inte.k3);
}
template <typename KP>
typename KP::state_type
integrate(RK3<KP>, const ContinuousDynamics<KP> *model,
          const typename KP::state_type &x, const typename KP::control_type &u,
          typename KP::base_type t, typename KP::base_type h) {
  const auto k1 = dynamics<KP>(model, x, u, t) * h;
  const auto k2 = dynamics<KP>(model, x + k1 / 2, u, t + h / 2) * h;
  const auto k3 = dynamics<KP>(model, x - k1 + 2 * k2, u, t + h) * h;
  return x + (k1 + 4 * k2 + k3) / 6;
}
template <typename KP>
void integrate(RK3<KP> *inte, const ContinuousDynamics<KP> *model,
               typename KP::ref_vector_type xn,
               const typename KP::state_type &x,
               const typename KP::control_type &u, typename KP::base_type t,
               typename KP::base_type h) {
  dynamics<KP>(model, inte->k1, x, u, t);
  xn = x + inte->k1 * h / 2;
  dynamics<KP>(model, inte->k2, xn, u, t + h / 2);
  xn = x - inte->k1 * h + 2 * inte->k2 * h;
  dynamics<KP>(model, inte->k3, xn, u, t + h);
  xn = x + h * (inte->k1 + 4 * inte->k2 + inte->k3) / 6;
}
template <typename KP, typename FS = FunctionSignature>
void jacobian(RK3<KP> *inte, FunctionSignature sig,
              const ContinuousDynamics<KP> *model,
              typename KP::ref_matrix_type J, typename KP::ref_vector_type xn,
              const typename KP::state_type &x,
              const typename KP::control_type &u, typename KP::base_type t,
              typename KP::base_type h) {
  static_assert(std::is_base_of<FunctionSignature, FS>::value,
                "FS is not derived of FunctionSignature");
}
template <typename KP>
void jacobian(RK3<KP> *inte, StaticReturn, const ContinuousDynamics<KP> *model,
              typename KP::ref_matrix_type J, typename KP::ref_vector_type xn,
              const typename KP::state_type &x,
              const typename KP::control_type &u, typename KP::base_type t,
              typename KP::base_type h) {
  int n, m, p;
  std::tie(n, m, p) = model->dims();
  const auto ix = Eigen::seq(0, n - 1);
  const auto iu = Eigen::seqN(n, m);
  const auto k1 = dynamics(model, x, u, t) * h;
  const auto k2 = dynamics(model, x + k1 / 2, u, t + h / 2) * h;
  const auto &n_identity = MatrixXd::Identity(n, n);

  jacobian(model, J, xn, x, u, t);
  const auto &A1 = J(ix, ix);
  const auto &B1 = J(ix, iu);

  jacobian(model, J, xn, x + k1 / 2, u, t + h / 2);
  const auto &A2 = J(ix, ix);
  const auto &B2 = J(ix, iu);

  jacobian(model, J, xn, x - k1 + 2 * k2, u, t + h);
  const auto &A3 = J(ix, ix);
  const auto &B3 = J(ix, iu);

  const auto dA1 = A1 * h;
  const auto dA2 = A2 * (n_identity + 0.5 * dA1) * h;
  const auto dA3 = A3 * (n_identity - dA1 + 2 * dA2) * h;

  const auto dB1 = B1 * h;
  const auto dB2 = B2 * h + 0.5 * A2 * dB1 * h;
  const auto dB3 = B3 * h + A3 * (2 * dB2 - dB1) * h;

  // Be careful with Eigen's delay calcualtion.
  J(ix, ix) = n_identity + (dA1 + 4 * dA2 + dA3) / 6;
  J(ix, iu) = (dB1 + 4 * dB2 + dB3) / 6;
  // TODO. Check if (1,,3), (0,6), (1, 7) is true or not.
}
template <typename KP>
void jacobian(RK3<KP> *inte, Inplace, const ContinuousDynamics<KP> *model,
              typename KP::ref_matrix_type J, typename KP::ref_vector_type xn,
              const typename KP::state_type &x,
              const typename KP::control_type &u, typename KP::base_type t,
              typename KP::base_type h) {
  auto &k1 = inte->k1;
  auto &k2 = inte->k2;
  auto &k3 = inte->k3;
  auto &A1 = inte->A[0];
  auto &A2 = inte->A[1];
  auto &A3 = inte->A[2];
  auto &B1 = inte->B[0];
  auto &B2 = inte->B[1];
  auto &B3 = inte->B[2];
  auto &dA1 = inte->dA[0];
  auto &dA2 = inte->dA[1];
  auto &dA3 = inte->dA[2];
  auto &dB1 = inte->dB[0];
  auto &dB2 = inte->dB[1];
  auto &dB3 = inte->dB[2];
  int n, m, p;
  std::tie(n, m, p) = model->dims();
  const auto ix = Eigen::seq(0, n - 1);
  const auto iu = Eigen::seqN(n, m);
  const auto &n_identity = MatrixXd::Identity(n, n);

  jacobian(model, J, k1, x, u, t);
  dynamics(model, k1, x, u, t);
  A1 = J(ix, ix);
  B1 = J(ix, iu);

  xn = x + k1 * h / 2;
  jacobian(model, J, k2, xn, u, t + h / 2);
  dynamics(model, k2, xn, u, t + h / 2);
  A2 = J(ix, ix);
  B2 = J(ix, iu);

  xn = x - k1 * h + 2 * k2 * h;
  jacobian(model, J, k3, xn, u, t + h);
  dynamics(model, k3, xn, u, t + h);
  A3 = J(ix, ix);
  B3 = J(ix, iu);

  dA1 = A1 * h;
  dA2 = A2 * (n_identity + 0.5 * dA1) * h;
  dA3 = A3 * (n_identity - dA1 + 2 * dA2) * h;

  dB1 = B1 * h;
  dB2 = B2 * h + 0.5 * A2 * dB1 * h;
  dB3 = B3 * h + A3 * (2 * dB2 - dB1) * h;

  // Be careful with Eigen's delay calcualtion.
  J(ix, ix) = n_identity + (dA1 + 4 * dA2 + dA3) / 6;
  J(ix, iu) = (dB1 + 4 * dB2 + dB3) / 6;
}

template <typename KP> struct RK4 : Explicit {
  typename KP::state_type k1;
  typename KP::state_type k2;
  typename KP::state_type k3;
  typename KP::state_type k4;
  std::array<Matrix<typename KP::base_type, KP::N, KP::N>, 4> A;
  std::array<Matrix<typename KP::base_type, KP::N, KP::M>, 4> B;
  std::array<Matrix<typename KP::base_type, KP::N, KP::N>, 4> dA;
  std::array<Matrix<typename KP::base_type, KP::N, KP::M>, 4> dB;
  RK4() {
    k1.setZero();
    k2.setZero();
    k3.setZero();
    k4.setZero();
    loop(0, 4, [this](const int k) {
      this->A.at(k).setZero();
      this->B.at(k).setZero();
      this->dA.at(k).setZero();
      this->dB.at(k).setZero();
    });
  }
};
template <typename KP>
inline auto getks(const RK4<KP> &inte)
    -> decltype(std::make_tuple(inte.k1, inte.k2, inte.k3, inte.k4)) {
  return std::make_tuple(inte.k1, inte.k2, inte.k3, inte.k4);
}
// Profile: 2.00%
template <typename KP>
typename KP::state_type
integrate(const RK4<KP> &, const ContinuousDynamics<KP> *model,
          const typename KP::state_type &x, const typename KP::control_type &u,
          typename KP::base_type t, typename KP::base_type h) {
  const auto k1 = dynamics<KP>(model, x, u, t) * h;
  const auto k2 = dynamics<KP>(model, x + k1 / 2, u, t + h / 2) * h;
  const auto k3 = dynamics<KP>(model, x + k2 / 2, u, t + h / 2) * h;
  const auto k4 = dynamics<KP>(model, x + k3, u, t + h) * h;
  return x + (k1 + 2 * k2 + 2 * k3 + k4) / 6;
}
template <typename KP>
void integrate(RK4<KP> *inte, const ContinuousDynamics<KP> *model,
               typename KP::ref_vector_type xn,
               const typename KP::state_type &x,
               const typename KP::control_type &u, typename KP::base_type t,
               typename KP::base_type h) {
  dynamics<KP>(model, inte->k1, x, u, t);
  xn = x + inte->k1 * h / 2;
  dynamics<KP>(model, inte->k2, xn, u, t + h / 2);
  xn = x + inte->k2 * h / 2;
  dynamics<KP>(model, inte->k3, xn, u, t + h / 2);
  xn = x + inte->k3 * h;
  dynamics<KP>(model, inte->k4, xn, u, t + h);
  xn = x + h * (inte->k1 + 2 * inte->k2 + 2 * inte->k3 + inte->k4) / 6;
}
template <typename KP, typename FS = FunctionSignature>
void jacobian(RK4<KP> *inte, FS sig, const ContinuousDynamics<KP> *model,
              typename KP::ref_matrix_type J, typename KP::ref_vector_type xn,
              const typename KP::state_type &x,
              const typename KP::control_type &u, typename KP::base_type t,
              typename KP::base_type h) {
  static_assert(std::is_base_of<FunctionSignature, FS>::value,
                "FS is not derived of FunctionSignature");
}
// Profile: 41.4%
template <typename KP>
void jacobian(RK4<KP> *inte, StaticReturn, const ContinuousDynamics<KP> *model,
              typename KP::ref_matrix_type J, typename KP::ref_vector_type xn,
              const typename KP::state_type &x,
              const typename KP::control_type &u, typename KP::base_type t,
              typename KP::base_type h) {
  int n, m, p;
  std::tie(n, m, p) = model->dims();
  const auto ix = Eigen::seq(0, n - 1);
  const auto iu = Eigen::seqN(n, m);
  const auto k1 = dynamics(model, x, u, t) * h;
  const auto k2 = dynamics(model, x + k1 / 2, u, t + h / 2) * h;
  const auto k3 = dynamics(model, x + k2 / 2, u, t + h / 2) * h;
  const auto &n_identity = MatrixXd::Identity(n, n);

  jacobian(model, J, xn, x, u, t);
  const auto &A1 = J(ix, ix);
  const auto &B1 = J(ix, iu);

  jacobian(model, J, xn, x + k1 / 2, u, t + h / 2);
  const auto &A2 = J(ix, ix);
  const auto &B2 = J(ix, iu);

  jacobian(model, J, xn, x + k2 / 2, u, t + h / 2);
  const auto &A3 = J(ix, ix);
  const auto &B3 = J(ix, iu);

  jacobian(model, J, xn, x + k3, u, t + h);
  const auto &A4 = J(ix, ix);
  const auto &B4 = J(ix, iu);

  const auto dA1 = A1 * h;
  const auto dA2 = A2 * (n_identity + 0.5 * dA1) * h;
  const auto dA3 = A3 * (n_identity + 0.5 * dA2) * h;
  const auto dA4 = A4 * (n_identity + dA3) * h;

  const auto dB1 = B1 * h;
  const auto dB2 = B2 * h + 0.5 * A2 * dB1 * h;
  const auto dB3 = B3 * h + 0.5 * A3 * dB2 * h;
  const auto dB4 = B4 * h + A4 * dB3 * h;

  // Be careful with Eigen's delay calcualtion.
  J(ix, ix) = n_identity + (dA1 + 2 * dA2 + 2 * dA3 + dA4) / 6;
  J(ix, iu) = (dB1 + 2 * dB2 + 2 * dB3 + dB4) / 6;
}
template <typename KP>
void jacobian(RK4<KP> *inte, Inplace, const ContinuousDynamics<KP> *model,
              typename KP::ref_matrix_type J, typename KP::ref_vector_type xn,
              const typename KP::state_type &x,
              const typename KP::control_type &u, typename KP::base_type t,
              typename KP::base_type h) {
  auto &k1 = inte->k1;
  auto &k2 = inte->k2;
  auto &k3 = inte->k3;
  auto &k4 = inte->k4;
  auto &A1 = inte->A[0];
  auto &A2 = inte->A[1];
  auto &A3 = inte->A[2];
  auto &A4 = inte->A[3];
  auto &B1 = inte->B[0];
  auto &B2 = inte->B[1];
  auto &B3 = inte->B[2];
  auto &B4 = inte->B[3];
  auto &dA1 = inte->dA[0];
  auto &dA2 = inte->dA[1];
  auto &dA3 = inte->dA[2];
  auto &dA4 = inte->dA[3];
  auto &dB1 = inte->dB[0];
  auto &dB2 = inte->dB[1];
  auto &dB3 = inte->dB[2];
  auto &dB4 = inte->dB[3];
  int n, m, p;
  std::tie(n, m, p) = model->dims();
  const auto ix = Eigen::seq(0, n - 1);
  const auto iu = Eigen::seqN(n, m);
  const auto &n_identity = MatrixXd::Identity(n, n);

  jacobian(model, J, inte->k1, x, u, t);
  dynamics(model, inte->k1, x, u, t);
  A1 = J(ix, ix);
  B1 = J(ix, iu);

  xn = x + k1 * h / 2;
  jacobian(model, J, k2, xn, u, t + h / 2);
  dynamics(model, k2, xn, u, t + h / 2);
  A2 = J(ix, ix);
  B2 = J(ix, iu);

  xn = x + k2 * h / 2;
  jacobian(model, J, k3, xn, u, t + h);
  dynamics(model, k3, xn, u, t + h);
  A3 = J(ix, ix);
  B3 = J(ix, iu);

  xn = x + k3 * h;
  jacobian(model, J, k4, xn, u, t + h);
  A4 = J(ix, iu);
  B4 = J(ix, iu);

  dA1 = A1 * h;
  dA2 = A2 * (n_identity + 0.5 * dA1) * h;
  dA3 = A3 * (n_identity + 0.5 * dA2) * h;
  dA4 = A4 * (n_identity + dA3) * h;

  dB1 = B1 * h;
  dB2 = B2 * h + 0.5 * A2 * dB1 * h;
  dB3 = B3 * h + 0.5 * A3 * dB2 * h;

  dB4 = B4 * h + A4 * dB3 * h;

  // Be careful with Eigen's delay calcualtion.
  J(ix, ix) = n_identity + (dA1 + 2 * dA2 + 2 * dA3 + dA4) / 6;
  J(ix, iu) = (dB1 + 2 * dB2 + 2 * dB3 + dB4) / 6;
}

/*
 * Implicit Methods
 */

#endif
