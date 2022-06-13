#include "integration.h"

#include <Eigen/src/Core/ArithmeticSequence.h>
#include <Eigen/src/Core/Matrix.h>

#include "base/base.h"
#include "discrete_dynamics.h"
#include "discretized_dynamics.h"
#include "dynamics.h"

template <typename P, AbstractKnotPointTypeName>
void jacobian(Euler, FunctionSignature, const ContinuousDynamics *model, P *J,
              const typename AbstractKnotPointDeclare::state_type &xn,
              const typename AbstractKnotPointDeclare::state_type &x,
              const typename AbstractKnotPointDeclare::control_type &u, T t,
              T h) {
  jacobian(model, J, xn, x, u, t);
  J *= h;
  for (auto i = 0; i < model->state_dim(); ++i) {
    J(i, i) += 1.0;
  }
}

template <typename P, AbstractKnotPointTypeName>
void jacobian(RK3 *inte, FunctionSignature sig, const ContinuousDynamics *model,
              P *J, const typename AbstractKnotPointDeclare::state_type &xn,
              const typename AbstractKnotPointDeclare::state_type &x,
              const typename AbstractKnotPointDeclare::control_type &u, T t,
              T h) {
  if (FunctionSignature::StaticReturn == sig) {
    int n, m, p;
    std::tie(n, m, p) = model->dims();
    const auto ix = Eigen::seq(0, n);
    const auto iu = Eigen::seqN(n, m);
    const auto k1 = dynamics(model, x, u, t) * h;
    const auto k2 = dynamics(model, x + k1 / 2, u, t + h / 2) * h;

    jacobian(model, J, xn, x, u, t);
    auto &A1 = J(ix, ix);
    auto &B1 = J(ix, iu);

    jacobian(model, J, xn, x + k1 / 2, u, t + h / 2);
    auto &A2 = J(ix, ix);
    auto &B2 = J(ix, iu);

    jacobian(model, J, xn, x - k1 + 2 * k2, u, t + h);
    auto &A3 = J(ix, ix);
    auto &B3 = J(ix, iu);

    const auto dA1 = A1 * h;
    const auto dA2 = A2 * (MatrixXd::Identity(n, n) + 0.5 * dA1) * h;
    const auto dA3 = A3 * (MatrixXd::Identity(n, n) - dA1 + 2 * dA2) * h;

    const auto dB1 = B1 * h;
    const auto dB2 = B2 * h + 0.5 * A2 * dB1 * h;
    const auto dB3 = B3 * h + A3 * (2 * dB2 - dB1) * h;

    J(ix, ix) = MatrixXd::Identity(n, n) + (dA1 + 4 * dA2 + dA3) / 6;
    J(ix, iu) = (dB1 + 4 * dB2 + dB3) / 6;
  } else if (FunctionSignature::Inplace == sig) {
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
    const auto ix = Eigen::seq(0, n);
    const auto iu = Eigen::seqN(n, m);

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
    dA2 = A2 * (MatrixXd::Identity(n, n) + 0.5 * dA1) * h;
    dA3 = A3 * (MatrixXd::Identity(n, n) - dA1 + 2 * dA2) * h;

    dB1 = B1 * h;
    dB2 = B2 * h + 0.5 * A2 * dB1 * h;
    dB3 = B3 * h + A3 * (2 * dB2 - dB1) * h;

    J(ix, ix) = (dA1 + 4 * dA2 + dA3) / 6;
    J(ix, ix) += MatrixXd::Identity(n, n);
    J(ix, iu) = (dB1 + 4 * dB2 + dB3) / 6;
  }
}

template <typename P, AbstractKnotPointTypeName>
void jacobian(RK4 *inte, FunctionSignature sig, const ContinuousDynamics *model,
              P *J, const typename AbstractKnotPointDeclare::state_type &xn,
              const typename AbstractKnotPointDeclare::state_type &x,
              const typename AbstractKnotPointDeclare::control_type &u, T t,
              T h) {
  if (FunctionSignature::StaticReturn == sig) {
    int n, m, p;
    std::tie(n, m, p) = model->dims();
    const auto ix = Eigen::seq(0, n);
    const auto iu = Eigen::seqN(n, m);
    const auto k1 = dynamics(model, x, u, t) * h;
    const auto k2 = dynamics(model, x + k1 / 2, u, t + h / 2) * h;
    const auto k3 = dynamics(model, x + k2 / 2, u, t + h / 2) * h;

    jacobian(model, J, xn, x, u, t);
    auto &A1 = J(ix, ix);
    auto &B1 = J(ix, iu);

    jacobian(model, J, xn, x + k1 / 2, u, t + h / 2);
    auto &A2 = J(ix, ix);
    auto &B2 = J(ix, iu);

    jacobian(model, J, xn, x + k2 / 2, u, t + h / 2);
    auto &A3 = J(ix, ix);
    auto &B3 = J(ix, iu);

    jacobian(model, J, xn, x + k3, u, t + h);
    auto &A4 = J(ix, ix);
    auto &B4 = J(ix, iu);

    const auto dA1 = A1 * h;
    const auto dA2 = A2 * (MatrixXd::Identity(n, n) + 0.5 * dA1) * h;
    const auto dA3 = A3 * (MatrixXd::Identity(n, n) + 0.5 * dA2) * h;
    const auto dA4 = A4 * (MatrixXd::Identity(n, n) + dA3) * h;

    const auto dB1 = B1 * h;
    const auto dB2 = B2 * h + 0.5 * A2 * dB1 * h;
    const auto dB3 = B2 * h + 0.5 * A3 * dB2 * h;
    const auto dB4 = B4 * h + A4 * dB3 * h;

    J(ix, ix) = MatrixXd::Identity(n, n) + (dA1 + 2 * dA2 + 2 * dA3 + dA4) / 6;
    J(ix, iu) = (dB1 + 2 * dB2 + 2 * dB3 + dB4) / 6;
  } else if (FunctionSignature::Inplace == sig) {
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
    const auto ix = Eigen::seq(0, n);
    const auto iu = Eigen::seqN(n, m);

    jacobian(model, J, k1, x, u, t);
    dynamics(model, k1, x, u, t);
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
    dA2 = A2 * (MatrixXd::Identity(n, n) + 0.5 * dA1) * h;
    dA3 = A3 * (MatrixXd::Identity(n, n) + 0.5 * dA2) * h;
    dA4 = A4 * (MatrixXd::Identity(n, n) + dA3) * h;

    dB1 = B1 * h;
    dB2 = B2 * h + 0.5 * A2 * dB1 * h;
    dB3 = B3 * h + 0.5 * A3 * dB2 * h;

    dB4 = B4 * h + A4 * dB3 * h;

    J(ix, ix) = (dA1 + 2 * dA2 + 2 * dA3 + dA4) / 6;
    J(ix, ix) += MatrixXd::Identity(n, n);
    J(ix, iu) = (dB1 + 2 * dB2 + 2 * dB3 + dB4) / 6;
  }
}
