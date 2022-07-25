#ifndef BACKWARDPASS_H_
#define BACKWARDPASS_H_

#include "ilqr_solver.h"

// This is a supper cheap check.
template <typename Decom> bool isposdef(const Decom &in) {
  return in.info() == 0;
}

iLQRSolverTemplate std::vector<typename KP::base_type>
backwardpass(iLQRSolverDeclare *solver) {
  const auto &N = solver->N;
  const auto &D_vec = solver->D_vec; // dynamics expansion
  const auto &E = solver->Eerr_;     // cost expansion
  auto &S_vec = solver->S_vec;       // quadratic cost-to-go
  auto &Q_vec = solver->Q_vec;       // action-value expansion
  auto &K_vec = solver->K_vec;
  auto &d_vec = solver->d_vec;
  auto &Quu_reg = solver->Quu_reg;
  auto &Qux_reg = solver->Qux_reg;
  auto &DV = solver->DV;
  auto &Qtmp = solver->Qtmp;

  // Terminal cost-to-go
  S_vec[N - 1]->xx = E->at(N - 1)->xx;
  S_vec[N - 1]->x = E->at(N - 1)->x;
  int k = N - 2;
  std::fill(DV.begin(), DV.end(), 0);
  while (k >= 0) {
    const auto &A = D_vec[k]->fx;
    const auto &B = D_vec[k]->fu;
    const int m = solver->shared_models_[k]->control_dim();

    // Action-value expansion.
    Q_vec[k]->x = A.adjoint() * S_vec[k + 1]->x;
    Q_vec[k]->x += E->at(k)->x;

    Q_vec[k]->u = B.adjoint() * S_vec[k + 1]->x;
    LOG(INFO) << B.adjoint();
    LOG(INFO) << S_vec[k + 1]->x;
    LOG(INFO) << Q_vec[k]->u;
    Q_vec[k]->u += E->at(k)->u;
    LOG(INFO) << E->at(k)->u;
    LOG(INFO) << Q_vec[k]->u;

    Qtmp->xx = S_vec[k + 1]->xx * A;
    Q_vec[k]->xx = A.adjoint() * Qtmp->xx;
    Q_vec[k]->xx += E->at(k)->xx;

    // Something wrong with initial Qtmp->ux.
    Qtmp->ux = B.adjoint() * S_vec[k + 1]->xx;
    Q_vec[k]->uu = Qtmp->ux * B;
    Q_vec[k]->uu += E->at(k)->uu;

    Qtmp->xx = S_vec[k + 1]->xx * A;
    Q_vec[k]->ux = B.adjoint() * Qtmp->xx;
    Q_vec[k]->ux += E->at(k)->ux;

    // Regularization.
    const auto rou = solver->reg.rou;
    if (solver->opts.bp_reg_type == BpRegType::state) {
      Quu_reg = Q_vec[k]->uu;
      Quu_reg += rou * B.adjoint() * B;
      Qux_reg = Q_vec[k]->ux;
      Qux_reg += rou * A.adjoint() * A;
    } else if (solver->opts.bp_reg_type == BpRegType::control) {
      Quu_reg = Q_vec[k]->uu;
      for (int i = 0; i < m; ++i) {
        Quu_reg(i, i) += rou;
      }
      Qux_reg = Q_vec[k]->ux;
    }

    // Solve for gains.
    K_vec[k] = Qux_reg;
    // LOG(INFO) << K_vec[k];
    // LOG(INFO) << solver->gains[k];
    d_vec[k] = Q_vec[k]->u;
    const auto &Quu_fact = Quu_reg.llt();
    if (!isposdef(Quu_fact)) {
      LOG(INFO) << "Backwardpass cholesky failed at time step " << k;
      increaseregularization(solver);
      k = N - 2;
      std::fill(DV.begin(), DV.end(), 0);
      continue;
    }
    // Save time by solving for K and d at the same time (1 BLAS call)
    /// https://www.mathematik.uni-ulm.de/~lehn/FLENS/flens/examples/lapack-potrs.html
    // LOG(INFO) << solver->gains[k];
    // Solve linear system of equations with cholesky decompositon.
    solver->gains[k] = -1.0 * Quu_fact.solve(solver->gains[k]);
    // LOG(INFO) << solver->gains[k];

    // Update Cost-to-go.
    S_vec[k]->x = Q_vec[k]->x;
    Qtmp->u = Q_vec[k]->uu * d_vec[k];
    S_vec[k]->x += K_vec[k].adjoint() * Qtmp->u;
    S_vec[k]->x += K_vec[k].adjoint() * Q_vec[k]->u;
    S_vec[k]->x += Q_vec[k]->ux.adjoint() * d_vec[k];

    S_vec[k]->xx = Q_vec[k]->xx;
    Qtmp->ux = Q_vec[k]->uu * K_vec[k];
    S_vec[k]->xx += K_vec[k].adjoint() * Qtmp->ux;
    S_vec[k]->xx += K_vec[k].adjoint() * Q_vec[k]->ux;
    S_vec[k]->xx += Q_vec[k]->ux.adjoint() * K_vec[k];

    Qtmp->xx = S_vec[k]->xx.transpose();
    S_vec[k]->xx += Qtmp->xx;
    S_vec[k]->xx /= 2.;

    DV[0] += d_vec[k].dot(Q_vec[k]->u);
    DV[1] += 0.5 * d_vec[k].dot(Q_vec[k]->uu * d_vec[k]);

    --k;
  }
  decreaseregularization(solver);
  return DV;
}

#endif
