#include "ilqr_solver.h"

// This is a supper cheap check.
template <typename Decom> bool isposdef(const Decom &in) {
  return in.info() == 0;
}

iLQRSolverTemplate void backwardpass(iLQRSolverDeclare *solver) {
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
    const int m = solver->model[k]->control_dim();

    // Action-value expansion.
    Q_vec[k]->x = A.adjoint() * S_vec[k + 1]->x;
    Q_vec[k]->x += E->at(k)->x;

    Q_vec[k]->u = B.adjoint() * S_vec[k + 1]->x;
    Q_vec[k]->u += E->at(k)->u;

    Qtmp->xx = S_vec[k + 1]->xx * A;
    Q_vec[k]->xx = A.adjoint() * Qtmp->xx;
    Q_vec[k]->xx += E->at(k)->xx;

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
    d_vec[k] = Q_vec[k]->u;
    const auto &Quu_fact = Quu_reg.llt();
    if (!isposdef(Quu_fact)) {
      LOG(INFO) << "Backwardpass cholesky failed at time step " << k;
      increaseregularization(solver);
      k = N - 1;
      std::fill(DV.begin(), DV.end(), 0);
      continue;
    }
    // Save time by solving for K and d at the same time (1 BLAS call)

    --k;
  }
}
