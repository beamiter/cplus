#include "backwardpass.h"
#include "forwardpass.h"
#include "ilqr/dynamics_expansion.h"
#include "ilqr_solver.h"
#include "robot_dynamics/discretized_dynamics.h"
#include "robot_dynamics/knotpoint.h"

iLQRSolverTemplate void initialize(iLQRSolverDeclare *solver) {
  reset(*solver);

  solver->reg.rou = solver->opts.bp_reg_initial;
  solver->reg.d_rou = 0.0;

  if (solver->opts.closed_loop_initial_rollout) {
    CHECK(0);
    // In forwardpass.
    /* rollout(solver-> 0.0);  */
  } else {
    // static_cast<const DiscretizedDynamics<KP, Euler>
    // *>(solver->model[0].get());
    rollout(dynamics_signature(*solver), solver->model.front().get(),
            &solver->Z, solver->x0);
    // LOG(INFO) << solver->Z;
    // LOG(INFO) << solver->Z_dot;
  }

  solver->Z_dot = solver->Z;
}

iLQRSolverTemplate void solve(iLQRSolverDeclare *solver) {
  initialize(solver);
  for (auto iter = 0; iter < solver->opts.iterations; ++iter) {
    const auto J_prev = solver->cost(solver->Z_dot);
    LOG(INFO) << "********** " << J_prev;

    // Calculate expansions.
    errstate_jacobian(solver->model, solver->G_vec, solver->Z_dot);
    dynamics_expansion(solver, solver->Z_dot);
    cost_expansion(solver->obj, solver->Efull_, solver->Z_dot);
    error_expansion(solver->model, solver->Eerr_, solver->Efull_, solver->G_vec,
                    solver->Z_dot);

    // Get next iterate.
    backwardpass(solver);

    const auto Jnew = forwardpass(solver, J_prev);

    // LOG(INFO) << solver->Z;
    LOG(INFO) << solver->Z_dot;
    solver->Z = solver->Z_dot;

    const double dJ = J_prev - Jnew;
    // const auto grad = gradient(solver);

    // const bool exit = evaluate_convergence(solver);
    // if (exit) {
    // break;
    //}
  }
  // terminate(solver);
  // return solver;
}
