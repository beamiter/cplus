#include "ilqr_solver.h"
#include "robot_dynamics/discretized_dynamics.h"
#include "robot_dynamics/knotpoint.h"

iLQRSolverTemplate void initialize(iLQRSolverDeclare &solver) {
  reset(solver);

  solver.reg.rou = solver.opts.bp_reg_initial;
  solver.reg.d_rou = 0.0;

  if (solver.opts.closed_loop_initial_rollout) {
    CHECK(0);
    // In forwardpass.
    /* rollout(solver, 0.0);  */
  } else {
    const DiscretizedDynamics<RK4, KP> *tmp =
        dynamic_cast<const DiscretizedDynamics<RK4, KP> *>(
            solver.model[0].get());
    rollout(dynamics_signature(solver), tmp, solver.Z, solver.x0);
  }

  solver.Z_dot = solver.Z;
}

iLQRSolverTemplate void solve(iLQRSolverDeclare &solver) {
  initialize(solver);
  for (auto iter = 0; iter < solver.opts.iterations; ++iter) {
    //auto J_prev = cost(solver, solver.Z_dot);
  }
  // return solver;
}
