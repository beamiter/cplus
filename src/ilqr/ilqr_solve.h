#include "ilqr/dynamics_expansion.h"
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
    // A Dynamic_cast has runtime overhead because it checks object types at run
    // time using “Run-Time Type Information“.
    // If there is a surety we will never cast to wrong object then always avoid
    // dynamic_cast and use static_cast.
    const DiscretizedDynamics<KP, RK4> *tmp =
        static_cast<const DiscretizedDynamics<KP, RK4> *>(
            solver.model[0].get());
    rollout(dynamics_signature(solver), tmp, &solver.Z, solver.x0);
  }

  solver.Z_dot = solver.Z;
}

iLQRSolverTemplate void solve(iLQRSolverDeclare &solver) {
  initialize(solver);
  for (auto iter = 0; iter < solver.opts.iterations; ++iter) {
    auto J_prev = solver.cost(solver.Z_dot);
    LOG(INFO) << "********** " << J_prev;
    errstate_jacobian(solver.model, solver.G_vec, solver.Z_dot);
    dynamics_expansion(solver, solver.Z_dot);
    cost_expansion(solver.obj, solver.Efull, solver.Z_dot);
    error_expansion(solver.model, solver.Eerr, solver.Efull, solver.G_vec,
                    solver.Z_dot);
  }
  // return solver;
}
