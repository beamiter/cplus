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
    auto J_prev = solver.cost(solver.Z_dot);
    LOG(INFO) << "********** " << J_prev;
    LOG(INFO) << solver.Efull.data[0].grad;
    LOG(INFO) << solver.Efull.data[0].data;
    errstate_jacobian(solver.model, solver.G_vec, solver.Z_dot);
    dynamics_expansion(solver, solver.Z_dot);
    cost_expansion(solver.obj, solver.Efull, solver.Z_dot);
    solver.Efull.data[0].grad(3) = 1789;
    LOG(INFO) << solver.Efull.data[0].grad;
    LOG(INFO) << solver.Efull.data[0].data;
  }
  // return solver;
}
