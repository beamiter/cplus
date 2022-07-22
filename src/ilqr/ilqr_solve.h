#include <fstream>
#include <iostream>

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
    rollout(dynamics_signature(*solver),
            static_cast<const DiscretizedDynamics<KP, RK4> *>(
                solver->shared_models_.front().get()),
            &solver->Z, solver->x0);
    // rollout(dynamics_signature(*solver), solver->model.front().get(),
    //&solver->Z, solver->x0);
  }

  solver->Z_dot = solver->Z;
}

iLQRSolverTemplate void solve(iLQRSolverDeclare *solver) {
  initialize(solver);
  FILE *gnuplotPipe = popen("/usr/bin/gnuplot", "w");
  fprintf(gnuplotPipe, "set xrange [0:15]\n");
  fprintf(gnuplotPipe, "set yrange [-1.5:1]\n");
  const std::string name = "/home/yinj3/projects/cplus/data/data.dat";
  std::ofstream file(name);
  LOG(INFO) << name;
  for (auto iter = 0; iter < solver->opts.iterations; ++iter) {
    const auto J_prev = solver->cost(solver->Z_dot);
    // LOG(INFO) << "********** " << J_prev;

    // Calculate expansions.
    // Dummy.
    errstate_jacobian(solver->shared_models_, solver->G_vec, solver->Z_dot);
    dynamics_expansion(solver, solver->Z_dot);
    cost_expansion(solver->obj, solver->Efull_, solver->Z_dot);
    // Dummy.
    error_expansion(solver->shared_models_, solver->Eerr_, solver->Efull_, solver->G_vec,
                    solver->Z_dot);

    // Get next iterate.
    backwardpass(solver);

    const auto Jnew = forwardpass(solver, J_prev);

    // LOG(INFO) << solver->Z_dot;
    solver->Z = solver->Z_dot;
    // LOG(INFO) << solver->Z;
    for (const auto &pt : solver->Z_dot) {
      file << pt.state()(0) << " " << pt.state()(1) << std::endl;
    }
    std::string plot_str = "plot \"" + name + "\"\n";
    fprintf(gnuplotPipe, "%s", plot_str.c_str());
    fflush(gnuplotPipe);

    const double dJ = J_prev - Jnew;
    // Calculate the gradient of the new trajectory.
    const auto grad = gradient(solver, solver->Z);

    // Record the iteration.
    RecordParam param;
    param.dJ = dJ;
    param.cost = Jnew;
    param.gradient = grad;
    record_iteration(solver, param);

    // Check convergence.
    const bool exit = evaluate_convergence(solver);
    if (exit) {
      break;
    }
  }
  file.close();
  terminate(solver);
}

iLQRSolverTemplate double gradient(iLQRSolverDeclare *solver,
                                   const SampledTrajectory<KP> &Z) {
  double avggrad = 0.0;
  for (int k = 0; k < solver->d_vec.size(); ++k) {
    const auto m = solver->control_dim(k);
    double umax = -std::numeric_limits<double>::infinity();
    const auto &d = solver->d_vec[k];
    const auto &u = Z[k].control();
    for (int i = 0; i < m; ++i) {
      umax = std::max(umax, std::fabs(d(i)) / (std::fabs(u(i)) + 1));
    }
    solver->grad[k] = umax;
    avggrad += umax;
  }
  return avggrad / solver->d_vec.size();
}

iLQRSolverTemplate void record_iteration(iLQRSolverDeclare *solver,
                                         const RecordParam &param) {
  record_iteration(solver->stats_, param);
  const auto iter = solver->stats_.iterations;
  if (param.dJ < std::numeric_limits<double>::epsilon()) {
    solver->stats_.dJ_zero_counter += 1;
  } else {
    solver->stats_.dJ_zero_counter = 0;
  }
  LOG(INFO) << "-----------------";
  LOG(INFO) << "cost " << param.cost;
  LOG(INFO) << "iter " << iter;
  LOG(INFO) << "dJ " << param.dJ;
  LOG(INFO) << "grad " << param.gradient;
  LOG(INFO) << "dJ_zero " << solver->stats_.dJ_zero_counter;
  LOG(INFO) << "rou " << solver->reg.rou;
}

iLQRSolverTemplate bool evaluate_convergence(iLQRSolverDeclare *solver) {
  const auto &i = solver->stats().iterations;
  const auto &grad = solver->stats().gradient[i];
  const auto &dJ = solver->stats().dJ[i];
  const auto &J = solver->stats().cost[i];

  if (0.0 <= dJ && dJ < solver->opts.cost_tolerance &&
      grad < solver->opts.gradient_tolerance && !solver->stats().ls_failed) {
    LOG(INFO) << "Cost criteria satisfied.";
    solver->stats_.status = TerminationStatus::SOLVE_SUCCEEDED;
    return true;
  }

  if (i >= solver->opts.iterations) {
    LOG(INFO) << "Hit max iteration. Terminating.";
    solver->stats_.status = TerminationStatus::MAX_ITERATIONS;
    return true;
  }

  if (solver->stats_.dJ_zero_counter > solver->opts.dJ_counter_limit) {
    LOG(INFO) << "dJ Counter hit max. Terminating.";
    solver->stats_.status = TerminationStatus::NO_PROGRESS;
    return true;
  }

  if (J > solver->opts.max_cost_value) {
    LOG(INFO) << "Hit maximum cost. Terminating.";
    solver->stats_.status = TerminationStatus::MAXIMUM_COST;
    return true;
  }
  return false;
}
