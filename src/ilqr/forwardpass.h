#ifndef FORWARDPASS_H_
#define FORWARDPASS_H_

#include <fstream>

#include "ilqr_solver.h"

iLQRSolverTemplate bool rollout(iLQRSolverDeclare *solver, double alpha) {
  const auto &Z = solver->Z;
  auto &Z_dot = solver->Z_dot;
  const auto &K_vec = solver->K_vec;
  const auto &d_vec = solver->d_vec;
  // It's safe to clear this.
  auto &delta_x = solver->dx;
  auto &delta_u = solver->du;

  Z_dot[0].setstate(solver->x0);
  const auto sig = dynamics_signature(*solver);
  for (int k = 0; k < solver->N - 1; ++k) {
    state_diff(solver->shared_models_[k].get(), delta_x[k], Z_dot[k].state(),
               Z[k].state());
    // TODO: Comes to know that d_vec[k] is abnormal.
    delta_u[k] = d_vec[k] * alpha;
    delta_u[k] += K_vec[k] * delta_x[k];

    delta_u[k] += Z[k].control();
    Z_dot[k].setcontrol(delta_u[k]);
    const auto max_u =
        *std::max_element(Z_dot[k].control().begin(), Z_dot[k].control().end(),
                          [](const auto &lhs, const auto &rhs) {
                            return std::fabs(lhs) < std::fabs(rhs);
                          });
    if (max_u > solver->opts.max_control_value || std::isnan(max_u)) {
      solver->stats_.status = TerminationStatus::CONTROL_LIMIT;
      return false;
    }

    propagate_dynamics(sig,
                       static_cast<const DiscretizedDynamics<KP, RK4> *>(
                           solver->shared_models_[k].get()),
                       &Z_dot[k + 1], Z_dot[k]);
    const auto max_x = *std::max_element(
        Z_dot[k + 1].state().begin(), Z_dot[k + 1].state().end(),
        [](const auto &lhs, const auto &rhs) {
          return std::fabs(lhs) < std::fabs(rhs);
        });
    if (max_x > solver->opts.max_state_value || std::isnan(max_x)) {
      solver->stats_.status = TerminationStatus::STATE_LIMIT;
      return false;
    }
  }
  solver->stats_.status = TerminationStatus::UNSOLVED;
  return true;
}

iLQRSolverTemplate double forwardpass(iLQRSolverDeclare *solver,
                                      double J_prev) {
  const auto &phi = solver->opts.line_search_decrease_factor;
  const auto &z_lb = solver->opts.line_search_lower_bound;
  const auto &z_ub = solver->opts.line_search_upper_bound;

  double alpha = 1.0;
  double J = std::numeric_limits<double>::infinity();
  double z = std::numeric_limits<double>::infinity();
  double expected = std::numeric_limits<double>::infinity();

  solver->stats_.ls_failed = false;
  const double max_iters = solver->opts.iterations_linesearch;
  bool exit_linesearch = false;
  PlotInit();
  for (int i = 0; i < max_iters; ++i) {
#ifdef ENABLE_PLOT
    const std::string name =
        "/home/yinj3/projects/cplus/data/data_" + std::to_string(i) + ".dat";
    std::ofstream file(name);
    PlotRun(name);
    PlotShow();
    for (const auto &pt : solver->Z) {
      file << pt.state()(0) << " " << pt.state()(1) << std::endl;
    }
    PlotShow();
    for (const auto &pt : solver->Z_dot) {
      file << pt.state()(0) << " " << pt.state()(1) << std::endl;
    }
    PlotShow();
#endif
    const auto isrolloutgood = rollout(solver, alpha);
    if (!isrolloutgood) {
      alpha *= phi;
      continue;
    }
#ifdef ENABLE_PLOT
    for (const auto &pt : solver->Z_dot) {
      file << pt.state()(0) << " " << pt.state()(1) << std::endl;
    }
    PlotShow();
#endif

    J = solver->obj->cost(solver->Z_dot);
    LOG(INFO) << "******* " << J_prev << ", ******* " << J;

    expected = -alpha * (solver->DV[0] + alpha * solver->DV[1]);

    if (0.0 < expected && expected < solver->opts.expected_decrease_tolerance) {
      alpha = 0.0;
      z = std::numeric_limits<double>::infinity();
      solver->Z_dot = solver->Z;
      J = J_prev;
      LOG(INFO) << "No step, expected decrease too small!";

      increaseregularization(solver);
      exit_linesearch = true;
    } else if (expected > 0.0) {
      z = (J_prev - J) / expected;
    } else {
      z = -1.0;
    }

    if (z_lb <= z && z <= z_ub) {
      exit_linesearch = true;
      break;
    }

    if (i == max_iters) {
      alpha = 0.0;
      solver->Z_dot = solver->Z;
      J = J_prev;
      LOG(INFO) << "Max linesearch iters: " << i;
      increaseregularization(solver);
      solver->reg.rou += solver->opts.bp_reg_fp;
      solver->stats_.ls_failed = true;
      exit_linesearch = true;
    }

    if (exit_linesearch) {
      break;
    }

    alpha *= phi;
#ifdef ENABLE_PLOT
    file.close();
#endif
  }

  if (J > J_prev) {
    solver->stats_.status = TerminationStatus::COST_INCREASE;
    return std::numeric_limits<double>::quiet_NaN();
  }
  return J;
}

#endif
