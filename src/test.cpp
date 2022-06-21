#include <glog/logging.h>
#include <gtest/gtest.h>

#include "ilqr/cost_expansion.h"
#include "ilqr/ilqr_solve.h"
#include "ilqr/ilqr_solver.h"
#include "robot_dynamics/car_model.h"
#include "robot_dynamics/knotpoint.h"
#include "robot_dynamics/trajectories.h"
#include "solver_opts.h"
#include "trajectory_optimization/cost_functions.h"
#include "trajectory_optimization/problem.h"

using namespace google;

TEST(CostExpansionTest, StateControl) {
  auto opts = SolverOptionsD();
  auto stats = SolverStatsD();
  stats.parent = SolverName::iLQR;
  std::vector<double> x0({0, 0, 0, 0, 4, 0});
  std::vector<double> xf({13, -1.0, 0, 0, 1.0, 0});
  std::vector<double> uf({0, 0});
  const int N = 51;
  const double dt = 0.1;
  const double tf = 5.0;
  auto prob = CarProblemSd<6, 2, DiagonalCost>(x0, xf, uf, N, dt);
  auto solver = iLQRSolverSd<6, 2, DiagonalCost>(&prob, opts, stats, DiffMethod::UserDefined,
                                   Valbool<true>());
  solve(solver);
  LOG(INFO) << "come to here!";
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  // Set logging level
  google::SetStderrLogging(google::GLOG_INFO);

  LOG(INFO) << RUN_ALL_TESTS();

  google::ShutdownGoogleLogging();
  return 1;
}
