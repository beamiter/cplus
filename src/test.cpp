#include <glog/logging.h>
#include <gtest/gtest.h>

#include "ilqr/cost_expansion.h"
#include "ilqr/ilqr_solver.h"
#include "robot_dynamics/car_model.h"
#include "robot_dynamics/knotpoint.h"
#include "robot_dynamics/trajectories.h"
#include "solver_opts.h"
#include "trajectory_optimization/problem.h"

using namespace google;

TEST(CostExpansionTest, StateControl) {
  auto opts = SolverOptionsD();
  auto stats = SolverStatsD();
  std::vector<double> x0({0, 0, 0, 0, 0, 0});
  std::vector<double> xf({12, 0, 0, 0, 0, 0});
  std::vector<double> uf({0, 0});
  double tf = 5.0;
  auto prob = CarProblem<6, 2, double>(x0, xf, uf, 51, 0.1);
  auto solver = iLQRSolverXd<6, 2>(&prob, opts, stats, DiffMethod::UserDefined,
                                   Valbool<true>());
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
