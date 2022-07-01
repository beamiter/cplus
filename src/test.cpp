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
  CarProblemSd<6, 2, DiagonalCostS> prob(x0, xf, uf, N, dt);
  iLQRSolverSd<6, 2, DiagonalCostS> solver(
      &prob, opts, stats, DiffMethod::UserDefined, Valbool<true>());
  solve(solver);
  LOG(INFO) << "come to here!";
}
void test(CostExpansion<double> &ce) {
  ce.data.push_back(StateControlExpansion<double, true>(4, 2));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  // Set logging level
  google::SetStderrLogging(google::GLOG_INFO);

  CostExpansion<double> ce(4, 2, 5);
  // CostExpansion<double> ce;
  // test(ce);
  LOG(INFO) << ce.data.size();
  auto &haha = ce.data[0];
  haha.grad(3) = 908;
  haha.data(5, 6) = 100;
  LOG(INFO) << haha.grad;
  LOG(INFO) << haha.data;
  ce.data.push_back(StateControlExpansion<double, true>(4, 2));
  auto &haha0 = ce.data[5];
  haha0.grad(3) = 908;
  haha0.data(5, 6) = 100;
  LOG(INFO) << haha0.grad;
  LOG(INFO) << haha0.data;
  ce.data.push_back(StateControlExpansion<double, true>(4, 2));
  haha0.grad(3) = 1908;
  haha0.data(5, 6) = 1100;
  LOG(INFO) << haha0.grad;
  LOG(INFO) << haha0.data;

  // std::vector<StateControlExpansion<double, true>> he;
  // he.push_back(StateControlExpansion<double, true>(4, 2));
  // auto &hehe = he[0];
  // hehe.grad(3) = 908;
  // hehe.data(5, 6) = 100;
  // LOG(INFO) << hehe.grad;
  // LOG(INFO) << hehe.data;

  // LOG(INFO) << RUN_ALL_TESTS();

  google::ShutdownGoogleLogging();
  return 1;
}
