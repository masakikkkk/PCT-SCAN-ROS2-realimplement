// Standalone regression: no ROS, GTSAM, CUDA or Python bindings required.
#include <cassert>
#include "a_star/a_star_search.h"

int main() {
  Eigen::MatrixXd cost = Eigen::MatrixXd::Zero(5, 5);
  cost.col(2).setConstant(50.0);  // Separates the left and right components.
  const Eigen::MatrixXd height = Eigen::MatrixXd::Zero(5, 5);
  const Eigen::MatrixXd gateway = Eigen::MatrixXd::Zero(5, 5);
  Astar planner;
  planner.Init(20.0, 1, 0.2, 1.0, cost, height, gateway);
  const Eigen::Vector3i start(0, 0, 2);
  assert(!planner.Search(start, Eigen::Vector3i(0, 4, 2)));
  // Same map/start, now a reachable goal. Stale scores from the failed search
  // previously prevented this second request from expanding its neighbours.
  assert(planner.Search(start, Eigen::Vector3i(0, 1, 2)));
  assert(planner.GetResultMatrix().rows() >= 1);
  assert(!planner.Search(start, Eigen::Vector3i(0, 4, 2)));
  assert(planner.Search(start, Eigen::Vector3i(0, 1, 3)));
  return 0;
}
