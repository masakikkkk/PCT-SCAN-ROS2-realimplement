#include <fstream>
#include <iostream>
#include <stdexcept>
#include "a_star/a_star_search.h"

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  std::ifstream input(argv[1]);
  int layers, rows, cols, sx, sy, gx, gy, layer;
  double resolution;
  if (!(input >> layers >> rows >> cols >> resolution >> layer >> sx >> sy >> gx >> gy) ||
      layers < 1 || rows < 1 || cols < 1)
    throw std::runtime_error("bad smoke-test map header");
  Eigen::MatrixXd cost(layers*rows, cols), ground(layers*rows, cols);
  for (int r = 0; r < layers*rows; ++r) for (int c = 0; c < cols; ++c)
    if (!(input >> cost(r,c) >> ground(r,c))) throw std::runtime_error("truncated smoke map");
  Astar planner;
  const Eigen::MatrixXd gateways = Eigen::MatrixXd::Zero(layers*rows, cols);
  planner.Init(20., layers, resolution, 1., cost, ground, gateways);
  if (!planner.Search(Eigen::Vector3i(layer,sx,sy), Eigen::Vector3i(layer,gx,gy)) ||
      planner.GetResultMatrix().rows() < 2)
    throw std::runtime_error("no PCT route through accumulated-room tomogram");
  std::cout << "Accumulated cloud -> CPU tomogram -> native PCT A*: PASS\n";
  return 0;
}
