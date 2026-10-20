#include <cassert>
#include <limits>
#include <vector>

#include "plan_manage/tracking_checks.h"

int main()
{
  using scan_planner::validCubicKnots;
  using scan_planner::trackingTimesValid;
  const std::vector<double> uniform{-3, -2, -1, 0, 1, 2, 3, 4, 5, 6};
  assert(validCubicKnots(6, 3, uniform));
  assert(validCubicKnots(4, 3, {0, 0, 0, 0, 1, 1, 1, 1}));
  assert(!validCubicKnots(6, 2, uniform));
  assert(!validCubicKnots(6, 3, {0, 1}));
  assert(!validCubicKnots(3, 3, {0, 1, 2, 3, 4, 5, 6}));
  auto broken = uniform;
  broken[4] = std::numeric_limits<double>::quiet_NaN();
  assert(!validCubicKnots(6, 3, broken));
  broken = uniform;
  broken[4] = -10;
  assert(!validCubicKnots(6, 3, broken));
  assert(!validCubicKnots(6, 3, std::vector<double>(10, 0.0)));
  assert(!validCubicKnots(6, 3, {0, 0, 0, 0, 0, 1, 2, 3, 4, 5}));
  assert(trackingTimesValid(10.1, 10.0, 9.0, 2.0, 0.25, 0.25));
  assert(!trackingTimesValid(10.3, 10.0, 9.0, 2.0, 0.25, 0.25));
  assert(!trackingTimesValid(12.0, 12.0, 9.0, 2.0, 0.25, 0.25));
  assert(!trackingTimesValid(8.0, 10.0, 9.0, 2.0, 0.25, 0.25));
  assert(!trackingTimesValid(10.1, 10.0, 9.0, 0.0, 0.25, 0.25));
  assert(!trackingTimesValid(10.1, 10.0, 9.0,
                           std::numeric_limits<double>::infinity(), 0.25, 0.25));
  return 0;
}
