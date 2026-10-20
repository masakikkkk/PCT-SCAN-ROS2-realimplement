#ifndef PLAN_MANAGE_TRACKING_CHECKS_H
#define PLAN_MANAGE_TRACKING_CHECKS_H

#include <cmath>
#include <cstddef>
#include <vector>

namespace scan_planner
{
inline bool validCubicKnots(std::size_t points, int order,
                            const std::vector<double> &knots)
{
  if (order != 3 || points < 4 || knots.size() != points + 4)
    return false;
  for (std::size_t i = 0; i < knots.size(); ++i)
    if (!std::isfinite(knots[i]) || (i && knots[i] < knots[i - 1]))
      return false;
  if (!(knots[points] > knots[3]) || !std::isfinite(knots[points] - knots[3]))
    return false;
  // UniformBspline differentiates twice. Reject zero denominators in either
  // derivative before the spline can reach Eigen / the command callback.
  for (std::size_t i = 0; i + 1 < points; ++i)
    if (!(knots[i + 4] > knots[i + 1]))
      return false;
  for (std::size_t i = 0; i + 2 < points; ++i)
    if (!(knots[i + 4] > knots[i + 2]))
      return false;
  return true;
}

inline bool trackingTimesValid(double now, double odom_at, double trajectory_at,
                              double duration, double odom_timeout, double grace)
{
  return std::isfinite(now) && std::isfinite(odom_at) &&
         std::isfinite(trajectory_at) && std::isfinite(duration) && duration > 0.0 &&
         std::isfinite(odom_timeout) && odom_timeout > 0.0 &&
         std::isfinite(grace) && grace >= 0.0 &&
         now >= odom_at && now - odom_at <= odom_timeout &&
         now >= trajectory_at && now - trajectory_at <= duration + grace;
}
}  // namespace scan_planner
#endif
