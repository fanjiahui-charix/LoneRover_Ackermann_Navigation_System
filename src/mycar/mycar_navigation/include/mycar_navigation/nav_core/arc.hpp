#ifndef MYCAR_NAVIGATION_NAV_CORE_ARC_HPP_
#define MYCAR_NAVIGATION_NAV_CORE_ARC_HPP_

#include <vector>

#include "mycar_navigation/nav_core/types.hpp"

namespace mycar_navigation::nav_core
{

class ArcIntegrator
{
public:
  std::vector<Pose2D> integrate(
    const Pose2D & start, double v, double kappa, double horizon, double dt) const;
};

class CurvatureSampler
{
public:
  explicit CurvatureSampler(double minimum_turning_radius);

  std::vector<double> sample(int n = 31) const;
  std::vector<double> refineAround(double kappa_best, double half_width, int n) const;

  double maxCurvature() const;

private:
  double clampCurvature(double kappa) const;

  double minimum_turning_radius_;
  double max_curvature_;
};

}  // namespace mycar_navigation::nav_core

#endif  // MYCAR_NAVIGATION_NAV_CORE_ARC_HPP_
