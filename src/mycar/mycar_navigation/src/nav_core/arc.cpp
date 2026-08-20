#include "mycar_navigation/nav_core/arc.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mycar_navigation::nav_core
{
namespace
{
constexpr double kStraightCurvatureEpsilon = 1e-9;

std::vector<double> makeUniformSamples(double lower, double upper, int n)
{
  if (n <= 0) {
    return {};
  }

  if (n == 1) {
    return {0.5 * (lower + upper)};
  }

  std::vector<double> samples;
  samples.reserve(static_cast<std::size_t>(n));

  const double step = (upper - lower) / static_cast<double>(n - 1);
  for (int index = 0; index < n; ++index) {
    samples.push_back(lower + static_cast<double>(index) * step);
  }

  samples.front() = lower;
  samples.back() = upper;
  return samples;
}
}  // namespace

std::vector<Pose2D> ArcIntegrator::integrate(
  const Pose2D & start, double v, double kappa, double horizon, double dt) const
{
  if (dt <= 0.0) {
    throw std::invalid_argument("dt must be positive");
  }

  if (horizon <= 0.0) {
    return {start};
  }

  const int steps = std::max(0, static_cast<int>(horizon / dt));
  std::vector<Pose2D> trajectory;
  trajectory.reserve(static_cast<std::size_t>(steps) + 1U);
  trajectory.push_back(start);

  Pose2D pose = start;
  const double omega = v * kappa;

  for (int step = 0; step < steps; ++step) {
    if (std::abs(kappa) <= kStraightCurvatureEpsilon) {
      pose.x += v * std::cos(pose.yaw) * dt;
      pose.y += v * std::sin(pose.yaw) * dt;
    } else {
      const double next_yaw = pose.yaw + omega * dt;
      pose.x += (v / omega) * (std::sin(next_yaw) - std::sin(pose.yaw));
      pose.y -= (v / omega) * (std::cos(next_yaw) - std::cos(pose.yaw));
      pose.yaw = next_yaw;
      trajectory.push_back(pose);
      continue;
    }

    pose.yaw += omega * dt;
    trajectory.push_back(pose);
  }

  return trajectory;
}

CurvatureSampler::CurvatureSampler(double minimum_turning_radius)
: minimum_turning_radius_(minimum_turning_radius),
  max_curvature_(0.0)
{
  if (minimum_turning_radius_ <= 0.0) {
    throw std::invalid_argument("minimum_turning_radius must be positive");
  }

  max_curvature_ = 1.0 / minimum_turning_radius_;
}

std::vector<double> CurvatureSampler::sample(int n) const
{
  return makeUniformSamples(-max_curvature_, max_curvature_, n);
}

std::vector<double> CurvatureSampler::refineAround(double kappa_best, double half_width, int n) const
{
  if (n <= 0) {
    return {};
  }

  const double clamped_half_width = std::max(0.0, half_width);
  const double center = clampCurvature(kappa_best);
  const double lower = clampCurvature(center - clamped_half_width);
  const double upper = clampCurvature(center + clamped_half_width);

  auto refined = makeUniformSamples(lower, upper, n);

  if (refined.size() >= 2U) {
    const double spacing = (refined.back() - refined.front()) /
      static_cast<double>(refined.size() - 1U);
    if (spacing > 0.05 + 1e-12) {
      const int adjusted_n = static_cast<int>(std::ceil((upper - lower) / 0.05)) + 1;
      refined = makeUniformSamples(lower, upper, std::max(n, adjusted_n));
    }
  }

  return refined;
}

double CurvatureSampler::maxCurvature() const
{
  return max_curvature_;
}

double CurvatureSampler::clampCurvature(double kappa) const
{
  return std::clamp(kappa, -max_curvature_, max_curvature_);
}

}  // namespace mycar_navigation::nav_core
