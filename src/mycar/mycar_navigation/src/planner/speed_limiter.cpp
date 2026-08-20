#include "mycar_navigation/planner/speed_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mycar_navigation::planner
{
namespace
{
constexpr double kCurvatureEpsilon = 1e-9;
}  // namespace

double curvatureSpeedLimit(double kappa, double a_lat_max, double max_speed)
{
  if (!(a_lat_max > 0.0)) {
    throw std::invalid_argument("a_lat_max must be positive");
  }
  if (max_speed < 0.0) {
    throw std::invalid_argument("max_speed must be non-negative");
  }
  if (std::abs(kappa) <= kCurvatureEpsilon) {
    return max_speed;
  }
  return std::min(max_speed, std::sqrt(a_lat_max / std::abs(kappa)));
}

double stopDistance(double v, double a_brake, double t_latency, double margin)
{
  if (!(a_brake > 0.0)) {
    throw std::invalid_argument("a_brake must be positive");
  }
  if (t_latency < 0.0 || margin < 0.0) {
    throw std::invalid_argument("t_latency and margin must be non-negative");
  }
  const double speed = std::max(0.0, v);
  return speed * t_latency + (speed * speed) / (2.0 * a_brake) + margin;
}

double brakeLimitedSpeed(
  double free_length, double a_brake, double t_latency, double margin, double max_speed)
{
  if (free_length <= 0.0) {
    return 0.0;
  }
  if (!(a_brake > 0.0)) {
    throw std::invalid_argument("a_brake must be positive");
  }
  if (t_latency < 0.0 || margin < 0.0 || max_speed < 0.0) {
    throw std::invalid_argument("parameters must be non-negative");
  }

  const double available = free_length - margin;
  if (available <= 0.0) {
    return 0.0;
  }

  const double discriminant = a_brake * a_brake * t_latency * t_latency + 2.0 * a_brake * available;
  const double vmax = -a_brake * t_latency + std::sqrt(std::max(0.0, discriminant));
  return std::clamp(vmax, 0.0, max_speed);
}

double approachSpeed(
  double distance_to_goal, double approach_distance, double max_speed, double min_speed)
{
  if (max_speed < 0.0 || min_speed < 0.0) {
    throw std::invalid_argument("speed bounds must be non-negative");
  }
  if (approach_distance <= 0.0) {
    return max_speed;
  }
  if (distance_to_goal >= approach_distance) {
    return max_speed;
  }
  const double ratio = std::clamp(distance_to_goal / approach_distance, 0.0, 1.0);
  return ratio * max_speed + (1.0 - ratio) * min_speed;
}

double accelClamp(double desired_v, double prev_v, double a_accel, double dt)
{
  if (!(a_accel > 0.0) || !(dt > 0.0)) {
    throw std::invalid_argument("a_accel and dt must be positive");
  }
  const double delta_limit = a_accel * dt;
  return std::clamp(desired_v, prev_v - delta_limit, prev_v + delta_limit);
}

double arcSpeed(
  double kappa, double free_length, double distance_to_goal, double prev_v,
  const PlannerConfig & cfg)
{
  const double speed_limit = std::min({
    cfg.max_speed,
    curvatureSpeedLimit(kappa, cfg.a_lat_max, cfg.max_speed),
    brakeLimitedSpeed(free_length, cfg.a_brake, cfg.t_latency, cfg.brake_margin, cfg.max_speed),
    approachSpeed(distance_to_goal, cfg.goal_approach_distance, cfg.max_speed, cfg.min_speed)});

  const double clamped = accelClamp(speed_limit, std::max(0.0, prev_v), cfg.a_accel, cfg.dt);
  return std::clamp(clamped, 0.0, cfg.max_speed);
}

}  // namespace mycar_navigation::planner
