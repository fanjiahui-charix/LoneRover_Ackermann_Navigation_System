#ifndef MYCAR_NAVIGATION_PLANNER_SPEED_LIMITER_HPP_
#define MYCAR_NAVIGATION_PLANNER_SPEED_LIMITER_HPP_

#include "mycar_navigation/planner/planner_types.hpp"

namespace mycar_navigation::planner
{

double curvatureSpeedLimit(double kappa, double a_lat_max, double max_speed);
double stopDistance(double v, double a_brake, double t_latency, double margin);
double brakeLimitedSpeed(
  double free_length, double a_brake, double t_latency, double margin, double max_speed);
double approachSpeed(
  double distance_to_goal, double approach_distance, double max_speed, double min_speed);
double accelClamp(double desired_v, double prev_v, double a_accel, double dt);
double arcSpeed(
  double kappa, double free_length, double distance_to_goal, double prev_v,
  const PlannerConfig & cfg);

}  // namespace mycar_navigation::planner

#endif  // MYCAR_NAVIGATION_PLANNER_SPEED_LIMITER_HPP_
