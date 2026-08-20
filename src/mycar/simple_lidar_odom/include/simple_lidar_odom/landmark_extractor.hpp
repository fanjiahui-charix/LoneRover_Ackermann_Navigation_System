#pragma once

#include <vector>

#include <Eigen/Dense>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace simple_lidar_odom
{

using Point2 = Eigen::Vector2d;
using PointList = std::vector<Point2>;

struct Segment
{
  PointList pts;
};

struct CircleFitResult
{
  Point2 center{Point2::Zero()};
  double radius{0.0};
  double rms_error{0.0};
  bool valid{false};
};

struct LineLandmark
{
  Point2 normal{Point2::UnitX()};
  double d{0.0};
  Point2 centroid{Point2::Zero()};
  PointList support_points;
};

struct ExtractedLandmarks
{
  PointList cone_centers;
  std::vector<LineLandmark> fences;
  std::vector<Segment> rejected_clusters;
};

// When `points` is non-null it must have one entry per scan ray (same size as
// scan.ranges). The Cartesian coordinate of ray i is then taken from points[i]
// (e.g. a motion-deskewed point) instead of polar-projecting scan.ranges[i];
// validity, ray adjacency and the wrap-around span are still read from `scan`.
std::vector<Segment> segmentScan(
  const sensor_msgs::msg::LaserScan & scan,
  double min_range,
  double max_range,
  double seg_break_dist,
  const PointList * points = nullptr);

CircleFitResult fitCircleFixedR(
  const PointList & pts,
  double radius,
  double max_rms);

CircleFitResult fitCircleRadiusRange(
  const PointList & pts,
  double radius_min,
  double radius_max,
  double radius_step,
  double max_rms);

std::vector<LineLandmark> extractFenceLines(
  const std::vector<Segment> & fence_segments,
  double merge_angle_tol,
  double merge_dist_tol);

// `points`, when non-null and sized one-per-ray, supplies deskewed Cartesian
// coordinates used in place of polar-projecting scan.ranges (forwarded to
// segmentScan). Cone/fence fitting then runs on the motion-corrected points.
ExtractedLandmarks extractLandmarks(
  const sensor_msgs::msg::LaserScan & scan,
  double min_range,
  double max_range,
  double seg_break_dist,
  int cone_min_pts,
  int cone_max_pts,
  double cone_max_span,
  double cone_radius,
  double cone_radius_min,
  double cone_radius_max,
  double cone_fit_max_rms,
  int fence_min_pts,
  double fence_split_residual,
  int fence_split_max_depth,
  double fence_aspect_ratio,
  double fence_merge_angle_tol,
  double fence_merge_dist_tol,
  const PointList * points = nullptr);

}  // namespace simple_lidar_odom
