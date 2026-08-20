#include "simple_lidar_odom/landmark_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace simple_lidar_odom
{
namespace
{

constexpr double kTwoPi = 2.0 * M_PI;

Point2 polarToPoint(double range, double angle)
{
  return Point2(range * std::cos(angle), range * std::sin(angle));
}

double pointLineDistance(const Point2 & p, const LineLandmark & line)
{
  return std::abs(line.normal.dot(p) - line.d);
}

bool fitLinePca(const PointList & pts, LineLandmark & line)
{
  if (pts.size() < 2) {
    return false;
  }

  Point2 centroid = Point2::Zero();
  for (const auto & p : pts) {
    centroid += p;
  }
  centroid /= static_cast<double>(pts.size());

  Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
  for (const auto & p : pts) {
    const Point2 d = p - centroid;
    cov += d * d.transpose();
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
  if (solver.info() != Eigen::Success) {
    return false;
  }

  Point2 normal = solver.eigenvectors().col(0);
  if (normal.norm() < 1.0e-9) {
    return false;
  }
  normal.normalize();

  if (normal.dot(-centroid) < 0.0) {
    normal = -normal;
  }

  line.normal = normal;
  line.centroid = centroid;
  line.d = normal.dot(centroid);
  line.support_points = pts;
  return true;
}

double principalSpan(const PointList & pts, double * minor_variance = nullptr)
{
  if (pts.empty()) {
    if (minor_variance != nullptr) {
      *minor_variance = 0.0;
    }
    return 0.0;
  }

  Point2 centroid = Point2::Zero();
  for (const auto & p : pts) {
    centroid += p;
  }
  centroid /= static_cast<double>(pts.size());

  Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
  for (const auto & p : pts) {
    const Point2 d = p - centroid;
    cov += d * d.transpose();
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
  if (solver.info() != Eigen::Success) {
    if (minor_variance != nullptr) {
      *minor_variance = 0.0;
    }
    return 0.0;
  }

  const auto eigenvalues = solver.eigenvalues();
  if (minor_variance != nullptr) {
    *minor_variance = std::max(0.0, eigenvalues(0));
  }
  return 2.0 * std::sqrt(std::max(0.0, eigenvalues(1)));
}

void splitFenceSegmentRecursive(
  const PointList & pts,
  double residual_threshold,
  int min_points,
  int max_depth,
  int depth,
  std::vector<Segment> & output)
{
  if (pts.empty()) {
    return;
  }
  if (residual_threshold <= 0.0 || max_depth <= 0 || depth >= max_depth ||
    static_cast<int>(pts.size()) < 2 * min_points)
  {
    output.push_back(Segment{pts});
    return;
  }

  const Point2 a = pts.front();
  const Point2 direction = pts.back() - a;
  const double length = direction.norm();
  if (length < 1.0e-9 || pts.size() <= 2U) {
    output.push_back(Segment{pts});
    return;
  }

  std::size_t split_index = 0U;
  double max_residual = 0.0;
  for (std::size_t i = 1U; i + 1U < pts.size(); ++i) {
    const Point2 d = pts[i] - a;
    const double residual =
      std::abs(d.x() * direction.y() - d.y() * direction.x()) / length;
    if (residual > max_residual) {
      max_residual = residual;
      split_index = i;
    }
  }

  const bool valid_split = max_residual > residual_threshold &&
    split_index + 1U >= static_cast<std::size_t>(min_points) &&
    pts.size() - split_index >= static_cast<std::size_t>(min_points);
  if (!valid_split) {
    output.push_back(Segment{pts});
    return;
  }

  PointList left(pts.begin(), pts.begin() + static_cast<std::ptrdiff_t>(split_index + 1U));
  PointList right(pts.begin() + static_cast<std::ptrdiff_t>(split_index), pts.end());
  splitFenceSegmentRecursive(
    left, residual_threshold, min_points, max_depth, depth + 1, output);
  splitFenceSegmentRecursive(
    right, residual_threshold, min_points, max_depth, depth + 1, output);
}

}  // namespace

std::vector<Segment> segmentScan(
  const sensor_msgs::msg::LaserScan & scan,
  double min_range,
  double max_range,
  double seg_break_dist,
  const PointList * points)
{
  std::vector<Segment> segments;
  Segment current;
  bool have_prev_valid = false;
  std::size_t prev_idx = 0;
  Point2 prev_point = Point2::Zero();

  // Use caller-supplied (deskewed) coordinates only when there is exactly one
  // per ray; otherwise fall back to polar projection of the raw ranges.
  const bool use_points = points != nullptr && points->size() == scan.ranges.size();

  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const float r = scan.ranges[i];
    const bool valid = std::isfinite(r) && r >= min_range && r <= max_range;
    if (!valid) {
      if (!current.pts.empty()) {
        segments.push_back(std::move(current));
        current = Segment{};
      }
      have_prev_valid = false;
      continue;
    }

    Point2 p;
    if (use_points) {
      p = (*points)[i];
    } else {
      const double angle = static_cast<double>(scan.angle_min) +
        static_cast<double>(i) * static_cast<double>(scan.angle_increment);
      p = polarToPoint(static_cast<double>(r), angle);
    }

    bool break_segment = false;
    if (have_prev_valid) {
      if (i != prev_idx + 1U) {
        break_segment = true;
      } else if ((p - prev_point).norm() > seg_break_dist) {
        break_segment = true;
      }
    }

    if (break_segment && !current.pts.empty()) {
      segments.push_back(std::move(current));
      current = Segment{};
    }

    current.pts.push_back(p);
    have_prev_valid = true;
    prev_idx = i;
    prev_point = p;
  }

  if (!current.pts.empty()) {
    segments.push_back(std::move(current));
  }

  const double span = static_cast<double>(scan.angle_max - scan.angle_min);
  if (segments.size() > 1 && std::abs(span - kTwoPi) < 0.2) {
    const Point2 & first = segments.front().pts.front();
    const Point2 & last = segments.back().pts.back();
    if ((first - last).norm() <= seg_break_dist) {
      Segment merged;
      merged.pts = segments.back().pts;
      merged.pts.insert(merged.pts.end(), segments.front().pts.begin(), segments.front().pts.end());
      segments.erase(segments.begin());
      segments.back() = std::move(merged);
    }
  }

  return segments;
}

CircleFitResult fitCircleFixedR(
  const PointList & pts,
  double radius,
  double max_rms)
{
  CircleFitResult result;
  if (pts.size() < 3 || radius <= 0.0) {
    return result;
  }

  Point2 centroid = Point2::Zero();
  for (const auto & p : pts) {
    centroid += p;
  }
  centroid /= static_cast<double>(pts.size());

  Point2 dir = centroid;
  if (dir.norm() < 1.0e-9) {
    dir = Point2::UnitX();
  } else {
    dir.normalize();
  }

  Point2 center = centroid + radius * dir;

  for (int iter = 0; iter < 5; ++iter) {
    Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
    Eigen::Vector2d b = Eigen::Vector2d::Zero();

    for (const auto & p : pts) {
      const Point2 diff = center - p;
      const double dist = std::max(diff.norm(), 1.0e-9);
      const double residual = dist - radius;
      const Point2 J = diff / dist;
      H += J * J.transpose();
      b += J * residual;
    }

    H += Eigen::Matrix2d::Identity() * 1.0e-9;
    const Eigen::LDLT<Eigen::Matrix2d> ldlt(H);
    if (ldlt.info() != Eigen::Success) {
      return result;
    }

    const Eigen::Vector2d delta = -ldlt.solve(b);
    if (!delta.allFinite()) {
      return result;
    }

    center += delta;
    if (delta.norm() < 1.0e-5) {
      break;
    }
  }

  double ssr = 0.0;
  for (const auto & p : pts) {
    const double residual = (p - center).norm() - radius;
    ssr += residual * residual;
  }

  result.center = center;
  result.radius = radius;
  result.rms_error = std::sqrt(ssr / static_cast<double>(pts.size()));
  result.valid = std::isfinite(result.rms_error) && result.rms_error <= max_rms;
  return result;
}

CircleFitResult fitCircleRadiusRange(
  const PointList & pts,
  double radius_min,
  double radius_max,
  double radius_step,
  double max_rms)
{
  if (radius_min > radius_max) {
    std::swap(radius_min, radius_max);
  }
  radius_step = std::max(radius_step, 0.001);

  CircleFitResult best;
  best.rms_error = std::numeric_limits<double>::infinity();
  for (double radius = radius_min; radius <= radius_max + 1.0e-9; radius += radius_step) {
    CircleFitResult fit = fitCircleFixedR(pts, radius, max_rms);
    if (fit.rms_error < best.rms_error) {
      best = fit;
    }
  }
  return best;
}

std::vector<LineLandmark> extractFenceLines(
  const std::vector<Segment> & fence_segments,
  double merge_angle_tol,
  double merge_dist_tol)
{
  std::vector<LineLandmark> lines;
  lines.reserve(fence_segments.size());

  for (const auto & seg : fence_segments) {
    LineLandmark line;
    if (fitLinePca(seg.pts, line)) {
      lines.push_back(std::move(line));
    }
  }

  bool merged = true;
  while (merged) {
    merged = false;
    for (std::size_t i = 0; i < lines.size() && !merged; ++i) {
      for (std::size_t j = i + 1; j < lines.size(); ++j) {
        const double dot = lines[i].normal.dot(lines[j].normal);
        if (dot <= 0.0) {
          continue;
        }

        const double sin_angle = std::abs(lines[i].normal.x() * lines[j].normal.y() -
          lines[i].normal.y() * lines[j].normal.x());
        if (sin_angle >= merge_angle_tol) {
          continue;
        }

        const double dist_ij = pointLineDistance(lines[i].centroid, lines[j]);
        const double dist_ji = pointLineDistance(lines[j].centroid, lines[i]);
        if (std::max(dist_ij, dist_ji) >= merge_dist_tol) {
          continue;
        }

        PointList pts = lines[i].support_points;
        pts.insert(pts.end(), lines[j].support_points.begin(), lines[j].support_points.end());

        LineLandmark merged_line;
        if (!fitLinePca(pts, merged_line)) {
          continue;
        }

        lines[i] = std::move(merged_line);
        lines.erase(lines.begin() + static_cast<std::ptrdiff_t>(j));
        merged = true;
        break;
      }
    }
  }

  return lines;
}

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
  const PointList * points)
{
  ExtractedLandmarks result;
  std::vector<Segment> fence_segments;

  const auto segments = segmentScan(scan, min_range, max_range, seg_break_dist, points);
  for (const auto & seg : segments) {
    const int point_count = static_cast<int>(seg.pts.size());
    const double span = principalSpan(seg.pts);

    const bool cone_size_candidate =
      point_count >= cone_min_pts && point_count <= cone_max_pts && span <= cone_max_span;
    if (cone_size_candidate) {
      const bool search_radius =
        cone_radius_min > 0.0 && cone_radius_max >= cone_radius_min;
      const CircleFitResult fit = search_radius ?
        fitCircleRadiusRange(seg.pts, cone_radius_min, cone_radius_max, 0.0025, cone_fit_max_rms) :
        fitCircleFixedR(seg.pts, cone_radius, cone_fit_max_rms);
      if (fit.valid) {
        result.cone_centers.push_back(fit.center);
        continue;
      }
      result.rejected_clusters.push_back(seg);
    }

    bool accepted_as_fence = false;
    if (point_count >= fence_min_pts) {
      std::vector<Segment> split_segments;
      splitFenceSegmentRecursive(
        seg.pts, fence_split_residual, fence_min_pts,
        fence_split_max_depth, 0, split_segments);
      for (const auto & fence_segment : split_segments) {
        if (static_cast<int>(fence_segment.pts.size()) < fence_min_pts) {
          continue;
        }
        double child_minor_variance = 0.0;
        const double child_span = principalSpan(fence_segment.pts, &child_minor_variance);
        const double child_major_variance = std::max(1.0e-9, 0.25 * child_span * child_span);
        const double child_aspect_ratio =
          child_major_variance / std::max(child_minor_variance, 1.0e-9);
        if (child_aspect_ratio >= fence_aspect_ratio) {
          fence_segments.push_back(fence_segment);
          accepted_as_fence = true;
        }
      }
    }
    if (!accepted_as_fence && !cone_size_candidate) {
      result.rejected_clusters.push_back(seg);
    }
  }

  result.fences = extractFenceLines(
    fence_segments, fence_merge_angle_tol, fence_merge_dist_tol);
  return result;
}

}  // namespace simple_lidar_odom
