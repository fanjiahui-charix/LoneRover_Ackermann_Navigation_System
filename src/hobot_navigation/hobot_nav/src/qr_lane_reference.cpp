#include "hobot_nav/qr_lane_reference.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <utility>

namespace hobot_nav
{
namespace
{
double normalizeAngle(double angle) {return std::atan2(std::sin(angle), std::cos(angle));}
double lerpAngle(double a, double b, double u)
{
  return normalizeAngle(a + u * normalizeAngle(b - a));
}
}  // namespace

bool QrLaneReference::loadCsv(const std::string & path, std::string * error)
{
  std::ifstream stream(path);
  if (!stream) {
    if (error) {*error = "cannot open QR lane CSV: " + path;}
    return false;
  }
  std::string line;
  if (!std::getline(stream, line)) {
    if (error) {*error = "QR lane CSV is empty";}
    return false;
  }
  std::vector<QrLaneSample> loaded;
  while (std::getline(stream, line)) {
    if (line.empty()) {continue;}
    std::stringstream row(line);
    std::string field;
    std::vector<double> values;
    while (std::getline(row, field, ',')) {
      try {
        values.push_back(std::stod(field));
      } catch (const std::exception &) {
        if (error) {*error = "invalid numeric QR lane row: " + line;}
        return false;
      }
    }
    if (values.size() != 8U) {
      if (error) {*error = "QR lane row must contain 8 columns: " + line;}
      return false;
    }
    loaded.push_back(QrLaneSample{
      values[0], values[1], values[2], values[3], values[5], values[6], values[7]});
  }
  if (loaded.size() != 217U) {
    if (error) {*error = "QR lane CSV must contain exactly 217 samples";}
    return false;
  }
  for (std::size_t i = 1U; i < loaded.size(); ++i) {
    if (!(loaded[i].s_m > loaded[i - 1U].s_m)) {
      if (error) {*error = "QR lane s_m must be strictly increasing";}
      return false;
    }
  }
  samples_ = std::move(loaded);
  return true;
}

std::optional<QrLaneSample> QrLaneReference::interpolateByS(double s_m) const
{
  if (empty()) {return std::nullopt;}
  s_m = std::clamp(s_m, samples_.front().s_m, samples_.back().s_m);
  const auto upper = std::lower_bound(
    samples_.begin(), samples_.end(), s_m,
    [](const QrLaneSample & sample, double s) {return sample.s_m < s;});
  if (upper == samples_.begin()) {return *upper;}
  if (upper == samples_.end()) {return samples_.back();}
  const auto & b = *upper;
  const auto & a = *(upper - 1);
  const double u = std::clamp((s_m - a.s_m) / std::max(1.0e-12, b.s_m - a.s_m), 0.0, 1.0);
  const double yaw = lerpAngle(a.yaw_rad, b.yaw_rad, u);
  return QrLaneSample{
    s_m, a.x_m + u * (b.x_m - a.x_m), a.y_m + u * (b.y_m - a.y_m), yaw,
    a.curvature_1pm + u * (b.curvature_1pm - a.curvature_1pm),
    -std::sin(yaw), std::cos(yaw)};
}

std::optional<QrLaneProjection> QrLaneReference::project(double x_m, double y_m) const
{
  if (empty()) {return std::nullopt;}
  double best_d2 = std::numeric_limits<double>::infinity();
  QrLaneProjection best;
  for (std::size_t i = 0U; i + 1U < samples_.size(); ++i) {
    const auto & a = samples_[i];
    const auto & b = samples_[i + 1U];
    const double vx = b.x_m - a.x_m;
    const double vy = b.y_m - a.y_m;
    const double vv = vx * vx + vy * vy;
    const double u = vv > 1.0e-12 ? std::clamp(
      ((x_m - a.x_m) * vx + (y_m - a.y_m) * vy) / vv, 0.0, 1.0) : 0.0;
    const double px = a.x_m + u * vx;
    const double py = a.y_m + u * vy;
    const double dx = x_m - px;
    const double dy = y_m - py;
    const double d2 = dx * dx + dy * dy;
    if (d2 >= best_d2) {continue;}
    const double yaw = lerpAngle(a.yaw_rad, b.yaw_rad, u);
    best_d2 = d2;
    best = QrLaneProjection{
      a.s_m + u * (b.s_m - a.s_m), px, py, yaw,
      dx * -std::sin(yaw) + dy * std::cos(yaw), std::sqrt(d2), i};
  }
  return best;
}

double QrLaneReference::distanceTo(double x_m, double y_m) const
{
  const auto projection = project(x_m, y_m);
  return projection ? projection->distance_m : std::numeric_limits<double>::infinity();
}

}  // namespace hobot_nav
