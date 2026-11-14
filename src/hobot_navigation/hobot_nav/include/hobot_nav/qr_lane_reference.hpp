#ifndef HOBOT_NAV__QR_LANE_REFERENCE_HPP_
#define HOBOT_NAV__QR_LANE_REFERENCE_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace hobot_nav
{

struct QrLaneSample
{
  double s_m{0.0};
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double curvature_1pm{0.0};
  double normal_x{0.0};
  double normal_y{1.0};
};

struct QrLaneProjection
{
  double s_m{0.0};
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double signed_lateral_m{0.0};
  double distance_m{0.0};
  std::size_t segment_index{0U};
};

// Deliberately uses a bounded linear scan.  The shipped reference contains
// only 217 samples, so an index/background worker would add overhead and
// lifecycle complexity without improving this stage's latency.
class QrLaneReference
{
public:
  bool loadCsv(const std::string & path, std::string * error = nullptr);
  bool empty() const noexcept {return samples_.size() < 2U;}
  const std::vector<QrLaneSample> & samples() const noexcept {return samples_;}
  std::optional<QrLaneSample> interpolateByS(double s_m) const;
  std::optional<QrLaneProjection> project(double x_m, double y_m) const;
  double distanceTo(double x_m, double y_m) const;

private:
  std::vector<QrLaneSample> samples_;
};

}  // namespace hobot_nav

#endif  // HOBOT_NAV__QR_LANE_REFERENCE_HPP_
