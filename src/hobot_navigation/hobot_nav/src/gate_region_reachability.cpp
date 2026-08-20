#include "hobot_nav/gate_region_reachability.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>
#include <unordered_set>
#include <utility>

#include "hobot_nav/cost_semantics.hpp"

namespace hobot_nav
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
double clampFinite(double value, double minimum, double maximum)
{
  return std::max(minimum, std::min(maximum, value));
}
}  // namespace

const char * gateQueryStatusName(GateQueryResult::Status status)
{
  switch (status) {
    case GateQueryResult::Status::REACHABLE: return "REACHABLE";
    case GateQueryResult::Status::UNREACHABLE: return "UNREACHABLE";
    case GateQueryResult::Status::UNKNOWN_TIMEOUT: return "UNKNOWN_TIMEOUT";
    case GateQueryResult::Status::OUTSIDE_ROI: return "OUTSIDE_ROI";
  }
  return "UNKNOWN_TIMEOUT";
}

double GateRegionReachability::normalize(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool GateRegionReachability::angleInInterval(
  double angle, double minimum, double maximum)
{
  angle = normalize(angle);
  minimum = normalize(minimum);
  maximum = normalize(maximum);
  if (minimum <= maximum) {
    return angle >= minimum - 1.0e-9 && angle <= maximum + 1.0e-9;
  }
  return angle >= minimum - 1.0e-9 || angle <= maximum + 1.0e-9;
}

std::uint64_t GateRegionReachability::pack(const StateKey & key) const
{
  return (((static_cast<std::uint64_t>(key.isteer) * config_.yaw_bins +
    static_cast<std::uint64_t>(key.iyaw)) * static_cast<std::uint64_t>(ny_) +
    static_cast<std::uint64_t>(key.iy)) * static_cast<std::uint64_t>(nx_)) +
    static_cast<std::uint64_t>(key.ix);
}

bool GateRegionReachability::unpack(std::uint64_t packed, StateKey & key) const
{
  if (nx_ <= 0 || ny_ <= 0 || config_.yaw_bins == 0U ||
    config_.steering_levels == 0U)
  {
    return false;
  }
  key.ix = static_cast<int>(packed % static_cast<std::uint64_t>(nx_));
  packed /= static_cast<std::uint64_t>(nx_);
  key.iy = static_cast<int>(packed % static_cast<std::uint64_t>(ny_));
  packed /= static_cast<std::uint64_t>(ny_);
  key.iyaw = static_cast<int>(packed % config_.yaw_bins);
  packed /= config_.yaw_bins;
  key.isteer = static_cast<int>(packed);
  return key.ix >= 0 && key.ix < nx_ && key.iy >= 0 && key.iy < ny_ &&
         key.iyaw >= 0 && key.iyaw < static_cast<int>(config_.yaw_bins) &&
         key.isteer >= 0 && key.isteer < static_cast<int>(config_.steering_levels);
}

bool GateRegionReachability::discretize(
  const GateQuery & query, StateKey & key) const
{
  if (!std::isfinite(query.pose.x) || !std::isfinite(query.pose.y) ||
    !std::isfinite(query.pose.yaw) || !std::isfinite(query.steering_rad))
  {
    return false;
  }
  if (snapshot_.global_costmap) {
    const auto & metadata = snapshot_.global_costmap->metadata;
    const double x_min = metadata.origin.position.x;
    const double y_min = metadata.origin.position.y;
    const double x_max = x_min + metadata.size_x * metadata.resolution;
    const double y_max = y_min + metadata.size_y * metadata.resolution;
    if (query.pose.x < x_min || query.pose.x >= x_max ||
      query.pose.y < y_min || query.pose.y >= y_max)
    {
      return false;
    }
  }
  key.ix = static_cast<int>(std::llround(
    (query.pose.x - roi_x_min_) / config_.spatial_resolution_m));
  key.iy = static_cast<int>(std::llround(
    (query.pose.y - roi_y_min_) / config_.spatial_resolution_m));
  const double yaw_unit = 2.0 * kPi / static_cast<double>(config_.yaw_bins);
  key.iyaw = static_cast<int>(std::llround(
    (normalize(query.pose.yaw) + kPi) / yaw_unit)) %
    static_cast<int>(config_.yaw_bins);
  if (key.iyaw < 0) {key.iyaw += static_cast<int>(config_.yaw_bins);}
  const double steering_unit = 2.0 * delta_max_ /
    static_cast<double>(config_.steering_levels - 1U);
  key.isteer = static_cast<int>(std::llround(
    (clampFinite(query.steering_rad, -delta_max_, delta_max_) + delta_max_) /
    steering_unit));
  key.isteer = std::clamp(
    key.isteer, 0, static_cast<int>(config_.steering_levels) - 1);
  return key.ix >= 0 && key.ix < nx_ && key.iy >= 0 && key.iy < ny_;
}

GatePose2D GateRegionReachability::statePose(const StateKey & key) const
{
  const double yaw_unit = 2.0 * kPi / static_cast<double>(config_.yaw_bins);
  return GatePose2D{
    roi_x_min_ + static_cast<double>(key.ix) * config_.spatial_resolution_m,
    roi_y_min_ + static_cast<double>(key.iy) * config_.spatial_resolution_m,
    normalize(-kPi + static_cast<double>(key.iyaw) * yaw_unit)};
}

double GateRegionReachability::steeringAt(int level) const
{
  return -delta_max_ + 2.0 * delta_max_ * static_cast<double>(level) /
    static_cast<double>(config_.steering_levels - 1U);
}

bool GateRegionReachability::footprintFreeInGrid(
  const GatePose2D & pose, const nav2_msgs::msg::Costmap & grid,
  bool outside_is_collision) const
{
  if (grid.metadata.resolution <= 0.0F || grid.metadata.size_x == 0U ||
    grid.metadata.size_y == 0U || grid.data.empty())
  {
    return false;
  }
  const double front = geometry_.footprint_front_m + geometry_.footprint_padding_m;
  const double rear = geometry_.footprint_rear_m + geometry_.footprint_padding_m;
  const double half = geometry_.footprint_half_width_m + geometry_.footprint_padding_m;
  // This is the complete footprint in the frozen 5 cm search model. The final
  // connector is independently swept against native-resolution costmaps.
  const double step = std::max(0.02, 0.5 * config_.spatial_resolution_m);
  const int nx = std::max(1, static_cast<int>(std::ceil((front + rear) / step)));
  const int ny = std::max(1, static_cast<int>(std::ceil(2.0 * half / step)));
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  bool any_inside = false;
  for (int ix = 0; ix <= nx; ++ix) {
    const double local_x = -rear + (front + rear) * static_cast<double>(ix) / nx;
    for (int iy = 0; iy <= ny; ++iy) {
      const double local_y = -half + 2.0 * half * static_cast<double>(iy) / ny;
      const double x = pose.x + c * local_x - s * local_y;
      const double y = pose.y + s * local_x + c * local_y;
      const int mx = static_cast<int>(std::floor(
        (x - grid.metadata.origin.position.x) / grid.metadata.resolution));
      const int my = static_cast<int>(std::floor(
        (y - grid.metadata.origin.position.y) / grid.metadata.resolution));
      if (mx < 0 || my < 0 || mx >= static_cast<int>(grid.metadata.size_x) ||
        my >= static_cast<int>(grid.metadata.size_y))
      {
        if (outside_is_collision) {return false;}
        continue;
      }
      any_inside = true;
      const auto index = static_cast<std::size_t>(my) * grid.metadata.size_x +
        static_cast<std::size_t>(mx);
      if (index >= grid.data.size() || isHardObstacle(grid.data[index])) {
        return false;
      }
    }
  }
  (void)any_inside;
  return true;
}

bool GateRegionReachability::footprintFree(const GatePose2D & map_pose) const
{
  StateKey key;
  if (!discretize(GateQuery{map_pose, 0.0}, key)) {return false;}
  return footprintFreeState(key);
}

bool GateRegionReachability::footprintFreeState(const StateKey & key) const
{
  if (key.iyaw < 0 ||
    key.iyaw >= static_cast<int>(footprint_offsets_by_yaw_.size()) ||
    fused_collision_mask_.size() !=
    static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_))
  {
    return false;
  }
  for (const auto & offset : footprint_offsets_by_yaw_[key.iyaw]) {
    const int x = key.ix + offset.first;
    const int y = key.iy + offset.second;
    if (x < 0 || y < 0 || x >= nx_ || y >= ny_) {return false;}
    const auto index = static_cast<std::size_t>(y) * static_cast<std::size_t>(nx_) +
      static_cast<std::size_t>(x);
    if (fused_collision_mask_[index] != 0U) {return false;}
  }
  return true;
}

bool GateRegionReachability::primitiveFree(
  const GatePose2D & from, const GatePose2D & to) const
{
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double length = std::hypot(dx, dy);
  const int samples = std::max(
    1, static_cast<int>(std::ceil(length /
    std::max(0.02, 0.5 * config_.spatial_resolution_m))));
  const double dyaw = normalize(to.yaw - from.yaw);
  for (int i = 1; i < samples; ++i) {
    const double u = static_cast<double>(i) / samples;
    if (!footprintFree(GatePose2D{
        from.x + u * dx, from.y + u * dy, normalize(from.yaw + u * dyaw)}))
    {
      return false;
    }
  }
  return true;
}

GateBuildResult GateRegionReachability::build(
  const GateSearchSnapshot & snapshot,
  const GateVehicleGeometry & geometry,
  const GateEntryRegion & gate_region,
  const std::vector<GateQuery> & queries,
  const GateSearchConfig & config)
{
  using Clock = std::chrono::steady_clock;
  const auto started = Clock::now();
  GateBuildResult result;
  result.build_sequence = ++build_sequence_;
  result.requested_queries = queries.size();
  records_.clear();
  query_distance_2d_.clear();
  fused_collision_mask_.clear();
  footprint_offsets_by_yaw_.clear();
  state_collision_cache_.clear();
  built_ = false;
  timed_out_ = false;
  exhausted_ = false;
  snapshot_ = snapshot;
  geometry_ = geometry;
  gate_region_ = gate_region;
  config_ = config;
  // Reserve a small wall-clock tail for result assembly and audit publication
  // so the externally observed build call remains within the configured cap.
  const double budget_limit_ms = std::max(0.1, config_.hard_budget_ms - 5.0);

  const auto finish = [&](GateQueryResult::Status status, const std::string & detail) {
      result.status = status;
      result.detail = detail;
      result.elapsed_ms = std::chrono::duration<double, std::milli>(
        Clock::now() - started).count();
      return result;
    };
  if (queries.empty() || config_.maximum_queries == 0U)
  {
    return finish(
      GateQueryResult::Status::UNKNOWN_TIMEOUT,
      "Gate field exact-probe batch is empty or maximum_queries is zero");
  }
  if (!snapshot_.global_costmap || !snapshot_.local_costmap ||
    snapshot_.global_costmap->header.frame_id != config_.map_frame ||
    config_.spatial_resolution_m <= 0.0 || config_.yaw_bins < 8U ||
    config_.steering_levels < 3U || config_.steering_levels % 2U == 0U ||
    config_.edge_length_m <= 0.0 || config_.nominal_speed_mps <= 0.0 ||
    config_.steering_rate_radps <= 0.0 || config_.hard_budget_ms <= 0.0 ||
    config_.guidance_weight < 1.0 ||
    geometry_.wheelbase_m <= 0.0 || geometry_.minimum_turning_radius_m <= 0.0)
  {
    return finish(
      GateQueryResult::Status::UNKNOWN_TIMEOUT,
      "invalid or missing frozen search snapshot/configuration");
  }
  delta_max_ = std::min(
    std::atan(geometry_.wheelbase_m / geometry_.minimum_turning_radius_m),
    std::abs(geometry_.right_mechanical_limit_rad));
  if (delta_max_ <= 0.0) {
    return finish(GateQueryResult::Status::UNKNOWN_TIMEOUT, "invalid steering limit");
  }

  // Normal callers stay below 512. If a future caller violates that contract,
  // retain a deterministic uniform sample across the whole ordered batch so
  // late families are not starved. Omitted probes remain UNKNOWN unless the
  // search independently settles their lattice key.
  std::vector<const GateQuery *> bounded_queries;
  const std::size_t bounded_count = std::min(queries.size(), config_.maximum_queries);
  bounded_queries.reserve(bounded_count);
  if (queries.size() <= config_.maximum_queries) {
    for (const auto & query : queries) {bounded_queries.push_back(&query);}
  } else {
    result.overflow_dropped_queries = queries.size() - config_.maximum_queries;
    for (std::size_t slot = 0; slot < config_.maximum_queries; ++slot) {
      const std::size_t index = config_.maximum_queries == 1U ? 0U :
        slot * (queries.size() - 1U) / (config_.maximum_queries - 1U);
      bounded_queries.push_back(&queries[index]);
    }
  }

  const auto & global = *snapshot_.global_costmap;
  const double grid_x_min = global.metadata.origin.position.x;
  const double grid_y_min = global.metadata.origin.position.y;
  const double grid_x_max = grid_x_min + global.metadata.size_x * global.metadata.resolution;
  const double grid_y_max = grid_y_min + global.metadata.size_y * global.metadata.resolution;
  double x_min = gate_region_.x_min;
  double x_max = gate_region_.x_max;
  double y_min = gate_region_.y_min;
  double y_max = gate_region_.y_max;
  std::size_t finite_inside_centers = 0U;
  for (const auto * query : bounded_queries) {
    const bool finite = std::isfinite(query->pose.x) && std::isfinite(query->pose.y) &&
      std::isfinite(query->pose.yaw) && std::isfinite(query->steering_rad);
    const bool center_inside = finite && query->pose.x >= grid_x_min &&
      query->pose.x < grid_x_max && query->pose.y >= grid_y_min && query->pose.y < grid_y_max;
    if (!center_inside) {
      ++result.outside_queries;
      continue;
    }
    ++finite_inside_centers;
    x_min = std::min(x_min, query->pose.x);
    x_max = std::max(x_max, query->pose.x);
    y_min = std::min(y_min, query->pose.y);
    y_max = std::max(y_max, query->pose.y);
  }
  if (finite_inside_centers == 0U) {
    return finish(
      GateQueryResult::Status::OUTSIDE_ROI,
      "all exact Gate probes lie outside the frozen global map");
  }
  x_min -= config_.roi_margin_m;
  x_max += config_.roi_margin_m;
  y_min -= config_.roi_margin_m;
  y_max += config_.roi_margin_m;
  roi_x_min_ = std::max(x_min, grid_x_min);
  roi_y_min_ = std::max(y_min, grid_y_min);
  x_max = std::min(x_max, grid_x_max);
  y_max = std::min(y_max, grid_y_max);
  nx_ = static_cast<int>(std::floor((x_max - roi_x_min_) /
    config_.spatial_resolution_m)) + 1;
  ny_ = static_cast<int>(std::floor((y_max - roi_y_min_) /
    config_.spatial_resolution_m)) + 1;
  if (nx_ <= 1 || ny_ <= 1) {
    return finish(GateQueryResult::Status::OUTSIDE_ROI, "empty clipped Gate ROI");
  }
  if (snapshot_.local_costmap->header.frame_id != config_.local_frame &&
    snapshot_.local_costmap->header.frame_id != config_.map_frame)
  {
    return finish(
      GateQueryResult::Status::UNKNOWN_TIMEOUT,
      "frozen local costmap frame is neither map nor configured local frame");
  }
  built_ = true;
  std::unordered_set<std::uint64_t> pending_queries;
  std::size_t discretized_query_count = 0U;
  for (const auto * query_value : bounded_queries) {
    StateKey key;
    if (discretize(*query_value, key)) {
      pending_queries.insert(pack(key));
      ++discretized_query_count;
    }
  }
  result.unique_queries = pending_queries.size();
  result.duplicate_queries = discretized_query_count - result.unique_queries;
  if (pending_queries.empty()) {
    built_ = false;
    return finish(
      GateQueryResult::Status::OUTSIDE_ROI,
      "no exact Gate probe discretizes inside the clipped global ROI");
  }
  const auto preprocessing_budget_expired = [&]() {
      return std::chrono::duration<double, std::milli>(
        Clock::now() - started).count() >= budget_limit_ms;
    };
  const auto preprocessing_timeout = [&](const std::string & phase) {
      timed_out_ = true;
      return finish(
        GateQueryResult::Status::UNKNOWN_TIMEOUT,
        "bounded Gate field budget exhausted during " + phase);
    };

  // Fuse both frozen costmaps once into a cache-friendly ROI mask. Global map
  // bounds are hard; outside the rolling local map stays neutral, matching the
  // previous dual-map semantics. No TF or ROS costmap access occurs in the hot
  // four-dimensional expansion loop after this point.
  const auto occupied_at = [](const nav2_msgs::msg::Costmap & grid,
      double x, double y, bool outside_is_collision) {
      const int mx = static_cast<int>(std::floor(
        (x - grid.metadata.origin.position.x) / grid.metadata.resolution));
      const int my = static_cast<int>(std::floor(
        (y - grid.metadata.origin.position.y) / grid.metadata.resolution));
      if (mx < 0 || my < 0 || mx >= static_cast<int>(grid.metadata.size_x) ||
        my >= static_cast<int>(grid.metadata.size_y))
      {
        return outside_is_collision;
      }
      const auto index = static_cast<std::size_t>(my) * grid.metadata.size_x +
        static_cast<std::size_t>(mx);
      return index >= grid.data.size() || isHardObstacle(grid.data[index]);
    };
  fused_collision_mask_.assign(
    static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_), 1U);
  const double map_to_local_c = std::cos(snapshot_.map_to_local.yaw);
  const double map_to_local_s = std::sin(snapshot_.map_to_local.yaw);
  for (int iy = 0; iy < ny_; ++iy) {
    if (preprocessing_budget_expired()) {
      return preprocessing_timeout("frozen dual-map ROI fusion");
    }
    for (int ix = 0; ix < nx_; ++ix) {
      const double map_x = roi_x_min_ + static_cast<double>(ix) *
        config_.spatial_resolution_m;
      const double map_y = roi_y_min_ + static_cast<double>(iy) *
        config_.spatial_resolution_m;
      bool occupied = occupied_at(*snapshot_.global_costmap, map_x, map_y, true);
      double local_x = map_x;
      double local_y = map_y;
      if (snapshot_.local_costmap->header.frame_id == config_.local_frame) {
        local_x = snapshot_.map_to_local.x + map_to_local_c * map_x -
          map_to_local_s * map_y;
        local_y = snapshot_.map_to_local.y + map_to_local_s * map_x +
          map_to_local_c * map_y;
      }
      occupied = occupied || occupied_at(
        *snapshot_.local_costmap, local_x, local_y, false);
      fused_collision_mask_[static_cast<std::size_t>(iy) *
        static_cast<std::size_t>(nx_) + static_cast<std::size_t>(ix)] =
        occupied ? 1U : 0U;
    }
  }

  footprint_offsets_by_yaw_.resize(config_.yaw_bins);
  const double front = geometry_.footprint_front_m + geometry_.footprint_padding_m;
  const double rear = geometry_.footprint_rear_m + geometry_.footprint_padding_m;
  const double half_width = geometry_.footprint_half_width_m +
    geometry_.footprint_padding_m;
  const double cell_guard = std::sqrt(2.0) * 0.5 * config_.spatial_resolution_m;
  const int footprint_radius_cells = static_cast<int>(std::ceil(
    (std::hypot(std::max(front, rear), half_width) + cell_guard) /
    config_.spatial_resolution_m));
  for (std::size_t yaw_index = 0; yaw_index < config_.yaw_bins; ++yaw_index) {
    if (preprocessing_budget_expired()) {
      return preprocessing_timeout("footprint-offset preprocessing");
    }
    const StateKey yaw_key{0, 0, static_cast<int>(yaw_index), 0};
    const double yaw = statePose(yaw_key).yaw;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    auto & offsets = footprint_offsets_by_yaw_[yaw_index];
    for (int dy = -footprint_radius_cells; dy <= footprint_radius_cells; ++dy) {
      for (int dx = -footprint_radius_cells; dx <= footprint_radius_cells; ++dx) {
        const double world_x = dx * config_.spatial_resolution_m;
        const double world_y = dy * config_.spatial_resolution_m;
        const double local_x = c * world_x + s * world_y;
        const double local_y = -s * world_x + c * world_y;
        if (local_x >= -rear - cell_guard && local_x <= front + cell_guard &&
          std::abs(local_y) <= half_width + cell_guard)
        {
          offsets.emplace_back(dx, dy);
        }
      }
    }
  }
  // Queue stores (weighted guidance + g, g, key). Query settlement is an O(1)
  // hash erase; critically, no expanded state linearly scans the probe batch.
  using QueueItem = std::tuple<double, double, std::uint64_t>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;

  // Precompute a small 2-D multi-source distance field over the ROI. The
  // expensive four-dimensional search then gets O(1) guidance toward the
  // entire exact-probe set without scanning queries per expanded state. A
  // conservative scale and one-cell subtraction keep the lattice heuristic
  // below planar travel distance despite 8-neighbour discretization.
  const std::size_t planar_size = static_cast<std::size_t>(nx_) *
    static_cast<std::size_t>(ny_);
  query_distance_2d_.assign(planar_size, std::numeric_limits<double>::infinity());
  using PlanarItem = std::pair<double, std::size_t>;
  std::priority_queue<PlanarItem, std::vector<PlanarItem>, std::greater<PlanarItem>> planar_open;
  for (const auto packed_query : pending_queries) {
    StateKey query_key;
    if (!unpack(packed_query, query_key)) {continue;}
    const auto index = static_cast<std::size_t>(query_key.iy) *
      static_cast<std::size_t>(nx_) + static_cast<std::size_t>(query_key.ix);
    if (query_distance_2d_[index] > 0.0) {
      query_distance_2d_[index] = 0.0;
      planar_open.emplace(0.0, index);
    }
  }
  constexpr std::array<std::pair<int, int>, 8> kPlanarMoves{{
    {-1, -1}, {0, -1}, {1, -1}, {-1, 0},
    {1, 0}, {-1, 1}, {0, 1}, {1, 1}}};
  while (!planar_open.empty()) {
    if (preprocessing_budget_expired()) {
      return preprocessing_timeout("query-distance preprocessing");
    }
    const auto [distance_value, index] = planar_open.top();
    planar_open.pop();
    if (distance_value > query_distance_2d_[index] + 1.0e-12) {continue;}
    const int ix = static_cast<int>(index % static_cast<std::size_t>(nx_));
    const int iy = static_cast<int>(index / static_cast<std::size_t>(nx_));
    for (const auto & [dx, dy] : kPlanarMoves) {
      const int next_x = ix + dx;
      const int next_y = iy + dy;
      if (next_x < 0 || next_x >= nx_ || next_y < 0 || next_y >= ny_) {continue;}
      const auto next_index = static_cast<std::size_t>(next_y) *
        static_cast<std::size_t>(nx_) + static_cast<std::size_t>(next_x);
      const double step = config_.spatial_resolution_m *
        ((dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0);
      const double candidate = distance_value + step;
      if (candidate + 1.0e-12 < query_distance_2d_[next_index]) {
        query_distance_2d_[next_index] = candidate;
        planar_open.emplace(candidate, next_index);
      }
    }
  }
  std::vector<double> query_yaw_distance(
    config_.yaw_bins, std::numeric_limits<double>::infinity());
  std::vector<double> query_steering_distance(
    config_.steering_levels, std::numeric_limits<double>::infinity());
  for (const auto packed_query : pending_queries) {
    if (preprocessing_budget_expired()) {
      return preprocessing_timeout("yaw/steering guidance preprocessing");
    }
    StateKey query_key;
    if (!unpack(packed_query, query_key)) {continue;}
    const auto query_pose = statePose(query_key);
    const double query_steering = steeringAt(query_key.isteer);
    for (std::size_t yaw_index = 0; yaw_index < config_.yaw_bins; ++yaw_index) {
      const StateKey yaw_key{0, 0, static_cast<int>(yaw_index), 0};
      query_yaw_distance[yaw_index] = std::min(
        query_yaw_distance[yaw_index],
        std::abs(normalize(statePose(yaw_key).yaw - query_pose.yaw)));
    }
    for (std::size_t steering_index = 0;
      steering_index < config_.steering_levels; ++steering_index)
    {
      query_steering_distance[steering_index] = std::min(
        query_steering_distance[steering_index],
        std::abs(steeringAt(static_cast<int>(steering_index)) - query_steering));
    }
  }
  const auto query_heuristic = [this, &query_yaw_distance,
      &query_steering_distance](const StateKey & key) {
      const auto index = static_cast<std::size_t>(key.iy) *
        static_cast<std::size_t>(nx_) + static_cast<std::size_t>(key.ix);
      if (index >= query_distance_2d_.size() ||
        !std::isfinite(query_distance_2d_[index]))
      {
        return 0.0;
      }
      const double planar_lower_bound = 0.90 * std::max(
        0.0, query_distance_2d_[index] -
        std::sqrt(2.0) * config_.spatial_resolution_m);
      const double yaw_lower_bound = query_yaw_distance[static_cast<std::size_t>(key.iyaw)] *
        geometry_.minimum_turning_radius_m;
      const double steering_lower_bound =
        query_steering_distance[static_cast<std::size_t>(key.isteer)] /
        config_.steering_rate_radps * config_.nominal_speed_mps;
      return config_.guidance_weight * std::max({
        planar_lower_bound, yaw_lower_bound, steering_lower_bound});
    };
  const int neutral_steer = static_cast<int>(config_.steering_levels / 2U);
  for (int iy = 0; iy < ny_; ++iy) {
    if (preprocessing_budget_expired()) {
      return preprocessing_timeout("Gate-terminal preprocessing");
    }
    for (int ix = 0; ix < nx_; ++ix) {
      for (int iyaw = 0; iyaw < static_cast<int>(config_.yaw_bins); ++iyaw) {
        StateKey key{ix, iy, iyaw, neutral_steer};
        const auto pose = statePose(key);
        if (pose.x < gate_region_.x_min - 1.0e-9 ||
          pose.x > gate_region_.x_max + 1.0e-9 ||
          pose.y < gate_region_.y_min - 1.0e-9 ||
          pose.y > gate_region_.y_max + 1.0e-9 ||
          !angleInInterval(pose.yaw, gate_region_.yaw_min_rad, gate_region_.yaw_max_rad) ||
          std::abs(steeringAt(neutral_steer)) >
          gate_region_.terminal_steering_abs_max_rad + 1.0e-9)
        {
          continue;
        }
        const auto packed = pack(key);
        const bool free = footprintFreeState(key);
        const auto pose_packed = (((static_cast<std::uint64_t>(key.iyaw) *
          static_cast<std::uint64_t>(ny_)) + static_cast<std::uint64_t>(key.iy)) *
          static_cast<std::uint64_t>(nx_)) + static_cast<std::uint64_t>(key.ix);
        state_collision_cache_.emplace(pose_packed, free);
        if (!free) {continue;}
        records_.emplace(packed, NodeRecord{0.0, 0U, false, false});
        open.emplace(query_heuristic(key), 0.0, packed);
        ++result.terminal_states;
      }
    }
  }
  if (open.empty()) {
    exhausted_ = true;
    return finish(GateQueryResult::Status::UNREACHABLE, "no collision-free Gate terminals");
  }

  const double edge_time = config_.edge_length_m / config_.nominal_speed_mps;
  while (!open.empty()) {
    const double elapsed = std::chrono::duration<double, std::milli>(
      Clock::now() - started).count();
    if (elapsed >= budget_limit_ms ||
      result.states_expanded >= config_.maximum_states_expanded)
    {
      timed_out_ = true;
      break;
    }
    const auto [priority, cost, packed] = open.top();
    (void)priority;
    open.pop();
    auto record_it = records_.find(packed);
    if (record_it == records_.end() || record_it->second.settled ||
      cost > record_it->second.cost + 1.0e-12)
    {
      continue;
    }
    record_it->second.settled = true;
    ++result.states_expanded;
    if (pending_queries.erase(packed) > 0U) {
      ++result.settled_queries;
      if (pending_queries.empty()) {break;}
    }

    StateKey current_key;
    if (!unpack(packed, current_key)) {continue;}
    const auto current_pose = statePose(current_key);
    const double current_delta = steeringAt(current_key.isteer);
    const int first_level = std::max(0, current_key.isteer - 1);
    const int last_level = std::min(
      static_cast<int>(config_.steering_levels) - 1, current_key.isteer + 1);
    for (int previous_level = first_level; previous_level <= last_level; ++previous_level) {
      const double previous_delta = steeringAt(previous_level);
      if (std::abs(current_delta - previous_delta) >
        config_.steering_rate_radps * edge_time + 1.0e-9)
      {
        continue;
      }
      const double mean_delta = 0.5 * (current_delta + previous_delta);
      const double curvature = std::tan(mean_delta) / geometry_.wheelbase_m;
      if (std::abs(curvature) >
        1.0 / geometry_.minimum_turning_radius_m + 1.0e-3)
      {
        continue;
      }
      GatePose2D previous_pose;
      previous_pose.yaw = normalize(
        current_pose.yaw - config_.edge_length_m * curvature);
      const double mid_yaw = normalize(0.5 * (previous_pose.yaw + current_pose.yaw));
      previous_pose.x = current_pose.x - config_.edge_length_m * std::cos(mid_yaw);
      previous_pose.y = current_pose.y - config_.edge_length_m * std::sin(mid_yaw);
      StateKey previous_key;
      if (!discretize(GateQuery{previous_pose, previous_delta}, previous_key)) {continue;}
      const auto previous_packed = pack(previous_key);
      const auto pose_packed = (((static_cast<std::uint64_t>(previous_key.iyaw) *
        static_cast<std::uint64_t>(ny_)) + static_cast<std::uint64_t>(previous_key.iy)) *
        static_cast<std::uint64_t>(nx_)) + static_cast<std::uint64_t>(previous_key.ix);
      bool previous_free = false;
      const auto cache_it = state_collision_cache_.find(pose_packed);
      if (cache_it != state_collision_cache_.end()) {
        previous_free = cache_it->second;
      } else {
        previous_free = footprintFreeState(previous_key);
        state_collision_cache_.emplace(pose_packed, previous_free);
      }
      if (!previous_free || !primitiveFree(statePose(previous_key), current_pose)) {continue;}
      const double candidate_cost = cost + config_.edge_length_m +
        0.01 * std::abs(current_delta - previous_delta);
      auto [next_it, inserted] = records_.emplace(
        previous_packed, NodeRecord{candidate_cost, packed, true, false});
      if (!inserted && candidate_cost + 1.0e-12 < next_it->second.cost &&
        !next_it->second.settled)
      {
        next_it->second.cost = candidate_cost;
        next_it->second.next_key = packed;
        next_it->second.has_next = true;
      } else if (!inserted) {
        continue;
      }
      open.emplace(
        candidate_cost + query_heuristic(previous_key),
        candidate_cost, previous_packed);
    }
  }
  exhausted_ = open.empty() && !timed_out_;
  if (timed_out_) {
    return finish(
      GateQueryResult::Status::UNKNOWN_TIMEOUT,
      "bounded Gate field budget exhausted before all exact probes settled");
  }
  if (pending_queries.empty()) {
    return finish(
      GateQueryResult::Status::REACHABLE,
      "all in-map exact probes settled in one multi-source field");
  }
  return finish(
    GateQueryResult::Status::UNREACHABLE,
    "finite Gate field exhausted with unsettled exact probes");
}

GateQueryResult GateRegionReachability::query(const GateQuery & query_value) const
{
  if (!built_) {
    return GateQueryResult{GateQueryResult::Status::UNKNOWN_TIMEOUT, 0.0};
  }
  StateKey key;
  if (!discretize(query_value, key)) {
    return GateQueryResult{GateQueryResult::Status::OUTSIDE_ROI, 0.0};
  }
  const auto it = records_.find(pack(key));
  if (it != records_.end() && it->second.settled) {
    return GateQueryResult{GateQueryResult::Status::REACHABLE, it->second.cost};
  }
  if (timed_out_ || !exhausted_) {
    return GateQueryResult{GateQueryResult::Status::UNKNOWN_TIMEOUT, 0.0};
  }
  return GateQueryResult{GateQueryResult::Status::UNREACHABLE, 0.0};
}

nav_msgs::msg::Path GateRegionReachability::reconstructConnector(
  const GateQuery & query_value) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = config_.map_frame;
  StateKey key;
  if (!discretize(query_value, key)) {return path;}
  auto record_it = records_.find(pack(key));
  if (record_it == records_.end() || !record_it->second.settled) {return path;}
  const auto append = [&path](const GatePose2D & pose) {
      geometry_msgs::msg::PoseStamped stamped;
      stamped.header = path.header;
      stamped.pose.position.x = pose.x;
      stamped.pose.position.y = pose.y;
      stamped.pose.orientation.z = std::sin(0.5 * pose.yaw);
      stamped.pose.orientation.w = std::cos(0.5 * pose.yaw);
      path.poses.push_back(std::move(stamped));
    };
  append(query_value.pose);
  std::uint64_t packed = pack(key);
  for (std::size_t count = 0; count < 10000U; ++count) {
    record_it = records_.find(packed);
    if (record_it == records_.end() || !record_it->second.settled) {
      path.poses.clear();
      return path;
    }
    if (!record_it->second.has_next) {break;}
    packed = record_it->second.next_key;
    StateKey next;
    if (!unpack(packed, next)) {
      path.poses.clear();
      return path;
    }
    append(statePose(next));
  }
  if (path.poses.size() < 2U) {path.poses.clear();}
  return path;
}

}  // namespace hobot_nav
