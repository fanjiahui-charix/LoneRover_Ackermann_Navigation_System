#ifndef HOBOT_NAV__GATE_REGION_REACHABILITY_HPP_
#define HOBOT_NAV__GATE_REGION_REACHABILITY_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"

namespace hobot_nav
{

struct GatePose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// Frozen map-frame pose transformed into the local-costmap frame as
// p_local = R(yaw) * p_map + translation.
struct GateTransform2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct GateVehicleGeometry
{
  double wheelbase_m{0.144};
  double minimum_turning_radius_m{0.35};
  double footprint_front_m{0.28};
  double footprint_rear_m{0.11};
  double footprint_half_width_m{0.13};
  double footprint_padding_m{0.0};
  double right_mechanical_limit_rad{0.39};
};

struct GateEntryRegion
{
  double x_min{2.30};
  double x_max{2.70};
  double y_min{2.25};
  double y_max{2.40};
  double yaw_min_rad{1.3962634015954636};
  double yaw_max_rad{1.7453292519943295};
  double terminal_steering_abs_max_rad{0.0523598775598299};
};

struct GateSearchSnapshot
{
  nav2_msgs::msg::Costmap::ConstSharedPtr global_costmap;
  nav2_msgs::msg::Costmap::ConstSharedPtr local_costmap;
  GateTransform2D map_to_local;
  std::uint64_t map_pose_epoch{0};
  std::uint64_t steering_epoch{0};
};

struct GateSearchConfig
{
  double spatial_resolution_m{0.05};
  std::size_t yaw_bins{72};
  std::size_t steering_levels{5};
  double edge_length_m{0.05};
  double nominal_speed_mps{0.25};
  double steering_rate_radps{1.20};
  double roi_margin_m{0.45};
  // Bounded weighted best-first guidance. It changes expansion order only;
  // a query is REACHABLE solely after its exact lattice key is settled.
  double guidance_weight{3.0};
  double hard_budget_ms{800.0};
  std::size_t maximum_states_expanded{250000};
  // Raw state-only probes are cheap and are discretized/deduplicated before
  // search. This bound prevents accidental unbounded callers without forcing
  // the old nine-representative-query approximation.
  std::size_t maximum_queries{512};
  std::string map_frame{"map"};
  std::string local_frame{"odom"};
};

struct GateQuery
{
  GatePose2D pose;
  double steering_rad{0.0};
};

struct GateQueryResult
{
  enum class Status
  {
    REACHABLE,
    UNREACHABLE,
    UNKNOWN_TIMEOUT,
    OUTSIDE_ROI
  };

  Status status{Status::UNKNOWN_TIMEOUT};
  double cost_to_gate{0.0};
};

struct GateBuildResult
{
  GateQueryResult::Status status{GateQueryResult::Status::UNKNOWN_TIMEOUT};
  double elapsed_ms{0.0};
  std::size_t states_expanded{0};
  std::size_t terminal_states{0};
  std::size_t requested_queries{0};
  std::size_t unique_queries{0};
  std::size_t duplicate_queries{0};
  std::size_t outside_queries{0};
  std::size_t overflow_dropped_queries{0};
  std::size_t settled_queries{0};
  std::uint64_t build_sequence{0};
  std::string detail;
};

// One immutable, ROI-bounded multi-source reverse search. Reachability is
// necessary and sufficient only in this frozen finite lattice. UNKNOWN_TIMEOUT
// is deliberately distinct from UNREACHABLE.
class GateRegionReachability
{
public:
  GateBuildResult build(
    const GateSearchSnapshot & snapshot,
    const GateVehicleGeometry & geometry,
    const GateEntryRegion & gate_region,
    const std::vector<GateQuery> & queries,
    const GateSearchConfig & config = GateSearchConfig{});

  GateQueryResult query(const GateQuery & query) const;
  nav_msgs::msg::Path reconstructConnector(const GateQuery & query) const;
  std::uint64_t buildSequence() const noexcept {return build_sequence_;}

private:
  struct StateKey
  {
    int ix{0};
    int iy{0};
    int iyaw{0};
    int isteer{0};
  };

  struct NodeRecord
  {
    double cost{0.0};
    std::uint64_t next_key{0};
    bool has_next{false};
    bool settled{false};
  };

  std::uint64_t pack(const StateKey & key) const;
  bool unpack(std::uint64_t packed, StateKey & key) const;
  bool discretize(const GateQuery & query, StateKey & key) const;
  GatePose2D statePose(const StateKey & key) const;
  double steeringAt(int level) const;
  bool footprintFree(const GatePose2D & pose) const;
  bool footprintFreeState(const StateKey & key) const;
  bool footprintFreeInGrid(
    const GatePose2D & pose, const nav2_msgs::msg::Costmap & grid,
    bool outside_is_collision) const;
  bool primitiveFree(const GatePose2D & from, const GatePose2D & to) const;
  static double normalize(double angle);
  static bool angleInInterval(double angle, double minimum, double maximum);

  GateSearchSnapshot snapshot_;
  GateVehicleGeometry geometry_;
  GateEntryRegion gate_region_;
  GateSearchConfig config_;
  double delta_max_{0.0};
  double roi_x_min_{0.0};
  double roi_y_min_{0.0};
  int nx_{0};
  int ny_{0};
  bool built_{false};
  bool timed_out_{false};
  bool exhausted_{false};
  std::uint64_t build_sequence_{0};
  std::unordered_map<std::uint64_t, NodeRecord> records_;
  std::vector<double> query_distance_2d_;
  std::vector<std::uint8_t> fused_collision_mask_;
  std::vector<std::vector<std::pair<int, int>>> footprint_offsets_by_yaw_;
  mutable std::unordered_map<std::uint64_t, bool> state_collision_cache_;
};

const char * gateQueryStatusName(GateQueryResult::Status status);

}  // namespace hobot_nav

#endif  // HOBOT_NAV__GATE_REGION_REACHABILITY_HPP_
