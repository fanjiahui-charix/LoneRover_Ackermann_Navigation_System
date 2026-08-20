#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "hobot_nav/gate_region_reachability.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

nav2_msgs::msg::Costmap::SharedPtr makeCostmap()
{
  auto map = std::make_shared<nav2_msgs::msg::Costmap>();
  map->header.frame_id = "map";
  map->metadata.resolution = 0.05F;
  map->metadata.size_x = 100U;
  map->metadata.size_y = 100U;
  map->metadata.origin.position.x = 0.0;
  map->metadata.origin.position.y = 0.0;
  map->data.assign(100U * 100U, 0U);
  return map;
}

void addVerticalWall(nav2_msgs::msg::Costmap & map, double x_min, double x_max)
{
  for (std::uint32_t y = 0; y < map.metadata.size_y; ++y) {
    for (std::uint32_t x = 0; x < map.metadata.size_x; ++x) {
      const double wx = map.metadata.origin.position.x +
        (static_cast<double>(x) + 0.5) * map.metadata.resolution;
      if (wx >= x_min && wx <= x_max) {
        map.data[static_cast<std::size_t>(y) * map.metadata.size_x + x] = 254U;
      }
    }
  }
}

hobot_nav::GateEntryRegion testGate()
{
  hobot_nav::GateEntryRegion gate;
  gate.x_min = 2.0;
  gate.x_max = 2.35;
  gate.y_min = 1.9;
  gate.y_max = 2.3;
  gate.yaw_min_rad = -5.0 * kPi / 180.0;
  gate.yaw_max_rad = 5.0 * kPi / 180.0;
  return gate;
}

hobot_nav::GateSearchSnapshot snapshot(
  const nav2_msgs::msg::Costmap::SharedPtr & global,
  const nav2_msgs::msg::Costmap::SharedPtr & local)
{
  hobot_nav::GateSearchSnapshot value;
  value.global_costmap = global;
  value.local_costmap = local;
  return value;
}
}  // namespace

TEST(GateRegionReachability, EmptyFieldIsReachableAndReconstructsForwardConnector)
{
  auto global = makeCostmap();
  auto local = makeCostmap();
  hobot_nav::GateRegionReachability field;
  const hobot_nav::GateQuery query{{1.25, 2.10, 0.0}, 0.0};
  auto config = hobot_nav::GateSearchConfig{};
  config.hard_budget_ms = 5000.0;  // QEMU execution budget; runtime default remains 80 ms.
  const auto build = field.build(
    snapshot(global, local), hobot_nav::GateVehicleGeometry{}, testGate(),
    std::vector<hobot_nav::GateQuery>{query}, config);
  EXPECT_EQ(build.status, hobot_nav::GateQueryResult::Status::REACHABLE)
    << build.detail;
  EXPECT_EQ(field.query(query).status, hobot_nav::GateQueryResult::Status::REACHABLE);
  const auto path = field.reconstructConnector(query);
  ASSERT_GE(path.poses.size(), 2U);
  double previous_delta = 0.0;
  bool have_previous_delta = false;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto & previous = path.poses[i - 1U].pose;
    const auto & current = path.poses[i].pose;
    const double yaw = 2.0 * std::atan2(
      previous.orientation.z, previous.orientation.w);
    const double projection = (current.position.x - previous.position.x) * std::cos(yaw) +
      (current.position.y - previous.position.y) * std::sin(yaw);
    EXPECT_GE(projection, -0.015);
    const double segment = std::hypot(
      current.position.x - previous.position.x,
      current.position.y - previous.position.y);
    if (segment > 1.0e-4 && i > 1U) {
      const double current_yaw = 2.0 * std::atan2(
        current.orientation.z, current.orientation.w);
      const double yaw_delta = std::atan2(
        std::sin(current_yaw - yaw), std::cos(current_yaw - yaw));
      const double curvature = yaw_delta / segment;
      EXPECT_LE(std::abs(curvature), 1.0 / 0.35 + 1.0e-3);
      const double delta = std::atan(0.144 * curvature);
      if (have_previous_delta) {
        EXPECT_LE(std::abs(delta - previous_delta),
          1.20 * segment / 0.20 + 1.0e-6);
      }
      previous_delta = delta;
      have_previous_delta = true;
    }
  }
}

TEST(GateRegionReachability, CompletelyBlockedGateIsUnreachable)
{
  auto global = makeCostmap();
  auto local = makeCostmap();
  addVerticalWall(*global, 1.65, 1.90);
  addVerticalWall(*local, 1.65, 1.90);
  hobot_nav::GateRegionReachability field;
  const hobot_nav::GateQuery query{{1.25, 2.10, 0.0}, 0.0};
  auto config = hobot_nav::GateSearchConfig{};
  config.hard_budget_ms = 5000.0;  // allow finite-graph exhaustion under QEMU
  const auto build = field.build(
    snapshot(global, local), hobot_nav::GateVehicleGeometry{}, testGate(),
    std::vector<hobot_nav::GateQuery>{query}, config);
  EXPECT_EQ(build.status, hobot_nav::GateQueryResult::Status::UNREACHABLE)
    << build.detail;
  EXPECT_EQ(field.query(query).status, hobot_nav::GateQueryResult::Status::UNREACHABLE);
}

TEST(GateRegionReachability, LocalOnlyDynamicWallAlsoRejects)
{
  auto global = makeCostmap();
  auto local = makeCostmap();
  local->header.frame_id = "odom";
  addVerticalWall(*local, 1.65, 1.90);
  hobot_nav::GateRegionReachability field;
  const hobot_nav::GateQuery query{{1.25, 2.10, 0.0}, 0.0};
  auto config = hobot_nav::GateSearchConfig{};
  config.hard_budget_ms = 5000.0;
  const auto build = field.build(
    snapshot(global, local), hobot_nav::GateVehicleGeometry{}, testGate(),
    std::vector<hobot_nav::GateQuery>{query}, config);
  EXPECT_EQ(build.status, hobot_nav::GateQueryResult::Status::UNREACHABLE)
    << build.detail;
}

TEST(GateRegionReachability, SideOfGateRemainsReachableWhenCenterIsBlocked)
{
  auto global = makeCostmap();
  auto local = makeCostmap();
  // Obstruct only the centerline. The terminal region still has legal side
  // states and the multi-source field must retain them.
  for (auto * map : {global.get(), local.get()}) {
    for (std::uint32_t y = 0; y < map->metadata.size_y; ++y) {
      for (std::uint32_t x = 0; x < map->metadata.size_x; ++x) {
        const double wx = (static_cast<double>(x) + 0.5) * map->metadata.resolution;
        const double wy = (static_cast<double>(y) + 0.5) * map->metadata.resolution;
        if (wx >= 2.05 && wx <= 2.20 && wy >= 2.05 && wy <= 2.15) {
          map->data[static_cast<std::size_t>(y) * map->metadata.size_x + x] = 254U;
        }
      }
    }
  }
  hobot_nav::GateRegionReachability field;
  const hobot_nav::GateQuery query{{1.25, 1.93, 0.0}, 0.0};
  auto config = hobot_nav::GateSearchConfig{};
  config.hard_budget_ms = 5000.0;
  const auto build = field.build(
    snapshot(global, local), hobot_nav::GateVehicleGeometry{}, testGate(),
    std::vector<hobot_nav::GateQuery>{query}, config);
  EXPECT_EQ(build.status, hobot_nav::GateQueryResult::Status::REACHABLE)
    << build.detail;
}

TEST(GateRegionReachability, TimeoutIsNeverReportedAsUnreachable)
{
  auto global = makeCostmap();
  auto local = makeCostmap();
  hobot_nav::GateRegionReachability field;
  const hobot_nav::GateQuery query{{1.25, 2.10, 0.0}, 0.0};
  auto config = hobot_nav::GateSearchConfig{};
  config.hard_budget_ms = 1.0e-9;
  const auto build = field.build(
    snapshot(global, local), hobot_nav::GateVehicleGeometry{}, testGate(),
    std::vector<hobot_nav::GateQuery>{query}, config);
  EXPECT_EQ(build.status, hobot_nav::GateQueryResult::Status::UNKNOWN_TIMEOUT);
  EXPECT_EQ(field.query(query).status,
    hobot_nav::GateQueryResult::Status::UNKNOWN_TIMEOUT);
}

TEST(GateRegionReachability, NineQueriesShareExactlyOneFieldBuild)
{
  auto global = makeCostmap();
  auto local = makeCostmap();
  std::vector<hobot_nav::GateQuery> queries;
  for (int i = 0; i < 9; ++i) {
    queries.push_back(hobot_nav::GateQuery{{1.25 + 0.02 * i, 2.10, 0.0}, 0.0});
  }
  hobot_nav::GateRegionReachability field;
  auto config = hobot_nav::GateSearchConfig{};
  config.hard_budget_ms = 5000.0;
  const auto before = field.buildSequence();
  const auto build = field.build(
    snapshot(global, local), hobot_nav::GateVehicleGeometry{}, testGate(),
    queries, config);
  EXPECT_EQ(field.buildSequence(), before + 1U);
  EXPECT_EQ(build.status, hobot_nav::GateQueryResult::Status::REACHABLE)
    << build.detail;
  EXPECT_EQ(build.requested_queries, 9U);
  EXPECT_EQ(build.build_sequence, before + 1U);
}

TEST(GateRegionReachability, OutsideProbeDoesNotPoisonExactBatch)
{
  auto global = makeCostmap();
  auto local = makeCostmap();
  const hobot_nav::GateQuery inside{{1.25, 2.10, 0.0}, 0.0};
  const hobot_nav::GateQuery outside{{-0.25, 2.10, 0.0}, 0.0};
  hobot_nav::GateRegionReachability field;
  auto config = hobot_nav::GateSearchConfig{};
  config.hard_budget_ms = 5000.0;
  const auto build = field.build(
    snapshot(global, local), hobot_nav::GateVehicleGeometry{}, testGate(),
    std::vector<hobot_nav::GateQuery>{inside, outside}, config);
  EXPECT_EQ(build.status, hobot_nav::GateQueryResult::Status::REACHABLE)
    << build.detail;
  EXPECT_EQ(build.requested_queries, 2U);
  EXPECT_EQ(build.unique_queries, 1U);
  EXPECT_EQ(build.outside_queries, 1U);
  EXPECT_EQ(field.query(inside).status, hobot_nav::GateQueryResult::Status::REACHABLE);
  EXPECT_EQ(field.query(outside).status, hobot_nav::GateQueryResult::Status::OUTSIDE_ROI);
}

TEST(GateRegionReachability, TimeoutPreservesAlreadySettledExactProbe)
{
  auto global = makeCostmap();
  auto local = makeCostmap();
  // The first query is itself the lexicographically first free Gate terminal;
  // one settled state therefore answers it before the hard state cap. The far
  // query must remain UNKNOWN, never erase the proven result.
  const hobot_nav::GateQuery terminal{{2.0, 1.9, -5.0 * kPi / 180.0}, 0.0};
  const hobot_nav::GateQuery far{{0.50, 4.50, 0.0}, 0.0};
  hobot_nav::GateRegionReachability field;
  auto config = hobot_nav::GateSearchConfig{};
  config.hard_budget_ms = 5000.0;
  config.maximum_states_expanded = 1U;
  const auto build = field.build(
    snapshot(global, local), hobot_nav::GateVehicleGeometry{}, testGate(),
    std::vector<hobot_nav::GateQuery>{terminal, far}, config);
  EXPECT_EQ(build.status, hobot_nav::GateQueryResult::Status::UNKNOWN_TIMEOUT)
    << build.detail;
  EXPECT_EQ(field.query(terminal).status, hobot_nav::GateQueryResult::Status::REACHABLE);
  EXPECT_EQ(field.query(far).status,
    hobot_nav::GateQueryResult::Status::UNKNOWN_TIMEOUT);
}
