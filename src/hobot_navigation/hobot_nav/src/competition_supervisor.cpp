#include "hobot_nav/competition_supervisor.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"

#include "hobot_nav/cost_semantics.hpp"

namespace hobot_nav
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kPathStep = 0.03;
constexpr std::size_t kDirectGateSizeX = 101U;
constexpr std::size_t kDirectGateSizeY = 58U;
constexpr std::size_t kDirectGateYawBins = 72U;
constexpr double kDirectGateResolutionM = 0.05;
constexpr double kDirectGateYawResolutionDeg = 5.0;
constexpr double kDirectGateXMinM = 0.0;
constexpr double kDirectGateXMaxM = 5.0;
constexpr double kDirectGateYMinM = 0.0;
constexpr double kDirectGateYMaxM = 2.85;
constexpr double kDirectGateQueryYMaxM = 2.0;
constexpr int kDirectGateLutSchemaVersion = 3;
constexpr int kQrLaneBiasSchemaVersion = 1;
constexpr std::size_t kQrLaneBiasWidth = 500U;
constexpr std::size_t kQrLaneBiasHeight = 500U;
constexpr std::size_t kQrLaneBiasBytes = kQrLaneBiasWidth * kQrLaneBiasHeight;
constexpr char kQrLaneBiasSha256[] =
  "22f87402127c6a6e88427ef884f1edc9bbe9446393db5106887a3ae53619006d";
constexpr char kDirectGateRobustMaskSha256[] =
  "70e4ff2ef755fc6997eb33b336191c18762a2e720384902c7c42f3d0ea5b26a2";
constexpr char kReverseGateEntryMagic[] = "RGE2";
constexpr char kReverseGateGoalIdMagic[] = "RGEG2";
constexpr double kGateGoalXyToleranceM = 0.30;
constexpr double kChannelEntryHandoffAdvanceM = 0.25;
constexpr double kChannelEntryRunwayAfterHandoffM = 0.15;
constexpr double kPostReverseGateHandoffX = 2.50;
constexpr double kPostReverseGateHandoffY = 2.30;
constexpr double kPostReverseGateHandoffYaw = kPi / 2.0;
constexpr std::size_t kChannelConeLutWidth = 500U;
constexpr std::size_t kChannelConeLutHeight = 500U;
constexpr double kChannelConeLutResolutionM = 0.01;
constexpr char kChannelConeLutSha256[] =
  "cc5ee4bf0f952f52432f4e7f46e36062d691c9388b3803edeb8bdbaddcfe6261";
constexpr double kChannelTransitionLengthM = 0.52;
constexpr double kChannelBottomStraightOffsetM = 0.070;
constexpr double kChannelTopStraightOffsetM = 0.055;
constexpr double kChannelMinimumStraightRemainingM = 0.60;
constexpr double kChannelConeLookaheadM = 1.25;
constexpr double kChannelConePassedMarginM = 0.45;
constexpr double kChannelEntryY = 3.18;
constexpr double kVlmStraightFractionMin = 0.35;
constexpr double kVlmStraightFractionMax = 0.72;
constexpr double kVlmStraightYMinM = 4.0;
constexpr double kVlmDetectorGateSettleSec = 0.15;

struct ChannelAssetSpec
{
  const char * file;
  const char * sha256;
};

constexpr std::array<ChannelAssetSpec, 6> kChannelTubeAssets{{
  {"tube_center_ccw.csv", "ed6c02b5ad566e211daa181f3c9090fa44fff836ac8ef101c5c0dc04ea46d083"},
  {"tube_center_cw.csv", "e924686cb891a7bff088c6448dd17a7820e85b8c80efe7d911c7bb1515207678"},
  {"tube_inner_ccw.csv", "e834a53d496a77cfd8359e29a3c1d1de19a93dfd1e4394f7e829f518443b0bd9"},
  {"tube_inner_cw.csv", "44947c50588b10f6738aef667bd13109c4bbdf4ebfec742f72a17e914260d763"},
  {"tube_outer_ccw.csv", "3e20fb0f7bb3180adb7ebfa055f4064c01a99b20d42d421311d77a09d57cf611"},
  {"tube_outer_cw.csv", "82736247283ce01eb72f2ecee884f304422dbfcf47cd4efdf91c8491a8858d4e"}
}};

double steadyAgeSec(const std::chrono::steady_clock::time_point receipt)
{
  if (receipt == std::chrono::steady_clock::time_point{}) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(0.0, std::chrono::duration<double>(
    std::chrono::steady_clock::now() - receipt).count());
}

struct CostmapPayloadStatus
{
  bool geometry_valid{false};
  bool size_valid{false};
  bool all_unknown{true};

  bool valid() const noexcept
  {
    return geometry_valid && size_valid && !all_unknown;
  }
};

CostmapPayloadStatus analyzeCostmap(const nav2_msgs::msg::Costmap & msg)
{
  CostmapPayloadStatus status;
  status.geometry_valid = msg.metadata.size_x > 0U && msg.metadata.size_y > 0U &&
    std::isfinite(msg.metadata.resolution) && msg.metadata.resolution > 0.0F;
  const auto expected = static_cast<std::size_t>(msg.metadata.size_x) *
    static_cast<std::size_t>(msg.metadata.size_y);
  status.size_valid = expected > 0U && msg.data.size() == expected;
  if (status.size_valid) {
    status.all_unknown = std::all_of(msg.data.begin(), msg.data.end(), [](const auto value) {
      return static_cast<unsigned int>(value) == 255U;
    });
  }
  return status;
}

std::uint32_t rotateRight(std::uint32_t value, unsigned shift)
{
  return (value >> shift) | (value << (32U - shift));
}

std::string sha256Hex(const std::vector<std::uint8_t> & input)
{
  static constexpr std::array<std::uint32_t, 64> round_constants{{
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
    0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
    0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
    0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
    0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
    0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
    0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
    0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
    0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
    0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
    0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
    0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
    0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
    0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
    0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
    0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U}};
  std::array<std::uint32_t, 8> hash{{
    0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
    0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U}};
  std::vector<std::uint8_t> padded = input;
  const std::uint64_t bit_count = static_cast<std::uint64_t>(input.size()) * 8U;
  padded.push_back(0x80U);
  while ((padded.size() % 64U) != 56U) {padded.push_back(0U);}
  for (int byte = 7; byte >= 0; --byte) {
    padded.push_back(static_cast<std::uint8_t>(bit_count >> (byte * 8)));
  }
  for (std::size_t offset = 0; offset < padded.size(); offset += 64U) {
    std::array<std::uint32_t, 64> words{};
    for (std::size_t i = 0; i < 16U; ++i) {
      const std::size_t at = offset + 4U * i;
      words[i] = (static_cast<std::uint32_t>(padded[at]) << 24U) |
        (static_cast<std::uint32_t>(padded[at + 1U]) << 16U) |
        (static_cast<std::uint32_t>(padded[at + 2U]) << 8U) |
        static_cast<std::uint32_t>(padded[at + 3U]);
    }
    for (std::size_t i = 16U; i < words.size(); ++i) {
      const std::uint32_t s0 = rotateRight(words[i - 15U], 7U) ^
        rotateRight(words[i - 15U], 18U) ^ (words[i - 15U] >> 3U);
      const std::uint32_t s1 = rotateRight(words[i - 2U], 17U) ^
        rotateRight(words[i - 2U], 19U) ^ (words[i - 2U] >> 10U);
      words[i] = words[i - 16U] + s0 + words[i - 7U] + s1;
    }
    std::uint32_t a = hash[0];
    std::uint32_t b = hash[1];
    std::uint32_t c = hash[2];
    std::uint32_t d = hash[3];
    std::uint32_t e = hash[4];
    std::uint32_t f = hash[5];
    std::uint32_t g = hash[6];
    std::uint32_t h = hash[7];
    for (std::size_t i = 0; i < words.size(); ++i) {
      const std::uint32_t sum1 = rotateRight(e, 6U) ^ rotateRight(e, 11U) ^
        rotateRight(e, 25U);
      const std::uint32_t choose = (e & f) ^ ((~e) & g);
      const std::uint32_t temporary1 = h + sum1 + choose + round_constants[i] + words[i];
      const std::uint32_t sum0 = rotateRight(a, 2U) ^ rotateRight(a, 13U) ^
        rotateRight(a, 22U);
      const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t temporary2 = sum0 + majority;
      h = g; g = f; f = e; e = d + temporary1;
      d = c; c = b; b = a; a = temporary1 + temporary2;
    }
    hash[0] += a; hash[1] += b; hash[2] += c; hash[3] += d;
    hash[4] += e; hash[5] += f; hash[6] += g; hash[7] += h;
  }
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto value : hash) {stream << std::setw(8) << value;}
  return stream.str();
}

double distance(double ax, double ay, double bx, double by)
{
  return std::hypot(ax - bx, ay - by);
}

double pathLengthM(const nav_msgs::msg::Path & path)
{
  double length_m = 0.0;
  for (std::size_t index = 1U; index < path.poses.size(); ++index) {
    const auto & a = path.poses[index - 1U].pose.position;
    const auto & b = path.poses[index].pose.position;
    length_m += distance(a.x, a.y, b.x, b.y);
  }
  return length_m;
}

std::vector<std::string> splitCsv(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  // std::getline omits the final empty field when a CSV row ends in a
  // delimiter.  Channel v2 admission rows intentionally use empty trailing
  // source/target fields for map and avoid assets, so preserve that field.
  if (!line.empty() && line.back() == ',') {
    fields.emplace_back();
  }
  return fields;
}
}  // namespace

CompetitionSupervisor::CompetitionSupervisor()
: Node("competition_supervisor"),
  tf_buffer_(get_clock()), tf_listener_(tf_buffer_), state_entered_(now())
{
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_optical_frame");
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  start_mission_ = declare_parameter<bool>("start_mission", false);
  const std::string start_topic = declare_parameter<std::string>(
    "start_topic", "/race/start");
  mission_scope_ = declare_parameter<std::string>("mission_scope", "full");
  race_continue_enabled_ = declare_parameter<bool>("race_continue_enabled", true);
  competition_quiet_mode_ = declare_parameter<bool>("competition_quiet_mode", false);
  direct_reverse_to_channel_entry_enabled_ = declare_parameter<bool>(
    "direct_reverse_to_channel_entry_enabled", true);
  if (mission_scope_ != "full" && mission_scope_ != "qr_reverse_gate" &&
    mission_scope_ != "channel_validation")
  {
    throw std::runtime_error(
            "mission_scope must be full, qr_reverse_gate, or channel_validation");
  }
  channel_runtime_admitted_ = declare_parameter<bool>(
    "channel_templates_runtime_allowed", false);
  channel_avoidance_v2_enabled_ = declare_parameter<bool>(
    "channel_avoidance_v2_enabled", false);
  channel_v2_asset_directory_ = declare_parameter<std::string>(
    "channel_v2.asset_directory", "");
  channel_v2_admission_sha256_ = declare_parameter<std::string>(
    "channel_v2.admission_sha256", "");
  channel_straight_ceiling_mps_ = declare_parameter<double>(
    "channel.speed.straight_ceiling_mps", 0.60);
  channel_corner_ceiling_mps_ = declare_parameter<double>(
    "channel.speed.corner_ceiling_mps", 0.42);
  channel_lane_change_ceiling_mps_ = declare_parameter<double>(
    "channel.speed.lane_change_ceiling_mps", 0.42);
  channel_corner_preview_distance_m_ = declare_parameter<double>(
    "channel.corner_preview_distance_m", 0.45);
  channel_corner_exit_hold_distance_m_ = declare_parameter<double>(
    "channel.corner_exit_hold_distance_m", 0.0);
  channel_corner_curvature_threshold_1pm_ = declare_parameter<double>(
    "channel.corner_curvature_threshold_1pm", 0.15);
  channel_cone_lookahead_m_ = declare_parameter<double>(
    "channel.cone_lookahead_m", kChannelConeLookaheadM);
  channel_cone_passed_margin_m_ = declare_parameter<double>(
    "channel.cone_passed_margin_m", kChannelConePassedMarginM);
  channel_cone_lateral_relevance_m_ = declare_parameter<double>(
    "channel.cone_lateral_relevance_m", 0.27);
  if (channel_straight_ceiling_mps_ <= 0.0 ||
    channel_corner_ceiling_mps_ <= 0.0 ||
    channel_lane_change_ceiling_mps_ <= 0.0 ||
    channel_corner_preview_distance_m_ <= 0.0 ||
    channel_corner_exit_hold_distance_m_ < 0.0 ||
    channel_corner_curvature_threshold_1pm_ <= 0.0 ||
    channel_cone_lookahead_m_ <= 0.0 || channel_cone_passed_margin_m_ <= 0.0 ||
    channel_cone_lateral_relevance_m_ <= 0.0)
  {
    throw std::runtime_error("Channel v2 speed/trigger parameters must be positive");
  }
  min_turning_radius_ = declare_parameter<double>("minimum_turning_radius", 0.35);
  yellow_x_min_ = declare_parameter<double>("yellow_region.x_min", 4.5);
  yellow_x_max_ = declare_parameter<double>("yellow_region.x_max", 5.0);
  yellow_y_min_ = declare_parameter<double>("yellow_region.y_min", 1.6);
  yellow_y_max_ = declare_parameter<double>("yellow_region.y_max", 2.0);
  parking_x_min_ = declare_parameter<double>("parking_region.x_min", 0.0);
  parking_x_max_ = declare_parameter<double>("parking_region.x_max", 1.00);
  parking_y_min_ = declare_parameter<double>("parking_region.y_min", 0.0);
  parking_y_max_ = declare_parameter<double>("parking_region.y_max", 0.40);
  parking_footprint_fraction_min_ = declare_parameter<double>(
    "parking_region.footprint_fraction_min", 0.50);
  actuation_latency_sec_ = declare_parameter<double>("reverse.actuation_latency_sec", 0.27);
  braking_deceleration_mps2_ = declare_parameter<double>(
    "reverse.worst_case_braking_deceleration_mps2", 0.70);
  braking_distance_margin_m_ = declare_parameter<double>(
    "reverse.braking_distance_margin_m", 0.04);
  steering_build_rate_radps_ = declare_parameter<double>(
    "reverse.steering_build_rate_radps", 1.20);
  steering_return_rate_radps_ = declare_parameter<double>(
    "reverse.steering_return_rate_radps", 1.00);
  steering_delay_distance_m_ = declare_parameter<double>(
    "reverse.steering_delay_distance_m", 0.12);
  reverse_search_step_m_ = declare_parameter<double>(
    "reverse.search_step_m", 0.10);
  reverse_max_search_distance_m_ = declare_parameter<double>(
    "reverse.max_search_distance_m", 1.50);
  steering_feedback_timeout_sec_ = declare_parameter<double>(
    "reverse.steering_feedback_timeout_sec", 0.15);
  const double requested_reverse_speed = declare_parameter<double>(
    "reverse_speed_limit", 0.35);
  reverse_speed_limit_mps_ = requested_reverse_speed;
  // The executed reverse ceiling and the solver's steering/braking model must
  // have one source.  A faster FollowPathReverse may never be assessed with a
  // slower design speed.
  reverse_design_speed_mps_ = reverse_speed_limit_mps_;
  stationary_linear_threshold_mps_ = declare_parameter<double>(
    "stationary.linear_threshold_mps", 0.02);
  stationary_angular_threshold_radps_ = declare_parameter<double>(
    "stationary.angular_threshold_radps", 0.05);
  stationary_minimum_duration_sec_ = declare_parameter<double>(
    "stationary.minimum_duration_sec", 0.15);
  stationary_minimum_samples_ = declare_parameter<int>(
    "stationary.minimum_samples", 4);
  stationary_maximum_odom_age_sec_ = declare_parameter<double>(
    "stationary.maximum_odom_age_sec", 0.10);
  observer_uncertainty_limit_rad_ = declare_parameter<double>(
    "reverse.command_observer_uncertainty_limit_deg", 3.0) * kPi / 180.0;
  observer_uncertainty_growth_radps_ = declare_parameter<double>(
    "reverse.command_observer_uncertainty_growth_radps", 0.02);
  snapshot_odom_warn_age_sec_ = declare_parameter<double>(
    "snapshot_freshness.odom_warn_age_sec", 0.30);
  snapshot_tf_warn_age_sec_ = declare_parameter<double>(
    "snapshot_freshness.tf_warn_age_sec", 0.30);
  snapshot_local_warn_age_sec_ = declare_parameter<double>(
    "snapshot_freshness.local_costmap_warn_age_sec", 0.60);
  snapshot_global_warn_age_sec_ = declare_parameter<double>(
    "snapshot_freshness.global_costmap_warn_age_sec", 1.00);
  snapshot_odom_hard_age_sec_ = declare_parameter<double>(
    "snapshot_freshness.odom_hard_age_sec", 0.80);
  snapshot_tf_hard_age_sec_ = declare_parameter<double>(
    "snapshot_freshness.tf_hard_age_sec", 0.80);
  snapshot_local_hard_age_sec_ = declare_parameter<double>(
    "snapshot_freshness.local_costmap_hard_age_sec", 1.50);
  snapshot_global_hard_age_sec_ = declare_parameter<double>(
    "snapshot_freshness.global_costmap_hard_age_sec", 2.00);
  qr_board_center_x_ = declare_parameter<double>("qr_board.center_x", 4.75);
  qr_board_center_y_ = declare_parameter<double>("qr_board.center_y", 1.80);
  qr_board_center_z_ = declare_parameter<double>("qr_board.center_z", 0.18);
  qr_board_normal_yaw_rad_ = declare_parameter<double>(
    "qr_board.normal_yaw_deg", 180.0) * kPi / 180.0;
  qr_board_width_m_ = declare_parameter<double>("qr_board.width_m", 0.210);
  qr_board_height_m_ = declare_parameter<double>("qr_board.height_m", 0.148);
  qr_camera_fx_ = declare_parameter<double>("qr_camera.fx", 682.7522450473028);
  qr_camera_fy_ = declare_parameter<double>("qr_camera.fy", 684.6825030858382);
  qr_camera_cx_ = declare_parameter<double>("qr_camera.cx", 948.0233927822394);
  qr_camera_cy_ = declare_parameter<double>("qr_camera.cy", 552.5465008451092);
  qr_camera_image_width_ = declare_parameter<int>("qr_camera.image_width", 1920);
  qr_camera_image_height_ = declare_parameter<int>("qr_camera.image_height", 1080);
  qr_min_projected_width_px_ = declare_parameter<double>(
    "qr_camera.minimum_projected_width_px", 70.0);
  qr_image_border_fraction_ = declare_parameter<double>(
    "qr_camera.image_border_fraction", 0.08);
  qr_max_slant_angle_rad_ = declare_parameter<double>(
    "qr_camera.maximum_slant_angle_deg", 60.0) * kPi / 180.0;
  qr_primary_x_ = declare_parameter<double>("qr.primary_x", 3.60);
  qr_primary_y_ = declare_parameter<double>("qr.primary_y", 1.20);
  qr_backup_x_ = declare_parameter<double>("qr.backup_x", 3.60);
  qr_backup_y_ = declare_parameter<double>("qr.backup_y", 0.80);
  qr_midcourse_replan_x_ = declare_parameter<double>("qr_midcourse_replan_x", 1.60);
  qr_lane_bias_enabled_ = declare_parameter<bool>("qr_lane_bias_enabled", false);
  qr_stage_timeout_sec_ = declare_parameter<double>("qr_stage_timeout_sec", 30.0);
  gate_region_.x_min = declare_parameter<double>("gate_region.x_min", 2.30);
  gate_region_.x_max = declare_parameter<double>("gate_region.x_max", 2.70);
  gate_region_.y_min = declare_parameter<double>("gate_region.y_min", 2.25);
  gate_region_.y_max = declare_parameter<double>("gate_region.y_max", 2.40);
  // Gate is a position handoff. Smac still receives a +Y heading hint, but
  // neither the Supervisor nor the fallback goal checker waits for yaw repair.
  gate_region_.yaw_min_rad = -kPi;
  gate_region_.yaw_max_rad = kPi;
  gate_region_.terminal_steering_abs_max_rad = kPi;
  gate_vehicle_geometry_.wheelbase_m = declare_parameter<double>(
    "vehicle.wheelbase_m", 0.144);
  gate_vehicle_geometry_.minimum_turning_radius_m = min_turning_radius_;
  gate_vehicle_geometry_.footprint_front_m = declare_parameter<double>(
    "vehicle.footprint_front_m", 0.28);
  gate_vehicle_geometry_.footprint_rear_m = declare_parameter<double>(
    "vehicle.footprint_rear_m", 0.11);
  gate_vehicle_geometry_.footprint_half_width_m = declare_parameter<double>(
    "vehicle.footprint_half_width_m", 0.13);
  gate_vehicle_geometry_.footprint_padding_m = declare_parameter<double>(
    "vehicle.footprint_padding_m", 0.0);
  channel_heading_limit_rad_ = declare_parameter<double>(
    "channel.heading_limit_deg", 8.0) * kPi / 180.0;
  gate_vehicle_geometry_.right_mechanical_limit_rad = declare_parameter<double>(
    "vehicle.right_mechanical_limit_deg", 22.3) * kPi / 180.0;
  if (reverse_design_speed_mps_ <= 0.0 || actuation_latency_sec_ < 0.0 ||
    braking_deceleration_mps2_ <= 0.0 || braking_distance_margin_m_ < 0.0 ||
    steering_build_rate_radps_ <= 0.0 || steering_return_rate_radps_ <= 0.0 ||
    steering_delay_distance_m_ < 0.0 ||
    reverse_search_step_m_ <= 0.0 || reverse_search_step_m_ > 0.20 ||
    reverse_max_search_distance_m_ < reverse_search_step_m_ ||
    steering_feedback_timeout_sec_ <= 0.0 || reverse_speed_limit_mps_ <= 0.0 ||
    stationary_linear_threshold_mps_ <= 0.0 ||
    stationary_angular_threshold_radps_ <= 0.0 ||
    stationary_minimum_duration_sec_ <= 0.0 || stationary_minimum_samples_ < 1 ||
    stationary_maximum_odom_age_sec_ <= 0.0 ||
    parking_footprint_fraction_min_ <= 0.0 ||
    parking_footprint_fraction_min_ > 1.0 ||
    observer_uncertainty_growth_radps_ <= 0.0 ||
    snapshot_odom_warn_age_sec_ <= 0.0 || snapshot_tf_warn_age_sec_ <= 0.0 ||
    snapshot_local_warn_age_sec_ <= 0.0 || snapshot_global_warn_age_sec_ <= 0.0 ||
    snapshot_odom_hard_age_sec_ < snapshot_odom_warn_age_sec_ ||
    snapshot_tf_hard_age_sec_ < snapshot_tf_warn_age_sec_ ||
    snapshot_local_hard_age_sec_ < snapshot_local_warn_age_sec_ ||
    snapshot_global_hard_age_sec_ < snapshot_global_warn_age_sec_ ||
    qr_board_width_m_ <= 0.0 || qr_board_height_m_ <= 0.0 ||
    qr_board_center_z_ <= 0.0 || qr_camera_fx_ <= 0.0 || qr_camera_fy_ <= 0.0 ||
    qr_camera_image_width_ < 1 || qr_camera_image_height_ < 1 ||
    qr_min_projected_width_px_ <= 0.0 || qr_image_border_fraction_ < 0.0 ||
    qr_image_border_fraction_ >= 0.5 || qr_max_slant_angle_rad_ <= 0.0 ||
    qr_max_slant_angle_rad_ > kPi / 3.0 + 1.0e-9 ||
    qr_stage_timeout_sec_ <= 0.0 ||
    !std::isfinite(qr_primary_x_) || !std::isfinite(qr_primary_y_) ||
    !std::isfinite(qr_backup_x_) || !std::isfinite(qr_backup_y_) ||
    !std::isfinite(qr_midcourse_replan_x_) || qr_midcourse_replan_x_ <= 0.0 ||
    qr_midcourse_replan_x_ >= std::min(qr_primary_x_, qr_backup_x_) ||
    distance(qr_primary_x_, qr_primary_y_, qr_backup_x_, qr_backup_y_) < 0.05)
  {
    throw std::runtime_error("invalid reverse actuator/braking model parameters");
  }
  const auto package_share = ament_index_cpp::get_package_share_directory("hobot_nav");
  const auto channel_tube_directory = package_share + "/config/channel_tubes";
  const auto qr_lane_file = declare_parameter<std::string>(
    "qr_lane_reference_file", package_share + "/config/qr_lane_reference.csv");
  direct_gate_robust_mask_path_ =
    package_share + "/runtime/direct_gate_robust_neutral.bin";
  const bool direct_gate_lut_loaded = loadDirectGateRobustMask(
    direct_gate_robust_mask_path_, direct_gate_robust_mask_error_);
  const std::string lut_metadata =
    "path=" + direct_gate_robust_mask_path_ +
    ",bytes=" + std::to_string(direct_gate_robust_mask_file_size_) +
    ",sha256=" + direct_gate_robust_mask_sha256_ +
    ",schema_version=" + std::to_string(kDirectGateLutSchemaVersion) +
    ",size_x=" + std::to_string(kDirectGateSizeX) +
    ",size_y=" + std::to_string(kDirectGateSizeY) +
    ",yaw_bins=" + std::to_string(kDirectGateYawBins) +
    ",xy_resolution_m=" + std::to_string(kDirectGateResolutionM) +
    ",yaw_resolution_deg=" + std::to_string(kDirectGateYawResolutionDeg) +
    ",gate_entry_semantics=start_xy_yaw_lattice" +
    ",x_range_m=[0.0,5.0],y_range_m=[0.0,2.85],query_y_max_m=2.0,loaded=" +
    (direct_gate_lut_loaded ? "true" : "false") +
    ",reason=" + direct_gate_robust_mask_error_;
  if (direct_gate_lut_loaded) {
    RCLCPP_INFO(get_logger(), "Direct-Gate LUT %s", lut_metadata.c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "Direct-Gate LUT %s", lut_metadata.c_str());
  }
  reverse_gate_entry_mask_path_ = package_share + "/runtime/reverse_gate_entry_clean.bin";
  reverse_gate_goal_ids_path_ = package_share + "/runtime/reverse_gate_goal_id_clean.bin";
  const bool reverse_entry_loaded = loadReverseGateEntryMask(
    reverse_gate_entry_mask_path_, reverse_gate_entry_mask_error_);
  const bool reverse_goals_loaded = loadReverseGateGoalIds(
    reverse_gate_goal_ids_path_, reverse_gate_goal_ids_error_);
  const std::string reverse_lut_detail =
    "entry_path=" + reverse_gate_entry_mask_path_ +
    ",entry_bytes=" + std::to_string(reverse_gate_entry_mask_file_size_) +
    ",entry_sha256=" + reverse_gate_entry_mask_sha256_ +
    ",goal_id_path=" + reverse_gate_goal_ids_path_ +
    ",goal_id_bytes=" + std::to_string(reverse_gate_goal_ids_file_size_) +
    ",goal_id_sha256=" + reverse_gate_goal_ids_sha256_ +
    ",goal_id_semantics=gate_witness,lookup=nearest_cell_only" +
    ",entry_grid=" + std::to_string(reverse_gate_entry_size_x_) + "x" +
    std::to_string(reverse_gate_entry_size_y_) + "x" +
    std::to_string(reverse_gate_entry_yaw_bins_) +
    ",loaded=" + (reverse_entry_loaded && reverse_goals_loaded ? "true" : "false") +
    ",entry_reason=" + reverse_gate_entry_mask_error_ +
    ",goal_reason=" + reverse_gate_goal_ids_error_;
  if (reverse_entry_loaded && reverse_goals_loaded) {
    RCLCPP_INFO(get_logger(), "Reverse-Gate entry LUT %s", reverse_lut_detail.c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "Reverse-Gate entry LUT unavailable; Reverse search will fail closed: %s",
      reverse_lut_detail.c_str());
  }
  cones_freshness_sec_ = declare_parameter<double>("cones_freshness_sec", 0.50);
  diagnostics_path_geometry_enabled_ = declare_parameter<bool>(
    "diagnostics_path_geometry_enabled", false);

  if (std::abs(min_turning_radius_ - 0.35) > 1e-9) {
    throw std::runtime_error("minimum_turning_radius must be exactly 0.35 m");
  }
  if (channel_avoidance_v2_enabled_) {
    if (!channel_runtime_admitted_) {
      throw std::runtime_error("Channel v2 requires channel_templates_runtime_allowed=true");
    }
    if (!loadChannelV2Assets(channel_v2_asset_directory_, channel_v2_admission_sha256_)) {
      throw std::runtime_error("Channel v2 manifest/assets failed admission");
    }
  } else if (!loadChannelTubeLibrary(channel_tube_directory) ||
    channel_tube_library_.size() != 6U ||
    !loadChannelConeSideLut(package_share + "/runtime/channel_cone_side_class_1cm.bin") ||
    !loadChannelTransitionTemplate(
      package_share + "/config/channel_transitions/transition_bottom_D_plus0.070_L_0.52.csv",
      "12fb78903e6dc74f7b4cee134e6a74a1360037e0952cd9af85cf48fae0e3c50a",
      channel_transition_bottom_plus_) ||
    !loadChannelTransitionTemplate(
      package_share + "/config/channel_transitions/transition_bottom_D_minus0.070_L_0.52.csv",
      "ad92dac62feb287cca843bf1858cef80cb9cbf6c2254327573f7ea4b8d7edaaa",
      channel_transition_bottom_minus_) ||
    !loadChannelTransitionTemplate(
      package_share + "/config/channel_transitions/transition_top_D_plus0.055_L_0.52.csv",
      "2b72abee8bcdcd6538d6755f3015a202e903c288d1c19ac6a5650bb4a5bd25ac",
      channel_transition_top_plus_) ||
    !loadChannelTransitionTemplate(
      package_share + "/config/channel_transitions/transition_top_D_minus0.055_L_0.52.csv",
      "1df3c681e660f467cc9429f0e67326972efb3632621f7e28ae6c26cffa275dca",
      channel_transition_top_minus_))
  {
    throw std::runtime_error("verified legacy Channel Tube/LUT/transition assets unavailable");
  }
  if (!loadChannelBranch(
      package_share + "/config/channel_branches/gate_exit_cw_to_return_handoff.csv",
      channel_exit_cw_path_) ||
    !loadChannelBranch(
      package_share + "/config/channel_branches/gate_exit_ccw_to_return_handoff.csv",
      channel_exit_ccw_path_))
  {
    throw std::runtime_error("verified Channel exit assets unavailable");
  }
  // Keep the historical reference/asset load available for an explicit A/B
  // rollback, but do not make it a prerequisite for the formal fixed-goal
  // QR path.  The formal launch leaves qr_lane_bias_enabled=false.
  if (qr_lane_bias_enabled_) {
    std::string reference_error;
    if (!qr_lane_reference_.loadCsv(qr_lane_file, &reference_error)) {
      throw std::runtime_error(reference_error);
    }
  }
  qr_lane_bias_asset_path_ = package_share + "/runtime/qr_lane_bias_active_1cm.bin";
  qr_lane_bias_asset_loaded_ = qr_lane_bias_enabled_ && loadQrLaneBiasAsset(
    qr_lane_bias_asset_path_, qr_lane_bias_asset_error_);
  if (!qr_lane_bias_enabled_) {
    RCLCPP_INFO(
      get_logger(), "QR lane bias disabled for formal fixed-goal P->QR navigation");
  } else if (!qr_lane_bias_asset_loaded_) {
    RCLCPP_ERROR(
      get_logger(), "QR lane bias asset unavailable path=%s schema_version=%d reason=%s",
      qr_lane_bias_asset_path_.c_str(), kQrLaneBiasSchemaVersion,
      qr_lane_bias_asset_error_.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(), "QR lane bias asset loaded path=%s bytes=%zu sha256=%s schema_version=%d",
      qr_lane_bias_asset_path_.c_str(), qr_lane_bias_active_data_.size(),
      qr_lane_bias_asset_sha256_.c_str(), kQrLaneBiasSchemaVersion);
  }

  arbiter_ = std::make_unique<MotionActionArbiter>(this);
  const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
  // Costmap snapshots are large (500x500) and their publishers are
  // RELIABLE + TRANSIENT_LOCAL.  SensorDataQoS is BEST_EFFORT + VOLATILE;
  // it can appear matched while silently losing fragmented snapshots on X5.
  // Keep the high-rate sensors on SensorDataQoS, but require complete/latest
  // global and local costmap samples here.
  const auto costmap_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", sensor_qos,
    std::bind(&CompetitionSupervisor::onOdom, this, std::placeholders::_1));
  costmap_sub_ = create_subscription<nav2_msgs::msg::Costmap>(
    "/global_costmap/costmap_raw", costmap_qos,
    std::bind(&CompetitionSupervisor::onCostmap, this, std::placeholders::_1));
  local_costmap_sub_ = create_subscription<nav2_msgs::msg::Costmap>(
    "/local_costmap/costmap_raw", costmap_qos,
    std::bind(&CompetitionSupervisor::onLocalCostmap, this, std::placeholders::_1));
  static_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&CompetitionSupervisor::onStaticMap, this, std::placeholders::_1));
  start_sub_ = create_subscription<std_msgs::msg::String>(
    start_topic, rclcpp::QoS(1).reliable(),
    std::bind(&CompetitionSupervisor::onStartSignal, this, std::placeholders::_1));
  qr_sub_ = create_subscription<hbmem_wechatcode_msgs::msg::QrCode>(
    "/code", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&CompetitionSupervisor::onQr, this, std::placeholders::_1));
  vlm_sub_ = create_subscription<ollama_msgs::msg::OllamaResponse>(
    "/ollama", rclcpp::QoS(1).reliable(),
    std::bind(&CompetitionSupervisor::onVlm, this, std::placeholders::_1));
  bev_accepted_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/vision/accepted_bev_pose", rclcpp::QoS(1).reliable(),
    std::bind(&CompetitionSupervisor::onBevAccepted, this, std::placeholders::_1));
  bev_correction_event_sub_ = create_subscription<std_msgs::msg::String>(
    "/localization/bev_correction_event", rclcpp::QoS(10).reliable(),
    std::bind(
      &CompetitionSupervisor::onBevCorrectionEvent, this, std::placeholders::_1));
  steering_angle_sub_ = create_subscription<std_msgs::msg::Float32>(
    "/base/steering_angle", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&CompetitionSupervisor::onSteeringAngle, this, std::placeholders::_1));
  safe_cmd_topic_ = declare_parameter<std::string>(
    "safe_cmd_topic", "/cmd_vel_safe");
  (void)declare_parameter<bool>("bev_correction_enabled_on_start", false);
  safe_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    safe_cmd_topic_, rclcpp::QoS(1).reliable(),
    std::bind(&CompetitionSupervisor::onSafeCommand, this, std::placeholders::_1));
  cones_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/cones/points", sensor_qos,
    std::bind(&CompetitionSupervisor::onCones, this, std::placeholders::_1));
  planner_state_probe_.client = create_client<lifecycle_msgs::srv::GetState>(
    "/planner_server/get_state");
  controller_state_probe_.client = create_client<lifecycle_msgs::srv::GetState>(
    "/controller_server/get_state");
  smoother_state_probe_.client = create_client<lifecycle_msgs::srv::GetState>(
    "/velocity_smoother/get_state");

  const auto gate_qos = rclcpp::QoS(1).reliable().transient_local();
  bev_compute_gate_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/race/bev_compute_enabled", gate_qos);
  bev_gate_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/race/bev_localization_corrections_enabled", gate_qos);
  vlm_gate_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/race/vlm_image_cache_enabled", gate_qos);
  vlm_detector_gate_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/race/vlm_detector_enabled", gate_qos);
  reverse_gate_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/race/qr_escape_reverse_only", gate_qos);
  direct_gate_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/race/direct_gate_active", gate_qos);
  vlm_trigger_pub_ = create_publisher<std_msgs::msg::String>("/vlm_trigger", 1);
  display_pub_ = create_publisher<std_msgs::msg::String>("/race/display_text", 1);
  direction_pub_ = create_publisher<std_msgs::msg::String>("/race/direction", gate_qos);
  const auto audit_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable().transient_local();
  event_pub_ = create_publisher<std_msgs::msg::String>("/race/mission_event", audit_qos);
  audit_pub_ = create_publisher<std_msgs::msg::String>("/race/mission_audit", audit_qos);
  reverse_diag_pub_ = create_publisher<std_msgs::msg::String>(
    "/race/reverse_entry_diag", audit_qos);
  perf_trace_pub_ = create_publisher<std_msgs::msg::String>(
    "/race/perf_trace", audit_qos);
  startup_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/race/ready", gate_qos);
  channel_speed_limit_pub_ = create_publisher<nav2_msgs::msg::SpeedLimit>(
    "/speed_limit", rclcpp::QoS(10).reliable());
  if (qr_lane_bias_enabled_) {
    qr_lane_bias_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/race/qr_lane_bias_map", gate_qos);
  }
  publishStartupReady(false);
  publishGates();
  tick_timer_ = create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&CompetitionSupervisor::onTick, this));
  worker_thread_ = std::thread(&CompetitionSupervisor::workerLoop, this);
  publishAudit("startup", std::string("mission_scope=") + mission_scope_ + "," +
    (channel_runtime_admitted_ ?
    "channel templates explicitly admitted" :
    "channel templates NOT admitted; mission will fail closed before channel motion") +
    ",direct_gate_robust_mask=" +
    (direct_gate_robust_mask_.empty() ?
    "unavailable:" + direct_gate_robust_mask_error_ : direct_gate_robust_mask_path_));
}

CompetitionSupervisor::~CompetitionSupervisor()
{
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    worker_stop_ = true;
    pending_worker_job_.reset();
    worker_completion_.reset();
  }
  worker_cv_.notify_all();
  if (worker_thread_.joinable()) {worker_thread_.join();}
}

void CompetitionSupervisor::enqueueWorker(
  std::function<std::function<void()>()> compute)
{
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    const auto generation = ++worker_generation_;
    pending_worker_job_ = std::make_pair(generation, std::move(compute));
    worker_completion_.reset();
  }
  worker_cv_.notify_one();
}

void CompetitionSupervisor::drainWorkerCompletion()
{
  std::function<void()> completion;
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_completion_ && worker_completion_->first == worker_generation_) {
      completion = std::move(worker_completion_->second);
    }
    worker_completion_.reset();
  }
  if (completion) {completion();}
}

void CompetitionSupervisor::invalidateWorker()
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  ++worker_generation_;
  pending_worker_job_.reset();
  worker_completion_.reset();
  reverse_build_pending_ = false;
}

void CompetitionSupervisor::workerLoop()
{
  while (true) {
    std::pair<std::uint64_t, std::function<std::function<void()>()>> job;
    {
      std::unique_lock<std::mutex> lock(worker_mutex_);
      worker_cv_.wait(lock, [this]() {return worker_stop_ || pending_worker_job_;});
      if (worker_stop_) {return;}
      job = std::move(*pending_worker_job_);
      pending_worker_job_.reset();
    }
    auto completion = job.second();
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      if (!worker_stop_ && job.first == worker_generation_) {
        worker_completion_ = std::make_pair(job.first, std::move(completion));
      }
    }
  }
}

bool CompetitionSupervisor::loadChannelTubeLibrary(const std::string & directory)
{
  channel_tube_library_.clear();
  for (const auto & asset : kChannelTubeAssets) {
    const std::string path = directory + "/" + asset.file;
    std::ifstream binary(path, std::ios::binary);
    if (!binary.good()) {
      RCLCPP_ERROR(get_logger(), "cannot open Channel Tube: %s", path.c_str());
      return false;
    }
    const std::vector<std::uint8_t> bytes{
      std::istreambuf_iterator<char>(binary), std::istreambuf_iterator<char>()};
    if (sha256Hex(bytes) != asset.sha256) {
      RCLCPP_ERROR(get_logger(), "Channel Tube SHA256 mismatch: %s", path.c_str());
      return false;
    }
    std::istringstream input(std::string(bytes.begin(), bytes.end()));
    std::string line;
    if (!std::getline(input, line) ||
      line != "s_m,x_m,y_m,yaw_rad,kappa_1pm,radius_m,tube,direction")
    {
      return false;
    }
    ChannelTube tube;
    tube.path.header.frame_id = map_frame_;
    double previous_s = -1.0;
    while (std::getline(input, line)) {
      const auto fields = splitCsv(line);
      if (fields.size() != 8U) {return false;}
      try {
        const double s_m = std::stod(fields[0]);
        const double kappa = std::stod(fields[4]);
        if (!std::isfinite(s_m) || s_m <= previous_s ||
          !std::isfinite(kappa) || std::abs(kappa) > 2.500001)
        {
          return false;
        }
        if (tube.path.poses.empty()) {
          const std::string & lane = fields[6];
          const std::string & direction = fields[7];
          if (lane == "center") {tube.lane = ChannelLane::CENTER;}
          else if (lane == "inner") {tube.lane = ChannelLane::INNER;}
          else if (lane == "outer") {tube.lane = ChannelLane::OUTER;}
          else {return false;}
          if (direction == "cw") {tube.clockwise = true;}
          else if (direction == "ccw") {tube.clockwise = false;}
          else {return false;}
          tube.id = lane + "/" + direction;
        }
        const std::string expected_lane = tube.lane == ChannelLane::CENTER ?
          "center" : (tube.lane == ChannelLane::INNER ? "inner" : "outer");
        const std::string expected_direction = tube.clockwise ? "cw" : "ccw";
        if (fields[6] != expected_lane || fields[7] != expected_direction) {return false;}
        auto stamped = poseStamped(Pose2D{
          std::stod(fields[1]), std::stod(fields[2]), std::stod(fields[3])});
        stamped.header.frame_id = map_frame_;
        tube.path.poses.push_back(std::move(stamped));
        tube.s_m.push_back(s_m);
        tube.kappa_1pm.push_back(kappa);
        previous_s = s_m;
      } catch (const std::exception &) {
        return false;
      }
    }
    const std::size_t expected_samples = 865U;
    if (tube.path.poses.size() != expected_samples ||
      std::abs(tube.s_m.front()) > 1.0e-9)
    {
      return false;
    }
    const auto & first = tube.path.poses.front().pose.position;
    const auto & last = tube.path.poses.back().pose.position;
    if (distance(first.x, first.y, last.x, last.y) > 0.011) {return false;}
    tube.loop_length_m = tube.s_m.back();
    channel_tube_library_.push_back(std::move(tube));
  }
  RCLCPP_INFO(
    get_logger(),
    "loaded six verified Channel Tubes: Center/Inner/Outer CW+CCW, design Rmin>=0.400m");
  return true;
}

bool CompetitionSupervisor::loadChannelConeSideLut(const std::string & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input.good()) {return false;}
  std::vector<std::uint8_t> bytes{
    std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  if (bytes.size() != kChannelConeLutWidth * kChannelConeLutHeight ||
    sha256Hex(bytes) != kChannelConeLutSha256 ||
    std::any_of(bytes.begin(), bytes.end(), [](std::uint8_t value) {return value > 3U;}))
  {
    return false;
  }
  channel_cone_side_lut_ = std::move(bytes);
  return true;
}

bool CompetitionSupervisor::loadChannelV2Assets(
  const std::string & directory, const std::string & admission_sha256)
{
  const auto reject = [this](const std::string & reason) {
      RCLCPP_ERROR(get_logger(), "Channel v2 admission reject: %s", reason.c_str());
      return false;
    };
  if (directory.empty() || admission_sha256.size() != 64U) {
    return reject("empty asset directory or invalid admission SHA256 length");
  }
  const std::string admission_path = directory + "/runtime_admission.csv";
  std::ifstream admission_binary(admission_path, std::ios::binary);
  if (!admission_binary.good()) {return reject("runtime_admission.csv unavailable");}
  const std::vector<std::uint8_t> admission_bytes{
    std::istreambuf_iterator<char>(admission_binary), std::istreambuf_iterator<char>()};
  if (sha256Hex(admission_bytes) != admission_sha256) {
    return reject("runtime_admission.csv SHA256 mismatch");
  }
  std::istringstream admission(std::string(admission_bytes.begin(), admission_bytes.end()));
  std::string line;
  if (!std::getline(admission, line) ||
    line != "kind,id,file,sha256,direction,source_lane,target_lane")
  {
    return reject("runtime_admission.csv header mismatch");
  }
  channel_tube_library_.clear();
  channel_v2_connectors_.clear();
  channel_avoid_lut_cw_.clear();
  channel_avoid_lut_ccw_.clear();
  std::string expected_static_map_sha256;
  const auto lane_from_text = [](const std::string & value, ChannelLane & lane) {
      if (value == "center") {lane = ChannelLane::CENTER; return true;}
      if (value == "inner") {lane = ChannelLane::INNER; return true;}
      if (value == "outer") {lane = ChannelLane::OUTER; return true;}
      return false;
    };
  while (std::getline(admission, line)) {
    const auto fields = splitCsv(line);
    if (fields.size() != 7U || fields[2].empty() || fields[3].size() != 64U ||
      fields[2].front() == '/' || fields[2].find("..") != std::string::npos)
    {
      return reject("malformed admission row");
    }
    const std::string path = directory + "/" + fields[2];
    std::ifstream binary(path, std::ios::binary);
    if (!binary.good()) {return reject("admission asset unavailable: " + path);}
    const std::vector<std::uint8_t> bytes{
      std::istreambuf_iterator<char>(binary), std::istreambuf_iterator<char>()};
    if (sha256Hex(bytes) != fields[3]) {
      return reject("asset SHA256 mismatch: " + fields[2]);
    }
    if (fields[0] == "map") {
      if (!expected_static_map_sha256.empty()) {
        return reject("duplicate static map admission row");
      }
      expected_static_map_sha256 = fields[3];
      continue;
    }
    if (fields[0] == "avoid") {
      if (bytes.size() != kChannelConeLutWidth * kChannelConeLutHeight ||
        std::any_of(bytes.begin(), bytes.end(), [](std::uint8_t value) {return value > 2U;}))
      {
        return reject("invalid avoid LUT: " + fields[2]);
      }
      if (fields[4] == "cw") {channel_avoid_lut_cw_ = bytes;}
      else if (fields[4] == "ccw") {channel_avoid_lut_ccw_ = bytes;}
      else {return reject("invalid avoid direction: " + fields[4]);}
      continue;
    }
    std::istringstream input(std::string(bytes.begin(), bytes.end()));
    std::string csv_header;
    if (!std::getline(input, csv_header)) {return reject("empty CSV asset: " + fields[2]);}
    if (fields[0] == "tube") {
      if (csv_header != "s_m,x_m,y_m,yaw_rad,kappa_1pm,radius_m,tube,direction") {
        return reject("tube CSV header mismatch: " + fields[2]);
      }
      ChannelTube tube;
      if (!lane_from_text(fields[5], tube.lane)) {
        return reject("invalid tube lane: " + fields[5]);
      }
      tube.clockwise = fields[4] == "cw";
      if (!tube.clockwise && fields[4] != "ccw") {
        return reject("invalid tube direction: " + fields[4]);
      }
      tube.id = fields[1];
      tube.path.header.frame_id = map_frame_;
      double previous_s = -1.0;
      while (std::getline(input, line)) {
        const auto row = splitCsv(line);
        if (row.size() != 8U || row[6] != fields[5] || row[7] != fields[4]) {
          return reject("tube CSV row schema mismatch: " + fields[2]);
        }
        try {
          const double s_m = std::stod(row[0]);
          const double kappa = std::stod(row[4]);
          if (!std::isfinite(s_m) || s_m <= previous_s || !std::isfinite(kappa) ||
            std::abs(kappa) > 1.0 / min_turning_radius_ + 1.0e-5) {
            return reject("tube CSV monotonicity/curvature violation: " + fields[2]);
          }
          auto stamped = poseStamped(Pose2D{
            std::stod(row[1]), std::stod(row[2]), std::stod(row[3])});
          stamped.header.frame_id = map_frame_;
          tube.path.poses.push_back(std::move(stamped));
          tube.s_m.push_back(s_m); tube.kappa_1pm.push_back(kappa); previous_s = s_m;
      } catch (const std::exception &) {
        return reject("tube CSV numeric parse failure: " + fields[2]);
      }
      }
      if (tube.path.poses.size() < 100U || tube.s_m.front() != 0.0) {
        return reject("tube CSV too short or first s is not zero: " + fields[2]);
      }
      const auto & first = tube.path.poses.front().pose.position;
      const auto & last = tube.path.poses.back().pose.position;
      if (distance(first.x, first.y, last.x, last.y) > 0.012) {
        return reject("tube CSV is not closed: " + fields[2]);
      }
      tube.loop_length_m = tube.s_m.back();
      channel_tube_library_.push_back(std::move(tube));
    } else if (fields[0] == "connector") {
      if (csv_header !=
        "s_m,x_m,y_m,yaw_rad,kappa_1pm,radius_m,direction,source_lane,target_lane,anchor_id")
      {
        return reject("connector CSV header mismatch: " + fields[2]);
      }
      ChannelConnector connector;
      connector.id = fields[1]; connector.clockwise = fields[4] == "cw";
      if ((!connector.clockwise && fields[4] != "ccw") ||
        !lane_from_text(fields[5], connector.source_lane) ||
        !lane_from_text(fields[6], connector.target_lane)) {
        return reject("invalid connector direction or lane: " + fields[2]);
      }
      connector.path.header.frame_id = map_frame_;
      while (std::getline(input, line)) {
        const auto row = splitCsv(line);
        if (row.size() != 10U || row[6] != fields[4] || row[7] != fields[5] ||
          row[8] != fields[6]) {
          return reject("connector CSV row schema mismatch: " + fields[2]);
        }
        try {
          auto stamped = poseStamped(Pose2D{
            std::stod(row[1]), std::stod(row[2]), std::stod(row[3])});
          stamped.header.frame_id = map_frame_;
          connector.path.poses.push_back(std::move(stamped));
        } catch (const std::exception &) {
          return reject("connector CSV numeric parse failure: " + fields[2]);
        }
      }
      if (connector.path.poses.size() < 20U) {
        return reject("connector CSV too short: " + fields[2]);
      }
      channel_v2_connectors_.push_back(std::move(connector));
    } else {
      return reject("unknown admission asset kind: " + fields[0]);
    }
  }
  for (auto & connector : channel_v2_connectors_) {
    const auto tube_it = std::find_if(
      channel_tube_library_.begin(), channel_tube_library_.end(),
      [&connector](const ChannelTube & tube) {
        return tube.clockwise == connector.clockwise &&
               tube.lane == connector.source_lane;
      });
    if (tube_it == channel_tube_library_.end() || connector.path.poses.empty()) {
      return reject("connector source lane has no matching tube or connector is empty: " + connector.id);
    }
    const auto & start = connector.path.poses.front().pose;
    const auto projection = projectToChannelTube(*tube_it, Pose2D{
      start.position.x, start.position.y, yawOf(start.orientation)});
    if (projection.distance_m > 0.003) {
      return reject("connector source is not on source tube: " + connector.id +
        " distance_m=" + std::to_string(projection.distance_m));
    }
    connector.source_index = projection.index;
    connector.source_s_m = projection.s_mod_m;
  }
  const std::string runtime_map_path =
    ament_index_cpp::get_package_share_directory("hobot_nav") +
    "/maps/rdk_2026_hospital_static_1cm.pgm";
  std::ifstream runtime_map(runtime_map_path, std::ios::binary);
  const bool runtime_map_open = runtime_map.good();
  const std::vector<std::uint8_t> runtime_map_bytes{
    std::istreambuf_iterator<char>(runtime_map), std::istreambuf_iterator<char>()};
  const bool map_matches = runtime_map_open && !expected_static_map_sha256.empty() &&
    sha256Hex(runtime_map_bytes) == expected_static_map_sha256;
  const bool complete = map_matches && channel_tube_library_.size() == 6U &&
    channel_v2_connectors_.size() >= 8U &&
    channel_avoid_lut_cw_.size() == kChannelConeLutWidth * kChannelConeLutHeight &&
    channel_avoid_lut_ccw_.size() == kChannelConeLutWidth * kChannelConeLutHeight;
  if (complete) {
    RCLCPP_INFO(get_logger(), "loaded manifest-bound Channel v2 Tube/connector/avoid assets");
  }
  return complete || (reject(
      "complete set missing: map_matches=" + std::string(map_matches ? "true" : "false") +
      ", tubes=" + std::to_string(channel_tube_library_.size()) +
      ", connectors=" + std::to_string(channel_v2_connectors_.size()) +
      ", avoid_cw=" + std::to_string(channel_avoid_lut_cw_.size()) +
      ", avoid_ccw=" + std::to_string(channel_avoid_lut_ccw_.size())), false);
}

bool CompetitionSupervisor::loadChannelTransitionTemplate(
  const std::string & path, const std::string & expected_sha256,
  std::vector<ChannelTransitionPoint> & points)
{
  std::ifstream binary(path, std::ios::binary);
  if (!binary.good()) {return false;}
  const std::vector<std::uint8_t> bytes{
    std::istreambuf_iterator<char>(binary), std::istreambuf_iterator<char>()};
  if (sha256Hex(bytes) != expected_sha256) {return false;}
  std::istringstream input(std::string(bytes.begin(), bytes.end()));
  std::string line;
  if (!std::getline(input, line)) {return false;}
  points.clear();
  double final_s = 0.0;
  while (std::getline(input, line)) {
    const auto fields = splitCsv(line);
    if (fields.size() != 8U) {return false;}
    try {
      final_s = std::stod(fields[0]);
      points.push_back(ChannelTransitionPoint{
        std::stod(fields[1]), std::stod(fields[2]), std::stod(fields[3])});
    } catch (const std::exception &) {return false;}
  }
  return points.size() == 105U &&
         std::abs(final_s - kChannelTransitionLengthM) <= 1.0e-6;
}

bool CompetitionSupervisor::loadChannelBranch(
  const std::string & path, nav_msgs::msg::Path & branch)
{
  std::ifstream binary(path, std::ios::binary);
  if (!binary.good()) {return false;}
  const std::vector<std::uint8_t> bytes{
    std::istreambuf_iterator<char>(binary), std::istreambuf_iterator<char>()};
  const bool clockwise = path.find("_cw_") != std::string::npos;
  const std::string expected_hash = clockwise ?
    "ac5121dc726a02354d93291b0891cd057d89a6b9383d2fa4ba42be440b5a7b27" :
    "673bea88babdcc9d2c75a9039871c40bf4a5a0c801e241238092818972fc5b07";
  if (sha256Hex(bytes) != expected_hash) {return false;}
  std::istringstream input(std::string(bytes.begin(), bytes.end()));
  std::string line;
  if (!std::getline(input, line)) {return false;}
  branch = nav_msgs::msg::Path{};
  branch.header.frame_id = map_frame_;
  while (std::getline(input, line)) {
    const auto fields = splitCsv(line);
    if (fields.size() != 7U) {return false;}
    try {
      auto stamped = poseStamped(Pose2D{
        std::stod(fields[1]), std::stod(fields[2]), std::stod(fields[3])});
      stamped.header.frame_id = map_frame_;
      branch.poses.push_back(std::move(stamped));
    } catch (const std::exception &) {return false;}
  }
  if (branch.poses.size() < 100U) {return false;}
  const auto & start = branch.poses.front().pose;
  const auto & finish = branch.poses.back().pose;
  const double split_x = clockwise ? 2.961454 : 2.038546;
  return distance(start.position.x, start.position.y, split_x, 3.18) <= 1.0e-5 &&
         distance(finish.position.x, finish.position.y, 2.5, 1.87) <= 1.0e-5 &&
         std::abs(normalize(yawOf(finish.orientation) + kPi / 2.0)) <= 1.0e-6;
}

void CompetitionSupervisor::onOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const auto snapshot = msg;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    latest_odom_ = msg;
  }
  const bool stop_gate = state_ == State::CANCEL_FOR_QR ||
    state_ == State::WAIT_STOP_AFTER_REVERSE ||
    (state_ == State::RETURN_FOLLOW && return_action_succeeded_);
  if (!stop_gate || !snapshot) {
    stopped_samples_ = 0;
    stationary_started_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    return;
  }
  if (std::abs(snapshot->twist.twist.linear.x) <= stationary_linear_threshold_mps_ &&
    std::abs(snapshot->twist.twist.angular.z) <= stationary_angular_threshold_radps_)
  {
    if (stopped_samples_ == 0) {
      stationary_started_ = rclcpp::Time(snapshot->header.stamp);
    }
    stopped_samples_ = std::min(stopped_samples_ + 1, 1000000);
  } else {
    stopped_samples_ = 0;
    stationary_started_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }
}

void CompetitionSupervisor::onCostmap(nav2_msgs::msg::Costmap::ConstSharedPtr msg)
{
  const auto payload = msg ? analyzeCostmap(*msg) : CostmapPayloadStatus{};
  bool emit_snapshot_diag = false;
  bool payload_valid = payload.valid();
  bool all_unknown = !msg || payload.all_unknown;
  bool global_geometry_valid = payload.geometry_valid;
  bool local_geometry_valid = false;
  bool local_payload_valid = false;
  bool local_all_unknown = true;
  double local_age_ms = std::numeric_limits<double>::infinity();
  std::uint64_t generation = 0U;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    ++global_costmap_receive_sequence_;
    latest_costmap_ = msg;  // depth-one immutable/latest-only snapshot
    latest_costmap_geometry_valid_ = payload.geometry_valid;
    latest_costmap_payload_valid_ = payload.valid();
    latest_costmap_all_unknown_ = !msg || payload.all_unknown;
    latest_costmap_receipt_ = now();
    latest_costmap_steady_receipt_ = std::chrono::steady_clock::now();
    if (qr_lane_bias_built_ && !qr_lane_bias_ready_ &&
      global_costmap_receive_sequence_ > qr_lane_bias_publish_sequence_)
    {
      qr_lane_bias_ready_ = true;
    }
    generation = global_costmap_receive_sequence_;
    local_payload_valid = latest_local_costmap_payload_valid_;
    local_all_unknown = latest_local_costmap_all_unknown_;
    local_geometry_valid = latest_local_costmap_geometry_valid_;
    local_age_ms = 1000.0 * steadyAgeSec(latest_local_costmap_steady_receipt_);
    if (diagnostics_path_geometry_enabled_ &&
      (last_global_snapshot_diag_ == std::chrono::steady_clock::time_point{} ||
      std::chrono::duration<double>(latest_costmap_steady_receipt_ -
      last_global_snapshot_diag_).count() >= 1.0))
    {
      last_global_snapshot_diag_ = latest_costmap_steady_receipt_;
      emit_snapshot_diag = true;
    }
  }
  if (emit_snapshot_diag) {
    RCLCPP_INFO(
      get_logger(),
      "costmap_snapshot global_snapshot_generation=%lu global_snapshot_age_ms=0.0 "
      "global_geometry_valid=%s global_data_valid=%s global_all_unknown=%s "
      "global_snapshot_admitted=%s local_snapshot_age_ms=%.1f "
      "local_geometry_valid=%s local_data_valid=%s local_all_unknown=%s",
      static_cast<unsigned long>(generation), global_geometry_valid ? "true" : "false",
      payload_valid ? "true" : "false", all_unknown ? "true" : "false",
      (global_geometry_valid && payload_valid && !all_unknown) ? "true" : "false",
      local_age_ms, local_geometry_valid ? "true" : "false",
      local_payload_valid ? "true" : "false", local_all_unknown ? "true" : "false");
  }
}

void CompetitionSupervisor::onLocalCostmap(
  nav2_msgs::msg::Costmap::ConstSharedPtr msg)
{
  const auto payload = msg ? analyzeCostmap(*msg) : CostmapPayloadStatus{};
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  ++local_costmap_receive_sequence_;
  latest_local_costmap_ = std::move(msg);
  latest_local_costmap_geometry_valid_ = payload.geometry_valid;
  latest_local_costmap_payload_valid_ = payload.valid();
  latest_local_costmap_all_unknown_ = !latest_local_costmap_ || payload.all_unknown;
  latest_local_costmap_receipt_ = now();
  latest_local_costmap_steady_receipt_ = std::chrono::steady_clock::now();
}

void CompetitionSupervisor::onStaticMap(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  if (!msg) {return;}
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    latest_static_map_ = msg;
  }
  if (!qr_lane_bias_enabled_) {
    return;
  }
  if (qr_lane_bias_built_) {
    return;
  }
  if (msg->info.width != kQrLaneBiasWidth || msg->info.height != kQrLaneBiasHeight ||
    std::abs(msg->info.resolution - 0.01F) > 1.0e-6F ||
    std::abs(msg->info.origin.position.x) > 1.0e-6 ||
    std::abs(msg->info.origin.position.y) > 1.0e-6)
  {
    RCLCPP_ERROR(
      get_logger(), "QR lane bias map rejected: asset=%s map=%ux%u res=%.6f origin=(%.6f,%.6f)",
      qr_lane_bias_asset_loaded_ ? "valid" : qr_lane_bias_asset_error_.c_str(),
      msg->info.width, msg->info.height, msg->info.resolution,
      msg->info.origin.position.x, msg->info.origin.position.y);
    return;
  }
  if (qr_lane_bias_asset_loaded_ && qr_lane_bias_active_data_.size() == kQrLaneBiasBytes) {
    qr_lane_bias_active_map_.header.frame_id = msg->header.frame_id;
    qr_lane_bias_active_map_.info = msg->info;
    qr_lane_bias_active_map_.data.assign(
      reinterpret_cast<const std::int8_t *>(qr_lane_bias_active_data_.data()),
      reinterpret_cast<const std::int8_t *>(qr_lane_bias_active_data_.data() +
        qr_lane_bias_active_data_.size()));
  }
  qr_lane_bias_neutral_map_.header.frame_id = msg->header.frame_id;
  qr_lane_bias_neutral_map_.info = msg->info;
  qr_lane_bias_neutral_map_.data.assign(kQrLaneBiasBytes, static_cast<std::int8_t>(0));
  qr_lane_bias_built_ = true;
  setQrLaneBiasState(QrLaneBiasState::NEUTRAL);
  if (state_ == State::QR_SELECT || state_ == State::QR_PLAN ||
    state_ == State::QR_FOLLOW || state_ == State::QR_WAIT_CODE)
  {
    setQrLaneBiasState(QrLaneBiasState::ACTIVE);
  }
}

void CompetitionSupervisor::onCones(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  const auto reject = [this]() {
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      latest_cones_message_valid_ = false;
    };
  if (!msg || msg->header.frame_id.empty() || msg->is_bigendian) {
    reject(); return;
  }
  const rclcpp::Time source_stamp(msg->header.stamp);
  if (source_stamp.nanoseconds() == 0) {reject(); return;}
  int x_offset = -1;
  int y_offset = -1;
  for (const auto & field : msg->fields) {
    if (field.name == "x" && field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
      field.count >= 1U)
    {
      x_offset = static_cast<int>(field.offset);
    } else if (field.name == "y" &&
      field.datatype == sensor_msgs::msg::PointField::FLOAT32 && field.count >= 1U)
    {
      y_offset = static_cast<int>(field.offset);
    }
  }
  if (x_offset < 0 || y_offset < 0) {reject(); return;}
  const auto required_end = static_cast<std::size_t>(
    std::max(x_offset, y_offset)) + sizeof(float);
  if (msg->point_step < required_end) {reject(); return;}
  const std::size_t minimum_row_step =
    static_cast<std::size_t>(msg->width) * msg->point_step;
  if ((msg->width > 0U && msg->height == 0U) ||
    msg->row_step < minimum_row_step ||
    static_cast<std::size_t>(msg->row_step) * msg->height > msg->data.size())
  {
    reject(); return;
  }
  std::vector<std::pair<double, double>> centers;
  if (msg->width == 0U || msg->height == 0U) {
    if (!msg->data.empty()) {reject(); return;}
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    latest_cone_centers_map_.clear();
    latest_cones_receipt_ = now();
    latest_cones_source_stamp_ = source_stamp;
    latest_cones_message_valid_ = true;
    ++latest_cones_receive_sequence_;
    return;
  }
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
      map_frame_, msg->header.frame_id, source_stamp);
    const double yaw = yawOf(transform.transform.rotation);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    centers.reserve(static_cast<std::size_t>(msg->width) * msg->height);
    for (std::uint32_t row = 0U; row < msg->height; ++row) {
      for (std::uint32_t col = 0U; col < msg->width; ++col) {
        const std::size_t base = static_cast<std::size_t>(row) * msg->row_step +
          static_cast<std::size_t>(col) * msg->point_step;
        if (base + required_end > msg->data.size()) {reject(); return;}
        float x = 0.0F;
        float y = 0.0F;
        std::memcpy(&x, msg->data.data() + base + x_offset, sizeof(float));
        std::memcpy(&y, msg->data.data() + base + y_offset, sizeof(float));
        if (!std::isfinite(x) || !std::isfinite(y)) {continue;}
        centers.emplace_back(
          transform.transform.translation.x + c * x - s * y,
          transform.transform.translation.y + s * x + c * y);
      }
    }
  } catch (const tf2::TransformException &) {
    // Invalid layout or unavailable source-stamp TF is not fresh usable cone
    // evidence. A legal zero-point cloud still reaches the update below.
    reject(); return;
  }
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  latest_cone_centers_map_ = std::move(centers);  // empty is a valid fresh message
  latest_cones_receipt_ = now();
  latest_cones_source_stamp_ = source_stamp;
  latest_cones_message_valid_ = true;
  ++latest_cones_receive_sequence_;
}

void CompetitionSupervisor::onStartSignal(std_msgs::msg::String::ConstSharedPtr msg)
{
  if (!msg || (msg->data != "start" && msg->data != "go" && msg->data != "1")) {
    return;
  }
  if (start_mission_) {
    RCLCPP_WARN(get_logger(), "start signal ignored: mission is already enabled");
    return;
  }
  if (!startup_ready_) {
    publishAudit("start_signal_ignored", "startup_not_ready");
    RCLCPP_WARN(get_logger(), "start signal ignored: startup readiness is not valid");
    return;
  }
  start_mission_ = true;
  publishAudit("start_signal_accepted", "topic=/race/start");
  RCLCPP_INFO(get_logger(), "start signal accepted; starting mission");
}

void CompetitionSupervisor::onQr(hbmem_wechatcode_msgs::msg::QrCode::ConstSharedPtr msg)
{
  // The decoder may already be publishing while the operator is still
  // placing the vehicle.  Do not retain a pre-start QR observation and turn
  // it into an immediate handoff when the ROS start message arrives.
  if (!start_mission_) {return;}
  // QR is an asynchronous one-shot event, not a readiness or costmap state.
  // A legal result is accepted in every mission state; motion readiness only
  // decides when the saved result may take control of the vehicle.
  if (!msg || !msg->recognized || qr_latched_.load(std::memory_order_acquire)) {return;}
  const rclcpp::Time source_stamp(msg->stamp, get_clock()->get_clock_type());
  const rclcpp::Time publish_stamp(msg->publish_stamp, get_clock()->get_clock_type());
  const auto receipt_now = now();
  const double source_age = (receipt_now - source_stamp).seconds();
  const double transport_age = (receipt_now - publish_stamp).seconds();
  // A recognized QR is the handoff event.  Source/publish timestamps are
  // Retain source/transport age for diagnostics only.  A slow decode must not
  // be rejected by a navigation freshness gate.
  int number = 0;
  try {
    std::size_t parsed = 0;
    number = std::stoi(msg->data, &parsed);
    if (parsed != msg->data.size() || number < 1 || number > 9999) {return;}
  } catch (const std::exception &) {return;}

  nav_msgs::msg::Odometry::ConstSharedPtr odom;
  Pose2D qr_pose;
  bool have_qr_pose = false;
  rclcpp::Time qr_pose_stamp(0, 0, get_clock()->get_clock_type());
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    odom = latest_odom_;
    qr_pose = latest_map_pose_;
    have_qr_pose = have_map_pose_;
    qr_pose_stamp = latest_map_pose_stamp_;
  }
  auto audit = std::make_shared<QrAudit>();
  audit->source_stamp = source_stamp;
  audit->receipt_stamp = now();
  audit->pose = have_qr_pose ? qr_pose : Pose2D{};
  audit->pose_valid = have_qr_pose && qr_pose_stamp.nanoseconds() != 0 &&
    (audit->receipt_stamp - qr_pose_stamp).seconds() <= snapshot_tf_hard_age_sec_;
  if (odom) {
    audit->linear_speed = odom->twist.twist.linear.x;
    audit->angular_speed = odom->twist.twist.angular.z;
  }
  audit->cx = msg->cx; audit->cy = msg->cy;
  audit->width = msg->width; audit->height = msg->height;
  audit->generation = arbiter_->generation();
  audit->raw = msg->data; audit->number = number;
  if (qr_latched_.exchange(true, std::memory_order_acq_rel)) {return;}
  // Freeze the scan-time pose exactly once.  QR handoff must not replace this
  // anchor with a later pose just because readiness or TF refreshed while the
  // vehicle was braking.
  qr_frozen_pose_ = audit->pose;
  qr_frozen_pose_valid_ = audit->pose_valid;
  qr_audit_ = audit;
  clockwise_ = (number % 2) == 1;  // judge manual: odd=CW, even=CCW
  std_msgs::msg::String direction;
  direction.data = clockwise_ ? "clockwise" : "counterclockwise";
  direction_pub_->publish(direction);
  competition_display_started_ = true;
  publishCompetitionDisplay("识别中...");
  std::ostringstream detail;
  detail << "number=" << number << ",generation=" << audit->generation
         << ",pose=" << audit->pose.x << "," << audit->pose.y << "," << audit->pose.yaw
         << ",pose_valid=" << (audit->pose_valid ? "true" : "false")
         << ",velocity=" << audit->linear_speed << "," << audit->angular_speed
         << ",source_age_sec=" << source_age
         << ",transport_age_sec=" << transport_age
         << ",bbox=" << audit->cx << "," << audit->cy << ","
         << audit->width << "," << audit->height;
  publishAudit("qr_latched", detail.str());
  maybeBeginQrHandoff();
}

void CompetitionSupervisor::maybeBeginQrHandoff()
{
  if (!qr_latched_.load(std::memory_order_acquire) || !motion_stack_ready_ ||
    qr_handoff_started_ || !qr_audit_) {
    return;
  }
  const bool qr_motion_state = state_ == State::WAIT_READY ||
    state_ == State::QR_SELECT || state_ == State::QR_PLAN ||
    state_ == State::QR_FOLLOW || state_ == State::QR_WAIT_CODE;
  if (!qr_motion_state) {return;}

  if (!qr_frozen_pose_valid_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "qr_handoff_wait: legal QR saved; waiting for one valid scan-time pose");
    return;
  }

  qr_handoff_started_ = true;
  direct_gate_query_attempted_ = false;
  direct_gate_qr_pose_ = qr_frozen_pose_;
  gate_candidate_index_ = 0U;
  reverse_trigger_reason_.clear();
  stopped_samples_ = 0;
  stationary_started_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

  // QR latch is the only boundary that cancels the QR motion.  Costmap
  // readiness never cancels or stops the decoder; it only controls planning.
  invalidateWorker();
  ++active_qr_path_revision_;
  arbiter_->cancelQrFollowPathAsync("QR latched; stop QR motion immediately");
  setQrLaneBiasState(QrLaneBiasState::NEUTRAL);
  enter(State::CANCEL_FOR_QR, "qr_cancel_requested");
  requestDirectGateQuery();
  publishAudit(
    "qr_handoff_requested",
    "scan_pose_frozen=true; motion readiness satisfied; QR path canceled; reverse worker queued");
}

void CompetitionSupervisor::onVlm(ollama_msgs::msg::OllamaResponse::ConstSharedPtr msg)
{
  if (!vlm_triggered_ || vlm_received_ || !msg->done || msg->response.empty()) {return;}
  vlm_received_ = true;
  vlm_response_ = msg->response;
  if (competition_display_started_) {
    publishCompetitionDisplay(vlm_response_);
  }
  publishAudit("vlm_result", msg->response);
  publishGates();
}

void CompetitionSupervisor::onBevAccepted(
  geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  if (msg && msg->header.frame_id == map_frame_ && state_ == State::CHANNEL_FOLLOW) {
    // Latest-only event flag.  Reprojection and corridor audit run from the
    // supervisor tick, never in the localization callback.
    bev_correction_pending_.store(true, std::memory_order_release);
  }
}

void CompetitionSupervisor::onBevCorrectionEvent(
  std_msgs::msg::String::ConstSharedPtr msg)
{
  if (!msg || state_ != State::CHANNEL_FOLLOW) {return;}
  if (msg->data.find("DISCARDED") != std::string::npos) {
    publishAudit("bev_correction_discarded", msg->data);
  }
}

void CompetitionSupervisor::onSteeringAngle(std_msgs::msg::Float32::ConstSharedPtr msg)
{
  if (!msg || !std::isfinite(msg->data)) {return;}
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  const auto receipt = now();
  const bool was_stale = steering_angle_stamp_.nanoseconds() == 0 ||
    (receipt - steering_angle_stamp_).seconds() > steering_feedback_timeout_sec_;
  const bool changed = !steering_angle_rad_ ||
    std::abs(*steering_angle_rad_ - static_cast<double>(msg->data)) > 0.5 * kPi / 180.0;
  if (was_stale || changed) {++steering_epoch_;}
  steering_angle_rad_ = static_cast<double>(msg->data);
  steering_angle_stamp_ = receipt;
}

void CompetitionSupervisor::onSafeCommand(
  geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  if (!msg || !std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {return;}
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  const auto receipt = now();
  // /cmd_vel_safe remains useful for speed diagnostics and Reverse's speed
  // envelope, but it is deliberately not converted into a steering estimate:
  // after the QR handover the observed servo is treated as centred.
  latest_safe_cmd_ = *msg;
  latest_safe_cmd_stamp_ = receipt;
}

std::optional<std::pair<double, double>>
CompetitionSupervisor::steeringEstimateLocked(
  const rclcpp::Time & stamp, std::string & source) const
{
  (void)stamp;
  if (qr_latched_.load(std::memory_order_acquire)) {
    source = "chassis_auto_center_contract";
    return std::pair<double, double>{0.0, 0.25 * kPi / 180.0};
  }
  source = "QR not latched";
  return std::nullopt;
}

bool CompetitionSupervisor::stationaryReady() const
{
  nav_msgs::msg::Odometry::ConstSharedPtr odom;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    odom = latest_odom_;
  }
  if (!odom || stopped_samples_ < stationary_minimum_samples_ ||
    stationary_started_.nanoseconds() == 0)
  {
    return false;
  }
  const auto stamp = rclcpp::Time(odom->header.stamp);
  return (now() - stamp).seconds() <= stationary_maximum_odom_age_sec_ &&
    (stamp - stationary_started_).seconds() >= stationary_minimum_duration_sec_ &&
    std::abs(odom->twist.twist.linear.x) <= stationary_linear_threshold_mps_ &&
    std::abs(odom->twist.twist.angular.z) <= stationary_angular_threshold_radps_;
}

bool CompetitionSupervisor::lifecycleActive(
  LifecycleProbe & probe, const std::string & node_name,
  std::string & reason)
{
  using GetState = lifecycle_msgs::srv::GetState;
  if (probe.active) {return true;}
  if (!probe.client || !probe.client->service_is_ready()) {
    reason = "waiting for " + node_name + " lifecycle service";
    return false;
  }
  if (probe.future.valid()) {
    if (probe.future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      reason = "waiting for " + node_name + " lifecycle state";
      return false;
    }
    try {
      const auto response = probe.future.get();
      probe.future = rclcpp::Client<GetState>::SharedFuture{};
      probe.active = response &&
        response->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
      if (!probe.active) {
        reason = node_name + " lifecycle state is not ACTIVE";
      }
      return probe.active;
    } catch (const std::exception & ex) {
      probe.future = rclcpp::Client<GetState>::SharedFuture{};
      reason = node_name + " lifecycle query failed: " + ex.what();
      return false;
    }
  }
  probe.future = probe.client->async_send_request(
    std::make_shared<GetState::Request>()).share();
  reason = "querying " + node_name + " lifecycle state";
  return false;
}

bool CompetitionSupervisor::ready(std::string & reason)
{
  nav_msgs::msg::Odometry::ConstSharedPtr odom;
  nav2_msgs::msg::Costmap::ConstSharedPtr costmap;
  nav2_msgs::msg::Costmap::ConstSharedPtr local_costmap;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr static_map;
  bool have_map_pose = false;
  bool costmap_payload_valid = false;
  bool local_costmap_payload_valid = false;
  rclcpp::Time map_pose_stamp(0, 0, get_clock()->get_clock_type());
  std::chrono::steady_clock::time_point costmap_receipt;
  std::chrono::steady_clock::time_point local_costmap_receipt;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    odom = latest_odom_; costmap = latest_costmap_;
    local_costmap = latest_local_costmap_;
    static_map = latest_static_map_;
    costmap_payload_valid = latest_costmap_payload_valid_;
    local_costmap_payload_valid = latest_local_costmap_payload_valid_;
    have_map_pose = have_map_pose_; map_pose_stamp = latest_map_pose_stamp_;
    costmap_receipt = latest_costmap_steady_receipt_;
    local_costmap_receipt = latest_local_costmap_steady_receipt_;
  }
  if (mission_scope_ != "qr_reverse_gate" && !channel_runtime_admitted_) {
    reason = "fixed channel template lacks on-car runtime admission"; return false;
  }
  if (!odom || !costmap || !local_costmap || !static_map ||
    !costmap_payload_valid || !local_costmap_payload_valid) {
    reason = "waiting for static map and latest odom/global/local costmap snapshots";
    return false;
  }
  if (std::abs(static_map->info.resolution - 0.01) > 1.0e-6 ||
    std::abs(costmap->metadata.resolution - 0.01) > 1.0e-6)
  {
    reason = "runtime static/global costmap resolution must be exactly 0.01m";
    return false;
  }
  if (std::abs(local_costmap->metadata.resolution - 0.03) > 0.005) {
    reason = "local costmap resolution must remain approximately 0.03m";
    return false;
  }
  if (cones_sub_->get_publisher_count() == 0U) {
    reason = "confirmed cone-center publisher unavailable"; return false;
  }
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    if (!latest_cones_message_valid_ || latest_cones_receipt_.nanoseconds() == 0 ||
      (now() - latest_cones_receipt_).seconds() > cones_freshness_sec_)
    {
      reason = "invalid or stale /cones/points (legal empty clouds are accepted)";
      return false;
    }
  }
  if ((now() - rclcpp::Time(odom->header.stamp)).seconds() >
    snapshot_odom_hard_age_sec_)
  {
    reason = "stale odom"; return false;
  }
  const auto odom_publishers = get_publishers_info_by_topic("/odom");
  if (odom_publishers.size() != 1U ||
    odom_publishers.front().node_name() != "ekf_fusion_node")
  {
    std::ostringstream detail;
    detail << "expected exactly one /odom publisher from ekf_fusion_node, found="
           << odom_publishers.size();
    for (const auto & publisher : odom_publishers) {
      detail << ",publisher=" << publisher.node_namespace()
             << publisher.node_name();
    }
    reason = detail.str();
    return false;
  }
  if (steadyAgeSec(costmap_receipt) > snapshot_global_hard_age_sec_)
  {
    reason = "stale global costmap snapshot"; return false;
  }
  if (steadyAgeSec(local_costmap_receipt) > snapshot_local_hard_age_sec_)
  {
    reason = "stale local costmap snapshot"; return false;
  }
  if (!have_map_pose ||
    (now() - map_pose_stamp).seconds() > snapshot_tf_hard_age_sec_)
  {
    reason = "stale map->base_link pose snapshot"; return false;
  }
  if (!arbiter_->serversReady()) {reason = "Nav2 compute/follow action unavailable"; return false;}
  if (!lifecycleActive(planner_state_probe_, "planner_server", reason) ||
    !lifecycleActive(controller_state_probe_, "controller_server", reason) ||
    !lifecycleActive(smoother_state_probe_, "velocity_smoother", reason))
  {
    return false;
  }
  const auto safe_publishers = get_publishers_info_by_topic("/cmd_vel_safe");
  if (safe_publishers.size() != 1U ||
    safe_publishers.front().node_name() != "competition_command_limiter")
  {
    std::ostringstream detail;
    detail << "expected exactly one /cmd_vel_safe publisher from competition_command_limiter, found="
           << safe_publishers.size();
    if (!safe_publishers.empty()) {
      detail << ",first=" << safe_publishers.front().node_name();
    }
    reason = detail.str();
    return false;
  }
  if (!tf_buffer_.canTransform(map_frame_, base_frame_, tf2::TimePointZero) ||
    !tf_buffer_.canTransform(base_frame_, camera_frame_, tf2::TimePointZero))
  {
    reason = "required map/base/camera TF unavailable"; return false;
  }
  reason.clear(); return true;
}

void CompetitionSupervisor::onTick()
{
  drainWorkerCompletion();
  refreshMapPose();
  if (state_ == State::WAIT_READY) {
    std::string reason;
    const bool stack_ready = ready(reason);
    if (stack_ready != startup_ready_) {
      startup_ready_ = stack_ready;
      publishStartupReady(stack_ready);
      if (stack_ready) {
        publishAudit("startup_ready", "waiting_for_start_signal");
      } else {
        publishAudit("startup_not_ready", reason);
      }
    }
    if (stack_ready && start_mission_) {
      motion_stack_ready_ = true;
      if (qr_latched_.load(std::memory_order_acquire)) {
        RCLCPP_INFO(get_logger(),
          "readiness_admitted: handing off saved QR without QR_SELECT");
        maybeBeginQrHandoff();
        return;
      }
      RCLCPP_INFO(get_logger(), "readiness_admitted: entering QR_SELECT");
      qr_stage_started_ = now();
      qr_retry_not_before_ = qr_stage_started_;
      enter(State::QR_SELECT, "ready");
    } else if (stack_ready) {
      motion_stack_ready_ = false;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "startup_ready: waiting for std_msgs/String 'start' on /race/start");
    } else {
      motion_stack_ready_ = false;
      // WAIT_READY runs at 20 Hz; keep this diagnostic bounded so it cannot
      // become another high-rate CPU/logging path during startup.  The
      // reason is the exact first failed gate, which is enough to diagnose a
      // missing map/costmap/TF/cone/QR publisher without dumping snapshots.
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "readiness_wait: %s", reason.c_str());
    }
    return;
  }
  // A legal QR result bypasses the initial costmap synchronization entirely.
  // The synchronization remains a prerequisite for a P->QR ComputePath, not
  // for receiving or latching the asynchronous QR result.
  if (qr_latched_.load(std::memory_order_acquire) && !qr_handoff_started_) {
    maybeBeginQrHandoff();
    if (qr_handoff_started_) {return;}
  }
  const bool qr_stage_active = state_ == State::QR_PLAN || state_ == State::QR_FOLLOW;
  if (qr_stage_active && !qr_latched_.load() &&
    qr_stage_started_.nanoseconds() != 0 &&
    (now() - qr_stage_started_).seconds() >= qr_stage_timeout_sec_)
  {
    // A time budget is diagnostic in race-continuation mode. Do not cancel a
    // path that may still be making progress and do not latch a terminal state.
    qr_stage_started_ = now();
    publishAudit(
      "qr_stage_budget_exceeded_continuing",
      "budget_sec=" + std::to_string(qr_stage_timeout_sec_) +
      ",mission_hold=false,active_action_preserved=true");
    return;
  }
  if (state_ == State::QR_FOLLOW && !qr_latched_.load()) {
    maybeStartQrMidcourseReplan();
  }
  if (state_ == State::GATE_FOLLOW && !gate_handoff_cancel_pending_) {
    std::string detail;
    if (gateHandoffRegionReached(currentPose(), detail)) {
      gate_handoff_cancel_pending_ = true;
      publishAudit("gate_handoff_region_reached", detail);
      arbiter_->cancelAllAsync(
        "post-Reverse Gate handoff region reached; release Gate action");
      return;
    }
  }
  if (gate_handoff_cancel_pending_ && state_ == State::GATE_FOLLOW) {
    if (!arbiter_->controlsReleased()) {
      return;
    }
    gate_handoff_cancel_pending_ = false;
    publishAudit(
      "gate_handoff_cancel_released",
      "expected_cancel=true,controls_released=true,entry_planning=true");
    startPostGateFlow();
    return;
  }
  if (state_ == State::QR_SELECT) {
    if (qr_latched_.load(std::memory_order_acquire)) {
      maybeBeginQrHandoff();
      return;
    }
    if (!qr_fixed_goals_built_ && now() >= qr_retry_not_before_) {
      bool global_costmap_ready = false;
      std::uint64_t global_generation = 0U;
      {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        global_costmap_ready = latest_costmap_ && latest_costmap_payload_valid_ &&
          steadyAgeSec(latest_costmap_steady_receipt_) <= snapshot_global_hard_age_sec_;
        global_generation = global_costmap_receive_sequence_;
      }
      if (!global_costmap_ready) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "qr_initial_costmap_wait: waiting for a fresh global costmap "
          "(global_generation=%lu)",
          static_cast<unsigned long>(global_generation));
        return;
      }
      publishAudit(
        "qr_initial_global_costmap_ready",
        "global_costmap_generation=" + std::to_string(global_generation) +
        ",cone_topic_consulted=false");
      qr_fixed_goals_built_ = true;
      const auto result = makeFixedQrGoals();
      qr_ranked_ = result.candidates;
      qr_candidate_index_ = 0U;
      qr_plan_attempts_ = 0;
      std::ostringstream detail;
      detail << std::fixed << std::setprecision(3)
             << "mode=fixed_primary_backup"
             << ",selection_total_ms=" << result.selection_total_ms
             << ",candidates=";
      for (std::size_t i = 0; i < qr_ranked_.size(); ++i) {
        if (i > 0U) {detail << "|";}
        detail << qr_ranked_[i].id << "@(" << qr_ranked_[i].pose.x << ","
               << qr_ranked_[i].pose.y << "," << qr_ranked_[i].pose.yaw << ")";
      }
      publishAudit("qr_fixed_goal_candidates", detail.str());
      if (qr_ranked_.empty()) {
        retryQrSelectionWithoutHold(
          "fixed_goal_geometry_unavailable:" + result.failure_reason);
      } else {
        requestNextQrPlan();
      }
    }
  } else if (state_ == State::CANCEL_FOR_QR) {
    // onQr() starts the LUT query immediately.  This branch is retained only
    // as a non-blocking safety net for a deferred executor callback.
    if (!direct_gate_query_attempted_) {requestDirectGateQuery();}
  } else if (state_ == State::DIRECT_GATE_QUERY) {
    if (!direct_gate_query_attempted_) {requestDirectGateQuery();}
  } else if (state_ == State::REVERSE_SELECT) {
    // Direct failure has already canceled the QR FollowPath. Build the
    // low-curvature reverse fan in one worker. Every radius is expanded at
    // every distance layer; the first feasible layer is selected without a
    // pre-Reverse Gate Smac request.
    if (selected_reverse_) {
      startSelectedReverse();
      return;
    }
    bool hard_snapshot_stale = false;
    {
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      const auto stamp = now();
      const bool odom_hard_stale = !latest_odom_ ||
        (stamp - rclcpp::Time(latest_odom_->header.stamp)).seconds() >
        snapshot_odom_hard_age_sec_;
      const bool tf_hard_stale = !have_map_pose_ ||
        (stamp - latest_map_pose_stamp_).seconds() > snapshot_tf_hard_age_sec_;
      const bool global_hard_stale = !latest_costmap_ || !latest_costmap_payload_valid_ ||
        (!race_continue_enabled_ &&
        steadyAgeSec(latest_costmap_steady_receipt_) > snapshot_global_hard_age_sec_);
      const bool local_hard_stale = !race_continue_enabled_ &&
        (!latest_local_costmap_ || !latest_local_costmap_payload_valid_ ||
        steadyAgeSec(latest_local_costmap_steady_receipt_) > snapshot_local_hard_age_sec_);
      hard_snapshot_stale = odom_hard_stale || tf_hard_stale ||
        global_hard_stale || local_hard_stale;
    }
    if (hard_snapshot_stale) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "reverse_waiting_for_fresh_snapshot: mission_hold=false");
      return;
    }
    if (!reverse_build_pending_) {
      if (!qr_frozen_pose_valid_) {
        publishAudit(
          "reverse_context_missing_continue_to_entry",
          "mission_hold=false,reason=missing_frozen_qr_pose");
        startPostGateFlow();
        return;
      }
      const Pose2D frozen_qr_pose = qr_frozen_pose_;
      reverse_build_pending_ = true;
      enqueueWorker([this, frozen_qr_pose]() {
        auto result = makeReverseCandidatesLayered(frozen_qr_pose);
        return [this, result = std::move(result)]() mutable {
          reverse_build_pending_ = false;
          if (state_ != State::REVERSE_SELECT) {return;}
          publishReverseSolveMetrics(result);
          // The solver is intentionally anchored to the immutable QR pose.
          // Normal TF/EKF updates during braking are not stale-result errors;
          // only the mission/worker generation can invalidate this closure.
          reverse_candidates_ = std::move(result.candidates);
          selected_reverse_.reset();
          nominal_reverse_.reset();
          reverse_candidate_index_ = 0U;
          reverse_retry_remaining_distance_m_.reset();
          reverse_execution_distance_m_ = 0.0;
          if (reverse_candidates_.empty()) {
            publishAudit(
              "reverse_no_safe_candidate_continue_forward",
              "mission_hold=false,reason=" + (result.failure_reason.empty() ?
              "no safe reverse candidate" : result.failure_reason));
            requestPostReverseForwardPath();
          } else {
            nominal_reverse_ = reverse_candidates_.front();
            selected_reverse_ = *nominal_reverse_;
            publishAudit(
              "reverse_entry_selected",
              "mode=distance_layer_search,family=" + selected_reverse_->family +
              ",distance_m=" + std::to_string(selected_reverse_->command_distance) +
              ",command_endpoint=" + std::to_string(selected_reverse_->command_endpoint.x) +
              "," + std::to_string(selected_reverse_->command_endpoint.y) +
              "," + std::to_string(selected_reverse_->command_endpoint.yaw) +
              ",worst_case_stop=" + std::to_string(selected_reverse_->worst_case_stop.x) +
              "," + std::to_string(selected_reverse_->worst_case_stop.y) +
              "," + std::to_string(selected_reverse_->worst_case_stop.yaw) +
              ",gate_entry_lut=clean_nearest_only" +
              ",gate_goal_id=" + std::to_string(selected_reverse_->goal_id) +
              ",lut_match=" + (selected_reverse_->lut_match ?
                ("linear=" + std::to_string(selected_reverse_->lut_match->linear_index) +
                ",xy_error_m=" + std::to_string(selected_reverse_->lut_match->xy_error_m) +
                ",yaw_error_rad=" +
                std::to_string(selected_reverse_->lut_match->yaw_error_rad)) :
                "none") +
              ",gate_goal=" +
              std::to_string(selected_reverse_->gate_goal.x) + "," +
              std::to_string(selected_reverse_->gate_goal.y) + "," +
              std::to_string(selected_reverse_->gate_goal.yaw));
            startSelectedReverse();
          }
        };
      });
    }
  } else if (state_ == State::WAIT_STOP_AFTER_REVERSE && stationaryReady()) {
    if (reverse_execution_failed_) {
      reverse_execution_failed_ = false;
      const Pose2D actual = currentPose();
      double completed_distance = 0.0;
      if (selected_reverse_ && selected_reverse_->command_path.poses.size() >= 2U) {
        double cumulative = 0.0;
        double best_distance_sq = std::numeric_limits<double>::infinity();
        for (std::size_t i = 1U; i < selected_reverse_->command_path.poses.size(); ++i) {
          const auto & a = selected_reverse_->command_path.poses[i - 1U].pose.position;
          const auto & b = selected_reverse_->command_path.poses[i].pose.position;
          const double dx = b.x - a.x;
          const double dy = b.y - a.y;
          const double segment_sq = dx * dx + dy * dy;
          const double segment = std::sqrt(segment_sq);
          const double projection = segment_sq > 1.0e-12 ? std::clamp(
            ((actual.x - a.x) * dx + (actual.y - a.y) * dy) / segment_sq,
            0.0, 1.0) : 0.0;
          const double px = a.x + projection * dx;
          const double py = a.y + projection * dy;
          const double distance_sq = (actual.x - px) * (actual.x - px) +
            (actual.y - py) * (actual.y - py);
          if (distance_sq < best_distance_sq) {
            best_distance_sq = distance_sq;
            completed_distance = cumulative + projection * segment;
          }
          cumulative += segment;
        }
      }
      const double execution_distance = reverse_execution_distance_m_ > 0.0 ?
        reverse_execution_distance_m_ :
        (selected_reverse_ ? selected_reverse_->command_distance : 0.0);
      const double remaining_distance = std::max(
        0.0, execution_distance - completed_distance);
      publishAudit(
        "reverse_retry_progress",
        "reverse_nominal_distance=" + std::to_string(
          nominal_reverse_ ? nominal_reverse_->command_distance : 0.0) +
        ",reverse_execution_distance=" + std::to_string(execution_distance) +
        ",reverse_completed_distance=" + std::to_string(completed_distance) +
        ",reverse_remaining_distance=" + std::to_string(remaining_distance) +
        ",reverse_retry_count=" + std::to_string(reverse_recomputations_ + 1));
      if (reverse_actions_ < 2 && nominal_reverse_ && remaining_distance > 0.03) {
        // Retry only the measured suffix.  Do not round this back up to the
        // solver's 0.10 m search quantum: that would repeat distance already
        // travelled after an Action/TF abort.
        reverse_retry_remaining_distance_m_ = remaining_distance;
        selected_reverse_ = *nominal_reverse_;
        ++reverse_recomputations_;
        enter(State::REVERSE_SELECT, "reverse_retry_remaining_suffix");
      } else if (remaining_distance <= 0.03) {
        requestPostReverseForwardPath();
      } else {
        publishAudit(
          "reverse_retry_exhausted_continuing_forward",
          "remaining_distance_m=" + std::to_string(remaining_distance) +
          ",mission_hold=false");
        requestPostReverseForwardPath();
      }
    } else if (!selected_reverse_) {
      enter(State::REVERSE_SELECT, "reverse_stopped_without_candidate_reselect");
    } else {
      const auto actual = currentPose();
      const double position_error = distance(
        actual.x, actual.y, selected_reverse_->predicted_stop.x,
        selected_reverse_->predicted_stop.y);
      const double yaw_error = std::abs(normalize(
        actual.yaw - selected_reverse_->predicted_stop.yaw));
      std::ostringstream stop_detail;
      stop_detail << "position_error=" << position_error
                  << ",yaw_error=" << yaw_error;
      publishAudit("reverse_actual_stop", stop_detail.str());
      requestPostReverseForwardPath();
    }
  } else if (state_ == State::CHANNEL_FOLLOW) {
    if (pending_channel_retry_path_ && now() >= pending_channel_retry_not_before_) {
      auto retry = std::move(*pending_channel_retry_path_);
      const ChannelLane lane = pending_channel_retry_lane_;
      const ChannelActionKind kind = pending_channel_retry_kind_;
      // The old goal may have completed during the short retry window.  In
      // that case this is an ordinary send, not a hot replacement request.
      const bool replace_active =
        pending_channel_retry_replace_active_ && arbiter_->motionActive();
      pending_channel_retry_path_.reset();
      publishAudit(
        "channel_action_retry",
        "arbiter_available_retry=true,replace_active=" +
        std::string(replace_active ? "true" : "false"));
      if (kind == ChannelActionKind::ENTRY_CENTER && channel_entry_connector_active_) {
        startChannelEntryConnector(retry);
      } else {
        startChannelPath(retry, lane, kind, replace_active);
      }
      return;
    }
    updateChannelProgress();
  } else if (state_ == State::RETURN_FOLLOW) {
    updateReturnHandoffSpeedLimit();
    if (return_action_succeeded_ && stationaryReady()) {
      const double fraction = footprintFractionInParkingRegion(currentPose());
      std::ostringstream detail;
      detail << "footprint_fraction_in_P=" << fraction
             << ",vlm_received=" << vlm_received_;
      publishAudit("return_P_stationary_verification", detail.str());
      if (fraction + 1.0e-9 < parking_footprint_fraction_min_) {
        publishAudit(
          "return_P_insufficient",
          "single P target completed but footprint fraction is below configured threshold");
        enter(State::COMPLETE, "mission_complete_with_return_P_penalty");
      } else {
        enter(State::COMPLETE, vlm_received_ ?
          "mission_complete_all_subtasks" :
          "mission_complete_with_subtask2_vlm_penalty");
      }
    }
  }
}

void CompetitionSupervisor::refreshMapPose()
{
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    if (!have_map_pose_ && (now() - last_map_tf_probe_).seconds() < 0.50) {return;}
    last_map_tf_probe_ = now();
  }
  if (!tf_buffer_.canTransform(map_frame_, base_frame_, tf2::TimePointZero)) {return;}
  try {
    const auto transform = tf_buffer_.lookupTransform(
      map_frame_, base_frame_, tf2::TimePointZero);
    Pose2D pose{transform.transform.translation.x, transform.transform.translation.y,
      yawOf(transform.transform.rotation)};
    GateTransform2D map_to_local = latest_map_to_local_;
    bool have_local_transform = false;
    try {
      const auto local_tf = tf_buffer_.lookupTransform(
        odom_frame_, map_frame_, tf2::TimePointZero);
      map_to_local = GateTransform2D{
        local_tf.transform.translation.x, local_tf.transform.translation.y,
        yawOf(local_tf.transform.rotation)};
      have_local_transform = true;
    } catch (const tf2::TransformException &) {
      have_local_transform = false;
    }
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    const bool epoch_sensitive = state_ == State::QR_SELECT ||
      state_ == State::DIRECT_GATE_QUERY || state_ == State::CANCEL_FOR_QR ||
      state_ == State::REVERSE_SELECT ||
      state_ == State::WAIT_STOP_AFTER_REVERSE || state_ == State::GATE_PLAN;
    if (epoch_sensitive && (!have_map_pose_ || distance(
        pose.x, pose.y, latest_map_pose_.x, latest_map_pose_.y) > 0.03 ||
      std::abs(normalize(pose.yaw - latest_map_pose_.yaw)) > 2.0 * kPi / 180.0))
    {
      ++map_pose_epoch_;
    }
    latest_map_pose_ = pose; latest_map_pose_stamp_ = now(); have_map_pose_ = true;
    if (have_local_transform) {
      latest_map_to_local_ = map_to_local;
      latest_map_to_local_stamp_ = now();
    }
  } catch (const tf2::TransformException &) {
    // Keep the previous bounded-age immutable snapshot; readiness/tick users
    // fail closed when it becomes stale.
  }
}

void CompetitionSupervisor::enter(State state, const std::string & event)
{
  const bool was_qr_state = state_ == State::QR_SELECT ||
    state_ == State::QR_PLAN || state_ == State::QR_FOLLOW ||
    state_ == State::QR_WAIT_CODE;
  const bool is_qr_state = state == State::QR_SELECT ||
    state == State::QR_PLAN || state == State::QR_FOLLOW ||
    state == State::QR_WAIT_CODE;
  invalidateWorker();
  if (was_qr_state && !is_qr_state) {
    active_qr_path_.poses.clear();
    active_qr_goal_id_.clear();
    ++active_qr_path_revision_;
  }
  state_ = state; state_entered_ = now();
  publishEvent(event + ":" + stateName(state));
  if (qr_lane_bias_enabled_ && state == State::QR_SELECT &&
    qr_lane_bias_state_ != QrLaneBiasState::ACTIVE) {
    setQrLaneBiasState(QrLaneBiasState::ACTIVE);
  } else if (qr_lane_bias_enabled_ && state != State::QR_SELECT && state != State::QR_PLAN &&
    state != State::QR_FOLLOW && state != State::QR_WAIT_CODE &&
    qr_lane_bias_state_ == QrLaneBiasState::ACTIVE)
  {
    setQrLaneBiasState(QrLaneBiasState::NEUTRAL);
  }
  publishGates();
}

void CompetitionSupervisor::setQrLaneBiasState(QrLaneBiasState desired)
{
  if (!qr_lane_bias_enabled_) {
    return;
  }
  if (!qr_lane_bias_built_ || !qr_lane_bias_pub_ ||
    desired == QrLaneBiasState::UNINITIALIZED || desired == qr_lane_bias_state_) {
    return;
  }
  if (desired == QrLaneBiasState::ACTIVE &&
    (!qr_lane_bias_asset_loaded_ || qr_lane_bias_active_map_.data.size() != kQrLaneBiasBytes))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "QR lane bias unavailable; continuing QR navigation with neutral costmap");
    return;
  }
  auto & map = desired == QrLaneBiasState::ACTIVE ?
    qr_lane_bias_active_map_ : qr_lane_bias_neutral_map_;
  map.header.stamp = now();
  map.info.map_load_time = map.header.stamp;
  std::uint64_t publish_sequence = 0U;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    qr_lane_bias_publish_sequence_ = global_costmap_receive_sequence_;
    publish_sequence = qr_lane_bias_publish_sequence_;
    qr_lane_bias_ready_ = false;
    qr_lane_bias_state_ = desired;
    // Keep the state/sequence transition atomic with the one event-driven
    // publish.  A costmap callback cannot observe a pre-publication sample as
    // proof that StaticLayer consumed this map.
    qr_lane_bias_pub_->publish(map);
  }
  RCLCPP_INFO(
    get_logger(), "qr_lane_bias_publish state=%s bytes=%zu sequence=%lu",
    desired == QrLaneBiasState::ACTIVE ? "ACTIVE" : "NEUTRAL", map.data.size(),
    static_cast<unsigned long>(publish_sequence));
}

void CompetitionSupervisor::publishStartupReady(bool ready)
{
  std_msgs::msg::Bool message;
  message.data = ready;
  startup_ready_pub_->publish(message);
}

void CompetitionSupervisor::publishGates()
{
  std_msgs::msg::Bool bev, vlm, vlm_detector, reverse, direct;
  // Competition schedule: the BEV processes are launched with the common
  // stack, but their CPU inference and correction controls are separate.
  // Hall/QR/Direct/Reverse keep both OFF; Gate planning/following, the
  // complete channel, and Return open both gates. The matcher/backend still
  // apply their existing freshness/stability acceptance gates before any
  // map->odom correction is published.
  const bool bev_stage = state_ == State::GATE_PLAN || state_ == State::GATE_FOLLOW ||
    state_ == State::CHANNEL_FOLLOW || state_ == State::RETURN_PLAN ||
    state_ == State::RETURN_FOLLOW;
  const bool bev_compute = bev_stage;
  bev.data = bev_stage;
  vlm.data = state_ == State::CHANNEL_FOLLOW && vlm_cache_armed_ && !vlm_received_;
  vlm_detector.data = vlm.data && vlm_capture_window_active_;
  reverse.data = state_ == State::REVERSE_FOLLOW;
  direct.data = state_ == State::DIRECT_GATE_FOLLOW;
  const auto publish_if_changed = [](
    const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & publisher,
    bool value, int & previous)
    {
      const int encoded = value ? 1 : 0;
      if (previous == encoded) {return;}
      std_msgs::msg::Bool message;
      message.data = value;
      publisher->publish(message);
      previous = encoded;
    };
  publish_if_changed(bev_compute_gate_pub_, bev_compute, last_bev_compute_gate_);
  publish_if_changed(bev_gate_pub_, bev.data, last_bev_gate_);
  publish_if_changed(vlm_gate_pub_, vlm.data, last_vlm_gate_);
  publish_if_changed(
    vlm_detector_gate_pub_, vlm_detector.data, last_vlm_detector_gate_);
  publish_if_changed(reverse_gate_pub_, reverse.data, last_reverse_gate_);
  publish_if_changed(direct_gate_pub_, direct.data, last_direct_gate_);
}

void CompetitionSupervisor::publishCompetitionDisplay(const std::string & vlm_result)
{
  std_msgs::msg::String display;
  std::ostringstream text;
  text << "二维码：" << (qr_audit_ ? qr_audit_->raw : std::string("未识别"))
       << "    方向：";
  if (qr_audit_) {
    text << (clockwise_ ? "顺时针" : "逆时针");
  } else {
    text << "未识别";
  }
  text << "\n大模型场景描述：\n" << vlm_result;
  display.data = text.str();
  display_pub_->publish(display);
}

void CompetitionSupervisor::publishEvent(const std::string & event)
{
  std_msgs::msg::String msg; msg.data = event; event_pub_->publish(msg);
}

void CompetitionSupervisor::publishAudit(const std::string & event, const std::string & detail)
{
  if (competition_quiet_mode_) {return;}
  std_msgs::msg::String msg;
  std::ostringstream out;
  out << std::fixed << std::setprecision(6) << "stamp=" << now().seconds()
      << ",event=" << event << ",state=" << stateName(state_)
      << ",generation=" << (arbiter_ ? arbiter_->generation() : 0U)
      << ",detail=" << detail;
  msg.data = out.str();
  audit_pub_->publish(msg);
}

CompetitionSupervisor::Pose2D CompetitionSupervisor::currentPose() const
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return have_map_pose_ ? latest_map_pose_ : Pose2D{};
}

bool CompetitionSupervisor::gateHandoffRegionReached(
  const Pose2D & pose, std::string & detail) const
{
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw)) {
    detail = "finite_pose=false";
    return false;
  }
  const bool reached = pose.x >= gate_region_.x_min && pose.x <= gate_region_.x_max &&
    pose.y >= gate_region_.y_min && pose.y <= gate_region_.y_max;
  std::ostringstream text;
  text << std::fixed << std::setprecision(3)
       << "pose=" << pose.x << "," << pose.y << "," << pose.yaw
       << ",target=" << kPostReverseGateHandoffX << ","
       << kPostReverseGateHandoffY << "," << kPostReverseGateHandoffYaw
       << ",xy_box=[" << gate_region_.x_min << "," << gate_region_.x_max
       << "]x[" << gate_region_.y_min << "," << gate_region_.y_max << "]"
       << ",yaw_ignored=true";
  detail = text.str();
  return reached;
}

geometry_msgs::msg::PoseStamped CompetitionSupervisor::poseStamped(const Pose2D & pose) const
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = map_frame_; result.header.stamp = now();
  result.pose.position.x = pose.x; result.pose.position.y = pose.y;
  result.pose.orientation = quaternion(pose.yaw);
  return result;
}

CompetitionSupervisor::QrRankResult CompetitionSupervisor::makeFixedQrGoals()
{
  QrRankResult result;
  const auto started = std::chrono::steady_clock::now();
  geometry_msgs::msg::TransformStamped camera_tf;
  try {
    camera_tf = tf_buffer_.lookupTransform(
      base_frame_, camera_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    result.failure_reason = std::string("fixed QR goal camera TF lookup failed: ") + ex.what();
    return result;
  }

  tf2::Quaternion camera_q(
    camera_tf.transform.rotation.x, camera_tf.transform.rotation.y,
    camera_tf.transform.rotation.z, camera_tf.transform.rotation.w);
  const auto optical_forward_base =
    tf2::Matrix3x3(camera_q) * tf2::Vector3(0.0, 0.0, 1.0);
  const double optical_xy_norm = std::hypot(
    optical_forward_base.x(), optical_forward_base.y());
  if (!std::isfinite(optical_xy_norm) || optical_xy_norm < 0.20) {
    result.failure_reason = "fixed QR goal camera optical axis is not horizontal enough";
    return result;
  }
  const double optical_yaw_in_base = std::atan2(
    optical_forward_base.y(), optical_forward_base.x());

  const auto make_goal = [&](const char * id, double x, double y) {
      QrObservationCandidate candidate;
      candidate.id = id;
      candidate.region = id;
      candidate.pose.x = x;
      candidate.pose.y = y;
      // The camera translation is expressed in base_link.  Since the base
      // yaw is the unknown we solve the small fixed-point equation a few
      // times; this preserves the calibrated camera offset without using the
      // historical lane tangent or a hard-coded yaw.
      candidate.pose.yaw = normalize(
        std::atan2(qr_board_center_y_ - y, qr_board_center_x_ - x) -
        optical_yaw_in_base);
      for (int iteration = 0; iteration < 4; ++iteration) {
        const double c = std::cos(candidate.pose.yaw);
        const double s = std::sin(candidate.pose.yaw);
        const double camera_x = candidate.pose.x +
          c * camera_tf.transform.translation.x -
          s * camera_tf.transform.translation.y;
        const double camera_y = candidate.pose.y +
          s * camera_tf.transform.translation.x +
          c * camera_tf.transform.translation.y;
        candidate.pose.yaw = normalize(
          std::atan2(qr_board_center_y_ - camera_y,
            qr_board_center_x_ - camera_x) - optical_yaw_in_base);
      }
      return candidate;
    };

  auto primary_goal = make_goal("primary", qr_primary_x_, qr_primary_y_);
  auto backup_goal = make_goal("backup", qr_backup_x_, qr_backup_y_);
  // The exact optical-axis yaw at the lower Backup is about 13 degrees more
  // leftward than at Primary. In the physical x=1.6 replan that terminal yaw
  // made Hybrid-A* pass the Backup, execute a 2 m U-turn, and approach it a
  // second time. Primary's camera-calibrated yaw still leaves the board only
  // about 13 degrees off axis from Backup, while permitting a direct Ackermann
  // approach. Keep the same QR goal checker; change the pathological goal
  // geometry rather than globally weakening arrival tolerances.
  backup_goal.pose.yaw = primary_goal.pose.yaw;
  result.candidates.push_back(std::move(primary_goal));
  result.candidates.push_back(std::move(backup_goal));
  result.light_candidate_checks = result.candidates.size();
  result.selection_total_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  const auto & primary = result.candidates[0].pose;
  const auto & backup = result.candidates[1].pose;
  std::ostringstream detail;
  detail << std::fixed << std::setprecision(4)
         << "primary=(" << primary.x << "," << primary.y << "," << primary.yaw
         << "),backup=(" << backup.x << "," << backup.y << "," << backup.yaw
         << "),board=(" << qr_board_center_x_ << "," << qr_board_center_y_
         << "),camera_frame=" << camera_frame_
         << ",backup_yaw_source=primary_camera_alignment";
  publishAudit("qr_fixed_goal_geometry", detail.str());
  return result;
}

bool CompetitionSupervisor::validateNav2PathLight(
  const nav_msgs::msg::Path & path, const Pose2D & expected_goal,
  std::string & reason, double yaw_tolerance_rad,
  bool require_current_costmap) const
{
  if (path.header.frame_id != map_frame_ || path.poses.size() < 2U) {
    reason = "Nav2 path is empty, too short, or not in map frame";
    return false;
  }
  double total_length = 0.0;
  for (std::size_t i = 0U; i < path.poses.size(); ++i) {
    const auto & pose = path.poses[i].pose;
    const auto finite = [](const double value) {return std::isfinite(value);};
    if (!finite(pose.position.x) || !finite(pose.position.y) ||
      !finite(pose.position.z) || !finite(pose.orientation.x) ||
      !finite(pose.orientation.y) || !finite(pose.orientation.z) ||
      !finite(pose.orientation.w))
    {
      reason = "Nav2 path contains a non-finite pose";
      return false;
    }
    const double quaternion_norm = std::hypot(
      std::hypot(pose.orientation.x, pose.orientation.y),
      std::hypot(pose.orientation.z, pose.orientation.w));
    if (quaternion_norm < 1.0e-6) {
      reason = "Nav2 path contains a zero quaternion";
      return false;
    }
    if (i > 0U) {
      const auto & previous = path.poses[i - 1U].pose.position;
      total_length += distance(
        previous.x, previous.y, pose.position.x, pose.position.y);
    }
  }
  // The fixed 5x2 m competition floor does not need a multi-loop Nav2 path.
  if (!std::isfinite(total_length) || total_length > 8.0) {
    reason = "Nav2 path length exceeds fixed-floor bound";
    return false;
  }
  const auto & terminal = path.poses.back().pose;
  const double terminal_yaw = normalize(yawOf(terminal.orientation));
  if (distance(terminal.position.x, terminal.position.y,
    expected_goal.x, expected_goal.y) > 0.20 ||
    std::abs(normalize(terminal_yaw - expected_goal.yaw)) > yaw_tolerance_rad)
  {
    reason = "Nav2 path terminal pose is outside the expected goal tolerance";
    return false;
  }
  nav2_msgs::msg::Costmap::ConstSharedPtr grid;
  bool payload_valid = false;
  std::chrono::steady_clock::time_point receipt;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    grid = latest_costmap_;
    payload_valid = latest_costmap_payload_valid_;
    receipt = latest_costmap_steady_receipt_;
  }
  // Return-to-P is the ordinary Smac -> MPPI route.  During the Channel to
  // Return handoff the global costmap publisher can legitimately have no new
  // generation even though ComputePathToPose has just produced a valid path.
  // Do not turn that transient snapshot age into a false "mission complete"
  // penalty.  Other stages retain the strict current-costmap admission gate.
  const bool qr_stage = state_ == State::QR_SELECT || state_ == State::QR_PLAN ||
    state_ == State::QR_FOLLOW || state_ == State::QR_WAIT_CODE;
  const bool relax_runtime_age = race_continue_enabled_ && !qr_stage;
  if (require_current_costmap &&
    (!grid || !payload_valid ||
    (!relax_runtime_age && steadyAgeSec(receipt) > snapshot_global_hard_age_sec_))) {
    reason = "latest global costmap snapshot unavailable or stale";
    return false;
  }
  const Pose2D terminal_pose{
    terminal.position.x, terminal.position.y, terminal_yaw};
  if (require_current_costmap && grid && payload_valid &&
    (relax_runtime_age || steadyAgeSec(receipt) <= snapshot_global_hard_age_sec_) &&
    !footprintFreeInGrid(terminal_pose, *grid)) {
    reason = "Nav2 path terminal footprint is not free in global costmap";
    return false;
  }
  reason.clear();
  return true;
}

bool CompetitionSupervisor::validateChannelPathLight(
  const nav_msgs::msg::Path & path, std::string & reason) const
{
  if (path.header.frame_id != map_frame_ || path.poses.size() < 2U) {
    reason = "Channel path is empty, too short, or not in map frame";
    return false;
  }
  double length = 0.0;
  for (std::size_t index = 0U; index < path.poses.size(); ++index) {
    const auto & pose = path.poses[index].pose;
    const double quaternion_norm = std::hypot(
      std::hypot(pose.orientation.x, pose.orientation.y),
      std::hypot(pose.orientation.z, pose.orientation.w));
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z) || !std::isfinite(pose.orientation.x) ||
      !std::isfinite(pose.orientation.y) || !std::isfinite(pose.orientation.z) ||
      !std::isfinite(pose.orientation.w) || quaternion_norm < 1.0e-6)
    {
      reason = "Channel path contains a non-finite or zero-quaternion pose";
      return false;
    }
    if (index == 0U) {continue;}
    const auto & previous = path.poses[index - 1U].pose;
    const double dx = pose.position.x - previous.position.x;
    const double dy = pose.position.y - previous.position.y;
    const double segment = std::hypot(dx, dy);
    if (!std::isfinite(segment) || segment > 0.15) {
      reason = "Channel path contains a non-finite or discontinuous segment";
      return false;
    }
    if (segment > 1.0e-4) {
      const double forward = dx * std::cos(yawOf(previous.orientation)) +
        dy * std::sin(yawOf(previous.orientation));
      if (forward < -0.01) {
        reason = "Channel path contains a reverse-directed segment";
        return false;
      }
      length += segment;
    }
  }
  if (!std::isfinite(length) || length < 0.05 || length > 20.0) {
    reason = "Channel path length is outside the fixed-course bound";
    return false;
  }
  reason.clear();
  return true;
}

bool CompetitionSupervisor::footprintFreeInGrid(
  const Pose2D & pose, const nav2_msgs::msg::Costmap & grid,
  GridFailureDiagnostic * const diagnostic) const
{
  const double front = gate_vehicle_geometry_.footprint_front_m +
    gate_vehicle_geometry_.footprint_padding_m;
  const double rear = -gate_vehicle_geometry_.footprint_rear_m -
    gate_vehicle_geometry_.footprint_padding_m;
  const double half = gate_vehicle_geometry_.footprint_half_width_m +
    gate_vehicle_geometry_.footprint_padding_m;
  constexpr double step = 0.02;
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  const int nx = static_cast<int>(std::ceil((front - rear) / step));
  const int ny = static_cast<int>(std::ceil(2.0 * half / step));
  for (int ix = 0; ix <= nx; ++ix) {
    const double local_x = rear + (front - rear) * ix / nx;
    for (int iy = 0; iy <= ny; ++iy) {
      const double local_y = -half + 2.0 * half * iy / ny;
      const double world_x = pose.x + c * local_x - s * local_y;
      const double world_y = pose.y + s * local_x + c * local_y;
      const int mx = static_cast<int>(std::floor(
        (world_x - grid.metadata.origin.position.x) / grid.metadata.resolution));
      const int my = static_cast<int>(std::floor(
        (world_y - grid.metadata.origin.position.y) / grid.metadata.resolution));
      if (mx < 0 || my < 0 || mx >= static_cast<int>(grid.metadata.size_x) ||
        my >= static_cast<int>(grid.metadata.size_y))
      {
        if (diagnostic) {
          diagnostic->valid = true;
          diagnostic->frame = grid.header.frame_id;
          diagnostic->checked_pose = pose;
          diagnostic->footprint_sample_x_m = local_x;
          diagnostic->footprint_sample_y_m = local_y;
          diagnostic->grid_frame_x_m = world_x;
          diagnostic->grid_frame_y_m = world_y;
          diagnostic->world_x_m = world_x;
          diagnostic->world_y_m = world_y;
          diagnostic->grid_x = mx;
          diagnostic->grid_y = my;
          diagnostic->cost = -1;
          diagnostic->reason = "outside";
        }
        return false;
      }
      const auto index = static_cast<std::size_t>(my) * grid.metadata.size_x +
        static_cast<std::size_t>(mx);
      if (index >= grid.data.size() || isHardObstacle(grid.data[index])) {
        if (diagnostic) {
          diagnostic->valid = true;
          diagnostic->frame = grid.header.frame_id;
          diagnostic->checked_pose = pose;
          diagnostic->footprint_sample_x_m = local_x;
          diagnostic->footprint_sample_y_m = local_y;
          diagnostic->grid_frame_x_m = world_x;
          diagnostic->grid_frame_y_m = world_y;
          diagnostic->world_x_m = world_x;
          diagnostic->world_y_m = world_y;
          diagnostic->grid_x = mx;
          diagnostic->grid_y = my;
          diagnostic->cost = index < grid.data.size() ? static_cast<int>(grid.data[index]) : -1;
          diagnostic->reason = index >= grid.data.size() ? "invalid_data_index" :
            (diagnostic->cost == 255 ? "255_unknown" : "254_lethal");
        }
        return false;
      }
    }
  }
  return true;
}

bool CompetitionSupervisor::qrGoalFootprintLegal(
  const QrObservationCandidate & candidate, std::string & reason) const
{
  nav2_msgs::msg::Costmap::ConstSharedPtr grid;
  bool payload_valid = false;
  std::chrono::steady_clock::time_point receipt;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    grid = latest_costmap_;
    payload_valid = latest_costmap_payload_valid_;
    receipt = latest_costmap_steady_receipt_;
  }
  if (!grid || !payload_valid) {
    reason = "latest global costmap unavailable";
    return false;
  }
  const double age_sec = steadyAgeSec(receipt);
  if (!std::isfinite(age_sec) || age_sec > snapshot_global_hard_age_sec_) {
    reason = "latest global costmap stale:age_sec=" + std::to_string(age_sec);
    return false;
  }
  return qrGoalFootprintLegal(candidate, *grid, reason);
}

bool CompetitionSupervisor::qrGoalFootprintLegal(
  const QrObservationCandidate & candidate,
  const nav2_msgs::msg::Costmap & grid, std::string & reason) const
{
  GridFailureDiagnostic diagnostic;
  if (!footprintFreeInGrid(candidate.pose, grid, &diagnostic)) {
    std::ostringstream detail;
    detail << "footprint_blocked:reason=" << diagnostic.reason
           << ",grid=" << diagnostic.grid_x << "," << diagnostic.grid_y
           << ",cost=" << diagnostic.cost;
    reason = detail.str();
    return false;
  }
  reason.clear();
  return true;
}

double CompetitionSupervisor::endpointClearanceInGrid(
  const Pose2D & pose, const nav2_msgs::msg::Costmap & grid) const
{
  // Endpoint-only bounded clearance proxy. This is deliberately not a path
  // sweep: at most five padded-footprint checks are performed for each of the
  // at-most-nine lightweight endpoints.
  const auto free_with_extra_padding = [this, &pose, &grid](double extra) {
      const double front = gate_vehicle_geometry_.footprint_front_m +
        gate_vehicle_geometry_.footprint_padding_m + extra;
      const double rear = -gate_vehicle_geometry_.footprint_rear_m -
        gate_vehicle_geometry_.footprint_padding_m - extra;
      const double half = gate_vehicle_geometry_.footprint_half_width_m +
        gate_vehicle_geometry_.footprint_padding_m + extra;
      constexpr double step = 0.02;
      const double c = std::cos(pose.yaw);
      const double s = std::sin(pose.yaw);
      const int nx = std::max(1, static_cast<int>(std::ceil((front - rear) / step)));
      const int ny = std::max(1, static_cast<int>(std::ceil(2.0 * half / step)));
      for (int ix = 0; ix <= nx; ++ix) {
        const double local_x = rear + (front - rear) * ix / nx;
        for (int iy = 0; iy <= ny; ++iy) {
          const double local_y = -half + 2.0 * half * iy / ny;
          const double x = pose.x + c * local_x - s * local_y;
          const double y = pose.y + s * local_x + c * local_y;
          const int mx = static_cast<int>(std::floor(
            (x - grid.metadata.origin.position.x) / grid.metadata.resolution));
          const int my = static_cast<int>(std::floor(
            (y - grid.metadata.origin.position.y) / grid.metadata.resolution));
          if (mx < 0 || my < 0 || mx >= static_cast<int>(grid.metadata.size_x) ||
            my >= static_cast<int>(grid.metadata.size_y))
          {
            return false;
          }
          const auto index = static_cast<std::size_t>(my) * grid.metadata.size_x +
            static_cast<std::size_t>(mx);
          if (index >= grid.data.size() || isHardObstacle(grid.data[index])) {
            return false;
          }
        }
      }
      return true;
    };
  double clearance = -1.0;
  for (const double extra : {0.0, 0.025, 0.050, 0.075, 0.100}) {
    if (!free_with_extra_padding(extra)) {break;}
    clearance = extra;
  }
  return clearance;
}

bool CompetitionSupervisor::auditReverseSegmentHardSafety(
  const Pose2D & previous, const Pose2D & current,
  const GateSearchSnapshot & snapshot, std::string & reason,
  GridFailureDiagnostic * const diagnostic) const
{
  nav_msgs::msg::Path segment;
  segment.header.frame_id = map_frame_;
  segment.header.stamp = now();
  segment.poses.push_back(poseStamped(previous));
  segment.poses.push_back(poseStamped(current));

  // The low-curvature reverse fan is generated from bounded analytic steering
  // primitives.  Do not
  // reject a branch because a 10 cm chord produces a coarse curvature or
  // steering-rate diagnostic; only hard occupancy and direction checks can
  // make a branch permanently dead.
  if (diagnostic) {
    *diagnostic = GridFailureDiagnostic{};
    diagnostic->source = "global";
    diagnostic->base_pose_map = previous;
  }
  if (!snapshot.global_costmap || !auditPathAgainstGrid(
      segment, true, *snapshot.global_costmap, reason, false, nullptr, diagnostic))
  {
    if (reason.empty()) {reason = "missing or invalid frozen global costmap";}
    return false;
  }
  if (!snapshot.local_costmap) {
    reason = "missing frozen local costmap";
    return false;
  }
  const auto & local = *snapshot.local_costmap;
  if (local.header.frame_id != odom_frame_ && local.header.frame_id != map_frame_) {
    reason = "frozen local costmap has unsupported frame";
    return false;
  }
  if (local.metadata.resolution <= 0.0F || local.metadata.size_x == 0U ||
    local.metadata.size_y == 0U || local.data.empty())
  {
    reason = "frozen local costmap geometry/data invalid";
    return false;
  }
  const auto local_footprint_free = [this, &local, &snapshot, diagnostic](
      const Pose2D & pose, const Pose2D & base_pose_map) {
      constexpr double sample_step = 0.02;
      const double front = gate_vehicle_geometry_.footprint_front_m +
        gate_vehicle_geometry_.footprint_padding_m;
      const double rear = gate_vehicle_geometry_.footprint_rear_m +
        gate_vehicle_geometry_.footprint_padding_m;
      const double half = gate_vehicle_geometry_.footprint_half_width_m +
        gate_vehicle_geometry_.footprint_padding_m;
      const int nx = std::max(1, static_cast<int>(std::ceil(
        (front + rear) / sample_step)));
      const int ny = std::max(1, static_cast<int>(std::ceil(
        2.0 * half / sample_step)));
      const double c = std::cos(pose.yaw);
      const double s = std::sin(pose.yaw);
      for (int ix = 0; ix <= nx; ++ix) {
        const double local_x = -rear + (front + rear) * ix / nx;
        for (int iy = 0; iy <= ny; ++iy) {
          const double local_y = -half + 2.0 * half * iy / ny;
          const double x = pose.x + c * local_x - s * local_y;
          const double y = pose.y + s * local_x + c * local_y;
          const int mx = static_cast<int>(std::floor(
            (x - local.metadata.origin.position.x) / local.metadata.resolution));
          const int my = static_cast<int>(std::floor(
            (y - local.metadata.origin.position.y) / local.metadata.resolution));
          if (mx < 0 || my < 0 ||
            mx >= static_cast<int>(local.metadata.size_x) ||
            my >= static_cast<int>(local.metadata.size_y))
          {
            continue;  // global remains authoritative outside rolling local
          }
          const auto index = static_cast<std::size_t>(my) * local.metadata.size_x +
            static_cast<std::size_t>(mx);
          if (index >= local.data.size() || isHardObstacle(local.data[index])) {
            if (diagnostic) {
              double map_x = x;
              double map_y = y;
              if (local.header.frame_id == odom_frame_) {
                const double tc = std::cos(snapshot.map_to_local.yaw);
                const double ts = std::sin(snapshot.map_to_local.yaw);
                const double dx = x - snapshot.map_to_local.x;
                const double dy = y - snapshot.map_to_local.y;
                map_x = tc * dx + ts * dy;
                map_y = -ts * dx + tc * dy;
              }
              diagnostic->valid = true;
              diagnostic->source = "local";
              diagnostic->frame = local.header.frame_id;
              diagnostic->checked_pose = pose;
              diagnostic->base_pose_map = base_pose_map;
              diagnostic->footprint_sample_x_m = local_x;
              diagnostic->footprint_sample_y_m = local_y;
              diagnostic->grid_frame_x_m = x;
              diagnostic->grid_frame_y_m = y;
              diagnostic->world_x_m = map_x;
              diagnostic->world_y_m = map_y;
              diagnostic->grid_x = mx;
              diagnostic->grid_y = my;
              diagnostic->cost = index < local.data.size() ?
                static_cast<int>(local.data[index]) : -1;
              diagnostic->reason = index >= local.data.size() ? "invalid_data_index" :
                (diagnostic->cost == 255 ? "255_unknown" : "254_lethal");
            }
            return false;
          }
        }
      }
      return true;
    };
  const double segment_length = distance(previous.x, previous.y, current.x, current.y);
  const int samples = std::max(1, static_cast<int>(std::ceil(segment_length / 0.015)));
  const double yaw_delta = normalize(current.yaw - previous.yaw);
  for (int sample = 0; sample <= samples; ++sample) {
    const double u = static_cast<double>(sample) / samples;
    const Pose2D base_pose_map{
      previous.x + u * (current.x - previous.x),
      previous.y + u * (current.y - previous.y),
      normalize(previous.yaw + u * yaw_delta)};
    Pose2D local_pose = base_pose_map;
    if (diagnostic) {
      diagnostic->segment_index = 0U;
      diagnostic->segment_count = 1U;
      diagnostic->sample_index = sample;
      diagnostic->sample_count = samples;
      diagnostic->base_pose_map = base_pose_map;
    }
    if (local.header.frame_id == odom_frame_) {
      const double c = std::cos(snapshot.map_to_local.yaw);
      const double s = std::sin(snapshot.map_to_local.yaw);
      const double map_x = local_pose.x;
      const double map_y = local_pose.y;
      local_pose.x = snapshot.map_to_local.x + c * map_x - s * map_y;
      local_pose.y = snapshot.map_to_local.y + s * map_x + c * map_y;
      local_pose.yaw = normalize(snapshot.map_to_local.yaw + local_pose.yaw);
    }
    if (!local_footprint_free(local_pose, base_pose_map)) {
      reason = "1.5cm frozen local-costmap hard footprint collision";
      return false;
    }
  }
  reason.clear();
  return true;
}

bool CompetitionSupervisor::auditPathAgainstGrid(
  const nav_msgs::msg::Path & path, bool reverse,
  const nav2_msgs::msg::Costmap & grid, std::string & reason,
  const bool enforce_curvature, std::string * const curvature_diagnostic,
  GridFailureDiagnostic * const diagnostic) const
{
  if (curvature_diagnostic) {curvature_diagnostic->clear();}
  if (path.header.frame_id != map_frame_ || path.poses.size() < 2U) {
    reason = "wrong frame or too few poses"; return false;
  }
  constexpr double sweep_step_m = 0.015;
  for (std::size_t i = 0; i + 1U < path.poses.size(); ++i) {
    const auto & a = path.poses[i].pose;
    const auto & b = path.poses[i + 1U].pose;
    const double segment = distance(
      a.position.x, a.position.y, b.position.x, b.position.y);
    const int samples = std::max(1, static_cast<int>(std::ceil(segment / sweep_step_m)));
    const double yaw_a = yawOf(a.orientation);
    const double yaw_delta = normalize(yawOf(b.orientation) - yaw_a);
    for (int sample = 0; sample <= samples; ++sample) {
      const double u = static_cast<double>(sample) / samples;
      const Pose2D swept{
        a.position.x + u * (b.position.x - a.position.x),
        a.position.y + u * (b.position.y - a.position.y),
        normalize(yaw_a + u * yaw_delta)};
      if (diagnostic) {
        diagnostic->segment_index = i;
        diagnostic->segment_count = path.poses.size() - 1U;
        diagnostic->sample_index = sample;
        diagnostic->sample_count = samples;
        diagnostic->base_pose_map = swept;
      }
      if (!footprintFreeInGrid(swept, grid, diagnostic)) {
        std::ostringstream detail;
        detail << std::fixed << std::setprecision(3)
               << "1.5cm swept footprint collision/unknown/outside costmap"
               << " segment=" << i << "/" << (path.poses.size() - 1U)
               << " sample=" << sample << "/" << samples
               << " pose=(" << swept.x << "," << swept.y << ","
               << swept.yaw * 180.0 / kPi << "deg)";
        reason = detail.str();
        return false;
      }
    }
  }
  const double max_curvature = 1.0 / min_turning_radius_ + 1.0e-3;
  double observed_max_curvature = 0.0;
  std::size_t observed_max_segment = 0U;
  for (std::size_t i = 1; i + 1 < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    const auto & c = path.poses[i + 1].pose.position;
    const double ab = distance(a.x, a.y, b.x, b.y);
    const double bc = distance(b.x, b.y, c.x, c.y);
    const double ac = distance(a.x, a.y, c.x, c.y);
    if (ab < 1e-4 || bc < 1e-4 || ac < 1e-4) {continue;}
    const double cross = std::abs((b.x - a.x) * (c.y - a.y) -
      (b.y - a.y) * (c.x - a.x));
    const double geometric_curvature = 2.0 * cross / (ab * bc * ac);
    if (geometric_curvature > observed_max_curvature) {
      observed_max_curvature = geometric_curvature;
      observed_max_segment = i;
    }
    if (enforce_curvature && geometric_curvature > max_curvature) {
      reason = "curvature exceeds 1/0.35"; return false;
    }
    const double heading = yawOf(path.poses[i].pose.orientation);
    const double projection = (c.x - b.x) * std::cos(heading) +
      (c.y - b.y) * std::sin(heading);
    if ((!reverse && projection < -0.01) || (reverse && projection > 0.01)) {
      reason = reverse ? "reverse path contains forward segment" : "forward path contains reverse segment";
      return false;
    }
  }
  if (curvature_diagnostic && observed_max_curvature > max_curvature) {
    std::ostringstream detail;
    detail << std::fixed << std::setprecision(4)
           << "geometric_max_curvature=" << observed_max_curvature
           << ",equivalent_radius=" << (1.0 / observed_max_curvature)
           << ",limit_radius=" << min_turning_radius_
           << ",segment=" << observed_max_segment
           << ",mode=diagnostic_only";
    *curvature_diagnostic = detail.str();
  }
  reason.clear(); return true;
}

void CompetitionSupervisor::requestNextQrPlan()
{
  if (qr_latched_.load()) {return;}
  if (qr_candidate_index_ >= qr_ranked_.size()) {
    retryQrSelectionWithoutHold("fixed_primary_and_backup_planning_failed");
    return;
  }
  const auto seed = qr_ranked_[qr_candidate_index_];
  const std::size_t candidate_index = qr_candidate_index_;
  ++qr_plan_attempts_; enter(State::QR_PLAN, "qr_plan_" + seed.id);
  const std::string action_tag = "qr:" + seed.id;
  publishAudit(
    candidate_index == 0U ? "qr_primary_plan_requested" : "qr_backup_plan_requested",
    action_tag + ":goal=" + std::to_string(seed.pose.x) + "," +
    std::to_string(seed.pose.y) + "," + std::to_string(seed.pose.yaw));
  if (!arbiter_->computePath(
    poseStamped(seed.pose), "GridBased", action_tag,
    [this, seed](std::uint64_t, bool ok, const nav_msgs::msg::Path & path,
      const std::string & detail) {
      if (qr_latched_.load() || state_ != State::QR_PLAN) {return;}
      std::string reason;
      if (!ok) {
        reason = detail.empty() ? "compute_path_failed" : detail;
        RCLCPP_ERROR(
          get_logger(),
          "QR plan rejected: seed=%s compute_ok=%s poses=%zu reason=%s",
          seed.id.c_str(), ok ? "true" : "false", path.poses.size(),
          reason.c_str());
        publishAudit(
          seed.id == "primary" ? "qr_primary_plan_failed" : "qr_backup_plan_failed",
          seed.id + ":" + reason);
        ++qr_candidate_index_;
        if (qr_candidate_index_ < qr_ranked_.size()) {
          requestNextQrPlan();
        } else {
          retryQrSelectionWithoutHold(seed.id + ":" + reason);
        }
        return;
      }
      if (!validateNav2PathLight(path, seed.pose, reason)) {
        publishAudit(
          seed.id == "primary" ? "qr_primary_plan_failed" : "qr_backup_plan_failed",
          seed.id + ":light_validation:" + reason);
        ++qr_candidate_index_;
        if (qr_candidate_index_ < qr_ranked_.size()) {
          requestNextQrPlan();
        } else {
          retryQrSelectionWithoutHold(seed.id + ":light_validation:" + reason);
        }
        return;
      }
      startQrPath(path, seed.id, seed.region);
    }))
  {
    publishAudit(
      seed.id == "primary" ? "qr_primary_plan_failed" : "qr_backup_plan_failed",
      seed.id + ":arbiter_refused");
    ++qr_candidate_index_;
    if (qr_candidate_index_ < qr_ranked_.size()) {
      requestNextQrPlan();
    } else {
      retryQrSelectionWithoutHold(seed.id + ":arbiter_refused");
    }
  }
}

void CompetitionSupervisor::retryQrSelectionWithoutHold(const std::string & reason)
{
  qr_fixed_goals_built_ = false;
  qr_ranked_.clear();
  qr_candidate_index_ = 0U;
  // A failed Primary+Backup pair can briefly coincide with an exact-time TF
  // backlog. Limit retries to 1 Hz so planning cannot starve the localization
  // and ConeLayer callbacks needed to make the next attempt succeed.
  qr_retry_not_before_ = now() + rclcpp::Duration::from_seconds(1.0);
  enter(State::QR_SELECT, "qr_recoverable_retry");
  publishAudit(
    "qr_retry_scheduled",
    "delay_sec=1.00,mission_hold=false,reason=" + reason);
}

void CompetitionSupervisor::startQrPath(
  const nav_msgs::msg::Path & path, const std::string & seed_id,
  const std::string & region, bool reset_midcourse)
{
  if (reset_midcourse) {
    qr_midcourse_triggered_ = false;
    qr_midcourse_plan_sent_ = false;
  }
  ++qr_motion_actions_;
  // Keep one compact geometry record per fixed QR attempt.  This audit is
  // intentionally observational and never participates in the one-shot
  // midcourse trigger or path replacement decision.
  if (diagnostics_path_geometry_enabled_ && path.poses.size() >= 2U) {
    const auto & first = path.poses.front().pose;
    const double first_yaw = yawOf(first.orientation);
    double travelled = 0.0;
    double max_lateral = 0.0;
    double max_yaw_delta = 0.0;
    for (std::size_t i = 1U; i < path.poses.size(); ++i) {
      const auto & previous = path.poses[i - 1U].pose;
      const auto & current = path.poses[i].pose;
      travelled += distance(
        previous.position.x, previous.position.y,
        current.position.x, current.position.y);
      if (travelled > 1.50) {break;}
      const double dx = current.position.x - first.position.x;
      const double dy = current.position.y - first.position.y;
      max_lateral = std::max(
        max_lateral, std::abs(-std::sin(first_yaw) * dx + std::cos(first_yaw) * dy));
      max_yaw_delta = std::max(
        max_yaw_delta, std::abs(normalize(yawOf(current.orientation) - first_yaw)));
    }
    std::ostringstream geometry;
    geometry << std::fixed << std::setprecision(3)
             << "seed=" << seed_id
             << ",poses=" << path.poses.size()
             << ",first_1p5m_max_lateral_m=" << max_lateral
             << ",first_1p5m_max_yaw_deg=" << max_yaw_delta * 180.0 / kPi
             << ",goal=" << path.poses.back().pose.position.x << ","
             << path.poses.back().pose.position.y << ","
             << yawOf(path.poses.back().pose.orientation) * 180.0 / kPi;
    publishAudit("qr_path_geometry", geometry.str());
  }
  const Pose2D motion_start = currentPose();
  enter(State::QR_FOLLOW, "qr_follow_" + seed_id);
  if (!arbiter_->sendFollowPath(
    path, "FollowPath", "QrRegionGoalChecker", "qr:" + seed_id,
    [this, motion_start, seed_id](
      std::uint64_t, rclcpp_action::ResultCode code, const std::string & tag) {
      if (qr_latched_.load() || state_ != State::QR_FOLLOW) {return;}
      const nav_msgs::msg::Path continuation = active_qr_path_;
      const std::string continuation_region = active_qr_goal_id_;
      active_qr_path_.poses.clear();
      active_qr_goal_id_.clear();
      ++active_qr_path_revision_;
      publishAudit("qr_motion_terminal", tag);
      const Pose2D terminal = currentPose();
      const double moved = distance(
        motion_start.x, motion_start.y, terminal.x, terminal.y);
      if (code == rclcpp_action::ResultCode::SUCCEEDED) {
        // A decoder result can arrive just after the fixed observation goal is
        // reached.  This is a recoverable timing race, not a mission failure:
        // keep the supervisor recoverable and let onQr() immediately
        // start the normal Direct-Gate/Reverse handoff when the late code is
        // published.  We intentionally do not drive blindly without a code.
        enter(State::QR_WAIT_CODE, "qr_goal_reached_waiting_for_code");
        publishAudit(
          "qr_waiting_for_code",
          "fixed_goal_reached=true,seed=" + seed_id +
          ",moved_m=" + std::to_string(moved));
      } else {
        if (race_continue_enabled_ && continuation.poses.size() >= 2U) {
          publishAudit(
            "qr_follow_abort_path_resubmitted",
            "seed=" + seed_id + ",mission_hold=false,new_smac=false,poses=" +
            std::to_string(continuation.poses.size()) + ",detail=" + tag);
          startQrPath(continuation, seed_id, continuation_region, false);
          return;
        }
        recoverQrFollowFailure(
          seed_id, "fixed_follow_failed:moved_m=" + std::to_string(moved) +
          ":detail=" + tag);
      }
    }))
  {
    publishAudit("qr_follow_arbiter_refused", seed_id);
    ++qr_candidate_index_;
    if (qr_candidate_index_ < qr_ranked_.size()) {
      requestNextQrPlan();
    } else {
      retryQrSelectionWithoutHold("fixed_follow_arbiter_refused:" + seed_id);
    }
    return;
  }
  active_qr_path_ = path;
  active_qr_goal_id_ = region;
  ++active_qr_path_revision_;
  publishAudit(
    "qr_goal_locked",
    "seed=" + seed_id + ",mode=fixed_goal_one_midcourse_replan,poses=" +
    std::to_string(path.poses.size()));
}

void CompetitionSupervisor::maybeStartQrMidcourseReplan()
{
  // At the one geometric trigger, use the global costmap as the only obstacle
  // authority. Select one footprint-legal goal and issue exactly one Smac
  // request. No cone topic, freshness counter, or costmap-generation gate is
  // consulted here. If neither fixed goal is legal, the old FollowPath keeps
  // running unchanged.
  if (qr_midcourse_triggered_) {return;}
  const Pose2D pose = currentPose();
  if (!std::isfinite(pose.x) || pose.x < qr_midcourse_replan_x_) {return;}
  qr_midcourse_triggered_ = true;

  const auto find_seed = [this](const std::string & id) {
      return std::find_if(
        qr_ranked_.begin(), qr_ranked_.end(),
        [&id](const QrObservationCandidate & candidate) {return candidate.id == id;});
    };
  const auto primary = find_seed("primary");
  const auto backup = find_seed("backup");
  std::string primary_reason = "candidate unavailable";
  std::string backup_reason = "candidate unavailable";
  nav2_msgs::msg::Costmap::ConstSharedPtr grid;
  bool grid_valid = false;
  std::chrono::steady_clock::time_point grid_receipt;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    grid = latest_costmap_;
    grid_valid = latest_costmap_payload_valid_;
    grid_receipt = latest_costmap_steady_receipt_;
  }
  const bool grid_fresh = grid && grid_valid &&
    steadyAgeSec(grid_receipt) <= snapshot_global_hard_age_sec_;
  if (!grid_fresh) {
    primary_reason = "latest global costmap unavailable or stale";
    backup_reason = primary_reason;
  }
  const bool primary_legal = grid_fresh && primary != qr_ranked_.end() &&
    qrGoalFootprintLegal(*primary, *grid, primary_reason);
  const bool backup_legal = grid_fresh && backup != qr_ranked_.end() &&
    qrGoalFootprintLegal(*backup, *grid, backup_reason);

  std::ostringstream selection;
  selection << std::fixed << std::setprecision(3)
            << "x=" << pose.x << ",threshold=" << qr_midcourse_replan_x_
            << ",primary=" << (primary_legal ? "legal" : primary_reason)
            << ",backup=" << (backup_legal ? "legal" : backup_reason);
  if (primary_legal) {
    publishAudit("qr_midcourse_target_selected", selection.str() + ",selected=primary");
    requestQrMidcourseReplan(*primary);
  } else if (backup_legal) {
    publishAudit("qr_midcourse_target_selected", selection.str() + ",selected=backup");
    requestQrMidcourseReplan(*backup);
  } else {
    publishAudit(
      "qr_midcourse_no_legal_target",
      selection.str() + ",old_path_continues=true");
  }
}

void CompetitionSupervisor::requestQrMidcourseReplan(
  const QrObservationCandidate & seed)
{
  if (state_ != State::QR_FOLLOW || qr_latched_.load() || qr_midcourse_plan_sent_) {return;}
  const std::string tag = "qr_midcourse_" + seed.id;
  const bool sent = arbiter_->computePathWhileFollowing(
    poseStamped(seed.pose), "GridBased", tag,
    [this, seed](std::uint64_t, bool ok,
      const nav_msgs::msg::Path & path, const std::string & detail) {
      if (state_ != State::QR_FOLLOW || qr_latched_.load()) {return;}
      std::string reason;
      if (!ok || !validateNav2PathLight(path, seed.pose, reason)) {
        const std::string failure = ok ? "light_validation:" + reason : detail;
        publishAudit(
          "qr_midcourse_plan_failed",
          "seed=" + seed.id + ",old_path_continues=true,reason=" + failure);
        return;
      }
      replaceQrPathMidcourse(path, seed);
    });
  if (!sent) {
    // A concurrent handoff/replacement may briefly own the arbiter.  Do not
    // count that as the one permitted planning attempt; retry on the next
    // 20 Hz tick while the original QR path continues.
    publishAudit(
      "qr_midcourse_plan_deferred",
      "seed=" + seed.id + ",old_path_continues=true");
    return;
  }
  qr_midcourse_plan_sent_ = true;
  publishAudit(
    "qr_midcourse_plan_requested",
    "seed=" + seed.id + ",goal=" + std::to_string(seed.pose.x) + "," +
    std::to_string(seed.pose.y));
}

void CompetitionSupervisor::replaceQrPathMidcourse(
  const nav_msgs::msg::Path & path, const QrObservationCandidate & seed)
{
  const Pose2D motion_start = currentPose();
  const bool sent = arbiter_->replaceFollowPath(
    path, "FollowPath", "QrRegionGoalChecker", "qr:" + seed.id + ":midcourse",
    [this, motion_start, seed](
      std::uint64_t, rclcpp_action::ResultCode code, const std::string & tag) {
      if (qr_latched_.load() || state_ != State::QR_FOLLOW) {return;}
      const nav_msgs::msg::Path continuation = active_qr_path_;
      const std::string continuation_region = active_qr_goal_id_;
      active_qr_path_.poses.clear();
      active_qr_goal_id_.clear();
      ++active_qr_path_revision_;
      publishAudit("qr_motion_terminal", tag);
      const Pose2D terminal = currentPose();
      const double moved = distance(
        motion_start.x, motion_start.y, terminal.x, terminal.y);
      if (code == rclcpp_action::ResultCode::SUCCEEDED) {
        enter(State::QR_WAIT_CODE, "qr_midcourse_goal_reached_waiting_for_code");
        publishAudit(
          "qr_waiting_for_code",
          "midcourse_goal_reached=true,seed=" + seed.id +
          ",moved_m=" + std::to_string(moved));
      } else {
        if (race_continue_enabled_ && continuation.poses.size() >= 2U) {
          publishAudit(
            "qr_follow_abort_path_resubmitted",
            "seed=" + seed.id + ",mission_hold=false,new_smac=false,poses=" +
            std::to_string(continuation.poses.size()) + ",detail=" + tag);
          startQrPath(continuation, seed.id, continuation_region, false);
          return;
        }
        recoverQrFollowFailure(
          seed.id, "midcourse_follow_failed:moved_m=" + std::to_string(moved) +
          ":detail=" + tag);
      }
    });
  if (!sent) {
    publishAudit(
      "qr_midcourse_replace_refused",
      "seed=" + seed.id + ",old_path_continues=true");
    return;
  }
  active_qr_path_ = path;
  active_qr_goal_id_ = seed.region;
  ++active_qr_path_revision_;
  publishAudit(
    "qr_midcourse_path_replaced",
    "seed=" + seed.id + ",poses=" + std::to_string(path.poses.size()));
}

void CompetitionSupervisor::recoverQrFollowFailure(
  const std::string & failed_seed_id, const std::string & detail)
{
  publishAudit(
    "qr_follow_recovery_started",
    "failed_seed=" + failed_seed_id + ",mission_hold=false,detail=" + detail);
  if (failed_seed_id != "primary") {
    // Backup is only a one-shot escape from a failed Primary. If it also
    // aborts, return to the preferred Primary from the live pose instead of
    // parking in QR_WAIT_CODE. The fresh Smac request gives the 5 Hz global
    // costmap another chance to incorporate the cone that invalidated the
    // previous controller rollout.
    const auto primary = std::find_if(
      qr_ranked_.begin(), qr_ranked_.end(),
      [](const QrObservationCandidate & candidate) {return candidate.id == "primary";});
    std::string legality_reason = "candidate unavailable";
    if (primary == qr_ranked_.end() || !qrGoalFootprintLegal(*primary, legality_reason)) {
      retryQrSelectionWithoutHold(
        "backup_follow_failed_primary_unavailable:" + legality_reason);
      return;
    }

    const QrObservationCandidate recovery = *primary;
    enter(State::QR_PLAN, "qr_abort_recovery_primary_plan");
    const bool sent = arbiter_->computePath(
      poseStamped(recovery.pose), "GridBased", "qr_abort_recovery_primary",
      [this, recovery](std::uint64_t, bool ok, const nav_msgs::msg::Path & path,
        const std::string & plan_detail) {
        if (qr_latched_.load() || state_ != State::QR_PLAN) {return;}
        std::string reason;
        if (!ok || !validateNav2PathLight(path, recovery.pose, reason)) {
          retryQrSelectionWithoutHold(
            "backup_follow_failed_primary_plan:" +
            (ok ? "light_validation:" + reason : plan_detail));
          return;
        }
        publishAudit(
          "qr_follow_recovery_path_ready",
          "seed=primary,after=backup_abort,poses=" + std::to_string(path.poses.size()));
        startQrPath(path, recovery.id, recovery.region, false);
      });
    if (!sent) {
      retryQrSelectionWithoutHold("backup_follow_failed_primary_arbiter_refused");
    }
    return;
  }

  const auto backup = std::find_if(
    qr_ranked_.begin(), qr_ranked_.end(),
    [](const QrObservationCandidate & candidate) {return candidate.id == "backup";});
  std::string legality_reason = "candidate unavailable";
  if (backup == qr_ranked_.end() || !qrGoalFootprintLegal(*backup, legality_reason)) {
    publishAudit(
      "qr_follow_recovery_unavailable",
      "backup=" + legality_reason + ",mission_hold=false,primary_retry=true");
    retryQrSelectionWithoutHold("primary_follow_failed_no_legal_backup:" + legality_reason);
    return;
  }

  const QrObservationCandidate recovery = *backup;
  enter(State::QR_PLAN, "qr_abort_recovery_backup_plan");
  const bool sent = arbiter_->computePath(
    poseStamped(recovery.pose), "GridBased", "qr_abort_recovery_backup",
    [this, recovery](std::uint64_t, bool ok, const nav_msgs::msg::Path & path,
      const std::string & plan_detail) {
      if (qr_latched_.load() || state_ != State::QR_PLAN) {return;}
      std::string reason;
      if (!ok || !validateNav2PathLight(path, recovery.pose, reason)) {
        publishAudit(
          "qr_follow_recovery_failed",
          "seed=backup,mission_hold=false,reason=" +
          (ok ? "light_validation:" + reason : plan_detail));
        retryQrSelectionWithoutHold(
          "qr_abort_recovery_backup_plan_failed:" +
          (ok ? "light_validation:" + reason : plan_detail));
        return;
      }
      publishAudit(
        "qr_follow_recovery_path_ready",
        "seed=backup,poses=" + std::to_string(path.poses.size()));
      startQrPath(path, recovery.id, recovery.region, false);
    });
  if (!sent) {
    publishAudit(
      "qr_follow_recovery_failed",
      "seed=backup,mission_hold=false,reason=arbiter_refused");
    retryQrSelectionWithoutHold("qr_abort_recovery_backup_arbiter_refused");
  }
}

bool CompetitionSupervisor::loadDirectGateRobustMask(
  const std::string & path, std::string & reason)
{
  direct_gate_robust_mask_.clear();
  direct_gate_robust_mask_sha256_.clear();
  direct_gate_robust_mask_file_size_ = 0U;
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    reason = "cannot open deterministic Direct-Gate robust mask";
    return false;
  }
  std::vector<std::uint8_t> bytes{
    std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  direct_gate_robust_mask_file_size_ = bytes.size();
  direct_gate_robust_mask_sha256_ = sha256Hex(bytes);
  const std::size_t expected_size =
    (kDirectGateSizeX * kDirectGateSizeY * kDirectGateYawBins + 7U) / 8U;
  if (bytes.size() != expected_size) {
    reason = "Direct-Gate robust mask size mismatch:expected=" +
      std::to_string(expected_size) + ",actual=" + std::to_string(bytes.size());
    return false;
  }
  if (direct_gate_robust_mask_sha256_ != kDirectGateRobustMaskSha256) {
    reason = "Direct-Gate robust mask SHA256 mismatch";
    return false;
  }
  direct_gate_robust_mask_ = std::move(bytes);
  reason.clear();
  return true;
}

bool CompetitionSupervisor::loadReverseGateEntryMask(
  const std::string & path, std::string & reason)
{
  reverse_gate_entry_mask_.clear();
  reverse_gate_entry_mask_sha256_.clear();
  reverse_gate_entry_mask_file_size_ = 0U;
  reverse_gate_entry_size_x_ = 0U;
  reverse_gate_entry_size_y_ = 0U;
  reverse_gate_entry_yaw_bins_ = 0U;
  reverse_gate_entry_x_min_m_ = 0.0;
  reverse_gate_entry_y_min_m_ = 0.0;
  reverse_gate_entry_xy_resolution_m_ = 0.0;
  reverse_gate_entry_yaw_min_deg_ = 0;
  reverse_gate_entry_yaw_step_deg_ = 0;
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    reason = "cannot open reverse-gate entry LUT";
    return false;
  }
  std::vector<std::uint8_t> bytes{
    std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  reverse_gate_entry_mask_file_size_ = bytes.size();
  reverse_gate_entry_mask_sha256_ = sha256Hex(bytes);
  constexpr std::size_t header_size = 52U;
  if (bytes.size() < header_size ||
    std::memcmp(bytes.data(), kReverseGateEntryMagic, 4U) != 0)
  {
    reason = "reverse-gate entry LUT magic/schema mismatch";
    return false;
  }
  const auto read_u32 = [&bytes](std::size_t offset) {
      std::uint32_t value = 0U;
      std::memcpy(&value, bytes.data() + offset, sizeof(value));
      return value;
    };
  const auto read_i32 = [&bytes](std::size_t offset) {
      std::int32_t value = 0;
      std::memcpy(&value, bytes.data() + offset, sizeof(value));
      return value;
    };
  const auto read_f64 = [&bytes](std::size_t offset) {
      double value = 0.0;
      std::memcpy(&value, bytes.data() + offset, sizeof(value));
      return value;
    };
  const std::uint32_t size_x = read_u32(4U);
  const std::uint32_t size_y = read_u32(8U);
  const std::uint32_t yaw_bins = read_u32(12U);
  const std::uint32_t mask_len = read_u32(16U);
  const double x_min = read_f64(20U);
  const double y_min = read_f64(28U);
  const double resolution = read_f64(36U);
  const std::int32_t yaw_min = read_i32(44U);
  // The RGE2 header stores yaw_step immediately after yaw_min.  Keep the
  // explicit bounds check here so a truncated file cannot be accepted.
  if (bytes.size() < 52U) {
    reason = "reverse-gate entry LUT truncated header";
    return false;
  }
  const std::int32_t yaw_step = read_i32(48U);
  const std::size_t expected_states = static_cast<std::size_t>(size_x) *
    static_cast<std::size_t>(size_y) * static_cast<std::size_t>(yaw_bins);
  const std::size_t expected_mask_len = (expected_states + 7U) / 8U;
  if (size_x == 0U || size_y == 0U || yaw_bins == 0U ||
    mask_len != expected_mask_len || bytes.size() != 52U + mask_len ||
    !std::isfinite(x_min) || !std::isfinite(y_min) ||
    !std::isfinite(resolution) || resolution <= 0.0 ||
    yaw_step <= 0 || yaw_step > 180)
  {
    reason = "reverse-gate entry LUT dimensions/header invalid";
    return false;
  }
  // This asset is the vehicle-generated phase-190 lattice.  Keep the
  // deployment contract explicit: accepting a different lattice would make
  // the runtime query a different set of states than the reverse search.
  // The RGE2 header remains self-describing so diagnostics can report the
  // exact grid that was installed.
  if (size_x != 11U || size_y != 18U || yaw_bins != 19U ||
    std::abs(x_min - 2.0) > 1.0e-9 || std::abs(y_min - 0.3) > 1.0e-9 ||
    std::abs(resolution - 0.10) > 1.0e-9 || yaw_min != 0 || yaw_step != 5)
  {
    reason = "reverse-gate entry LUT lattice does not match deployed search grid";
    return false;
  }
  reverse_gate_entry_size_x_ = size_x;
  reverse_gate_entry_size_y_ = size_y;
  reverse_gate_entry_yaw_bins_ = yaw_bins;
  reverse_gate_entry_x_min_m_ = x_min;
  reverse_gate_entry_y_min_m_ = y_min;
  reverse_gate_entry_xy_resolution_m_ = resolution;
  reverse_gate_entry_yaw_min_deg_ = yaw_min;
  reverse_gate_entry_yaw_step_deg_ = yaw_step;
  reverse_gate_entry_mask_.assign(bytes.begin() + 52, bytes.end());
  reason.clear();
  return true;
}

bool CompetitionSupervisor::loadReverseGateGoalIds(
  const std::string & path, std::string & reason)
{
  reverse_gate_goal_ids_.clear();
  reverse_gate_handoff_goals_.clear();
  reverse_gate_goal_ids_sha256_.clear();
  reverse_gate_goal_ids_file_size_ = 0U;
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    reason = "cannot open reverse-gate goal-id LUT";
    return false;
  }
  std::vector<std::uint8_t> bytes{
    std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  reverse_gate_goal_ids_file_size_ = bytes.size();
  reverse_gate_goal_ids_sha256_ = sha256Hex(bytes);
  constexpr std::size_t header_size = 13U;
  if (bytes.size() < header_size ||
    std::memcmp(bytes.data(), kReverseGateGoalIdMagic, 5U) != 0)
  {
    reason = "reverse-gate goal-id LUT magic/schema mismatch";
    return false;
  }
  const auto read_u32 = [&bytes](std::size_t offset) {
      std::uint32_t value = 0U;
      std::memcpy(&value, bytes.data() + offset, sizeof(value));
      return value;
    };
  const std::uint32_t state_count = read_u32(5U);
  const std::uint32_t goal_count = read_u32(9U);
  const std::size_t goal_bytes = static_cast<std::size_t>(goal_count) * 24U;
  const std::size_t expected_states = reverse_gate_entry_size_x_ *
    reverse_gate_entry_size_y_ * reverse_gate_entry_yaw_bins_;
  if (state_count == 0U || (expected_states != 0U && state_count != expected_states) ||
    goal_count == 0U || goal_count > 255U ||
    bytes.size() != header_size + goal_bytes + state_count)
  {
    reason = "reverse-gate goal-id LUT dimensions invalid";
    return false;
  }
  const auto read_f64 = [&bytes](std::size_t offset) {
      double value = 0.0;
      std::memcpy(&value, bytes.data() + offset, sizeof(value));
      return value;
    };
  reverse_gate_handoff_goals_.reserve(goal_count);
  std::size_t offset = header_size;
  for (std::size_t i = 0U; i < goal_count; ++i) {
    const Pose2D goal{read_f64(offset), read_f64(offset + 8U),
      read_f64(offset + 16U) * kPi / 180.0};
    if (!std::isfinite(goal.x) || !std::isfinite(goal.y) ||
      !std::isfinite(goal.yaw))
    {
      reason = "reverse-gate goal-id LUT contains non-finite handoff goal";
      reverse_gate_handoff_goals_.clear();
      return false;
    }
    reverse_gate_handoff_goals_.push_back(goal);
    offset += 24U;
  }
  reverse_gate_goal_ids_.assign(bytes.begin() + static_cast<std::ptrdiff_t>(offset), bytes.end());
  if (reverse_gate_goal_ids_.size() != state_count) {
    reason = "reverse-gate goal-id state count mismatch";
    reverse_gate_goal_ids_.clear();
    reverse_gate_handoff_goals_.clear();
    return false;
  }
  reason.clear();
  return true;
}

bool CompetitionSupervisor::reverseGateEntryMaskFeasible(
  const Pose2D & pose, std::size_t * linear_index) const
{
  if (reverse_gate_entry_mask_.empty() ||
    reverse_gate_goal_ids_.empty() || !std::isfinite(pose.x) ||
    !std::isfinite(pose.y) || !std::isfinite(pose.yaw) ||
    reverse_gate_entry_size_x_ == 0U || reverse_gate_entry_size_y_ == 0U ||
    reverse_gate_entry_yaw_bins_ == 0U || reverse_gate_entry_xy_resolution_m_ <= 0.0)
  {
    return false;
  }
  const long ix = std::llround(
    (pose.x - reverse_gate_entry_x_min_m_) / reverse_gate_entry_xy_resolution_m_);
  const long iy = std::llround(
    (pose.y - reverse_gate_entry_y_min_m_) / reverse_gate_entry_xy_resolution_m_);
  double yaw_deg = normalize(pose.yaw) * 180.0 / kPi;
  if (yaw_deg < 0.0) {yaw_deg += 360.0;}
  const long iyaw = std::llround(
    (yaw_deg - static_cast<double>(reverse_gate_entry_yaw_min_deg_)) /
    static_cast<double>(reverse_gate_entry_yaw_step_deg_));
  if (ix < 0 || iy < 0 || iyaw < 0 ||
    ix >= static_cast<long>(reverse_gate_entry_size_x_) ||
    iy >= static_cast<long>(reverse_gate_entry_size_y_) ||
    iyaw >= static_cast<long>(reverse_gate_entry_yaw_bins_))
  {
    return false;
  }
  const std::size_t linear =
    (static_cast<std::size_t>(iyaw) * reverse_gate_entry_size_y_ +
    static_cast<std::size_t>(iy)) * reverse_gate_entry_size_x_ +
    static_cast<std::size_t>(ix);
  if ((linear >> 3U) >= reverse_gate_entry_mask_.size() ||
    linear >= reverse_gate_goal_ids_.size())
  {
    return false;
  }
  if (linear_index) {*linear_index = linear;}
  return (reverse_gate_entry_mask_[linear >> 3U] & (1U << (linear & 7U))) != 0U &&
    reverse_gate_goal_ids_[linear] != 255U &&
    reverse_gate_goal_ids_[linear] < reverse_gate_handoff_goals_.size();
}

std::optional<CompetitionSupervisor::Pose2D>
CompetitionSupervisor::reverseGateEntryGoal(const Pose2D & pose) const
{
  const auto match = reverseGateEntryMatch(pose);
  return match ? std::optional<Pose2D>{match->gate_goal} : std::nullopt;
}

std::optional<CompetitionSupervisor::ReverseGateMatch>
CompetitionSupervisor::reverseGateEntryMatch(const Pose2D & pose) const
{
  std::size_t linear = 0U;
  if (!reverseGateEntryMaskFeasible(pose, &linear) ||
    linear >= reverse_gate_goal_ids_.size())
  {
    return std::nullopt;
  }
  const auto goal_id = reverse_gate_goal_ids_[linear];
  if (goal_id == 255U || goal_id >= reverse_gate_handoff_goals_.size()) {
    return std::nullopt;
  }
  const std::size_t xy_plane =
    reverse_gate_entry_size_x_ * reverse_gate_entry_size_y_;
  const std::size_t iyaw = linear / xy_plane;
  const std::size_t remainder = linear % xy_plane;
  const std::size_t iy = remainder / reverse_gate_entry_size_x_;
  const std::size_t ix = remainder % reverse_gate_entry_size_x_;
  const double cell_x = reverse_gate_entry_x_min_m_ +
    static_cast<double>(ix) * reverse_gate_entry_xy_resolution_m_;
  const double cell_y = reverse_gate_entry_y_min_m_ +
    static_cast<double>(iy) * reverse_gate_entry_xy_resolution_m_;
  const double cell_yaw = normalize(
    (static_cast<double>(reverse_gate_entry_yaw_min_deg_) +
    static_cast<double>(iyaw) * reverse_gate_entry_yaw_step_deg_) * kPi / 180.0);
  ReverseGateMatch match;
  match.linear_index = linear;
  match.goal_id = goal_id;
  match.gate_goal = reverse_gate_handoff_goals_[goal_id];
  match.xy_error_m = std::hypot(pose.x - cell_x, pose.y - cell_y);
  match.yaw_error_rad = std::abs(normalize(pose.yaw - cell_yaw));
  return match;
}

bool CompetitionSupervisor::loadQrLaneBiasAsset(
  const std::string & path, std::string & reason)
{
  qr_lane_bias_active_data_.clear();
  qr_lane_bias_asset_sha256_.clear();
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    reason = "cannot open QR lane bias asset";
    return false;
  }
  std::vector<std::uint8_t> bytes{
    std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  if (bytes.size() != kQrLaneBiasBytes) {
    reason = "QR lane bias asset size mismatch:expected=" +
      std::to_string(kQrLaneBiasBytes) + ",actual=" + std::to_string(bytes.size());
    return false;
  }
  qr_lane_bias_asset_sha256_ = sha256Hex(bytes);
  if (qr_lane_bias_asset_sha256_ != kQrLaneBiasSha256) {
    reason = "QR lane bias asset SHA256 mismatch";
    return false;
  }
  for (const auto value : bytes) {
    if (value > 5U) {
      reason = "QR lane bias asset contains a value above occupancy 5";
      return false;
    }
  }
  qr_lane_bias_active_data_ = std::move(bytes);
  reason.clear();
  return true;
}

CompetitionSupervisor::DirectFeasibilityResult
CompetitionSupervisor::directForwardFeasible(const Pose2D & pose) const
{
  DirectFeasibilityResult result;
  if (direct_gate_robust_mask_.empty()) {
    result.reason = direct_gate_robust_mask_error_.empty() ?
      "robust LUT unavailable" : direct_gate_robust_mask_error_;
    return result;
  }
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw)) {
    result.reason = "non-finite QR latch pose";
    return result;
  }
  if (pose.x < kDirectGateXMinM - 1.0e-9 ||
    pose.x > kDirectGateXMaxM + 1.0e-9 ||
    pose.y < kDirectGateYMinM - 1.0e-9 ||
    pose.y > kDirectGateYMaxM + 1.0e-9 ||
    pose.y > kDirectGateQueryYMaxM + 1.0e-9)
  {
    result.reason = "QR latch pose outside robust LUT query domain";
    return result;
  }

  const long ix = std::llround((pose.x - kDirectGateXMinM) / kDirectGateResolutionM);
  const long iy = std::llround((pose.y - kDirectGateYMinM) / kDirectGateResolutionM);
  const double yaw_step = 2.0 * kPi / static_cast<double>(kDirectGateYawBins);
  long iyaw = std::llround((normalize(pose.yaw) + kPi) / yaw_step);
  iyaw %= static_cast<long>(kDirectGateYawBins);
  if (iyaw < 0) {iyaw += static_cast<long>(kDirectGateYawBins);}
  if (ix < 0 || iy < 0 ||
    ix >= static_cast<long>(kDirectGateSizeX) ||
    iy >= static_cast<long>(kDirectGateSizeY))
  {
    result.reason = "rounded QR latch pose outside robust LUT grid";
    return result;
  }

  result.ix = static_cast<std::size_t>(ix);
  result.iy = static_cast<std::size_t>(iy);
  result.iyaw = static_cast<std::size_t>(iyaw);
  const std::size_t linear =
    (result.iyaw * kDirectGateSizeY + result.iy) * kDirectGateSizeX + result.ix;
  if ((linear >> 3U) >= direct_gate_robust_mask_.size()) {
    result.reason = "robust LUT index exceeds validated mask";
    return result;
  }
  const bool feasible =
    (direct_gate_robust_mask_[linear >> 3U] & (1U << (linear & 7U))) != 0U;
  result.decision = feasible ? DirectGateDecision::DIRECT : DirectGateDecision::REVERSE;
  result.region_id = feasible ? "direct_gate_robust_neutral" : "robust_bit_clear";
  result.reason = feasible ?
    "fixed-map robust feasible bit set" : "fixed-map robust feasible bit clear";
  return result;
}

bool CompetitionSupervisor::gateEntryFeasible(const Pose2D & pose) const
{
  // Reverse-Gate RGE2 is the only terminal LUT for Reverse search.  The
  // Direct-Gate mask is intentionally never used here and remains an
  // offline diagnostic asset only.
  return reverseGateEntryMaskFeasible(pose);
}

void CompetitionSupervisor::failDirectGateToReverse(const std::string & reason)
{
  if (state_ == State::REVERSE_SELECT || state_ == State::REVERSE_FOLLOW)
  {
    return;
  }
  reverse_trigger_reason_ = reason;
  publishAudit("direct_gate_rejected", reason);
  publishAudit(
    "qr_exit_mode",
    "qr_exit_mode=reverse_required,reverse_trigger_reason=" + reason);
  arbiter_->cancelAllAsync("Direct-Gate fallback to Reverse: " + reason);
  stopped_samples_ = 0;
  stationary_started_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  enter(State::REVERSE_SELECT, "direct_gate_failed_prepare_reverse");
}

void CompetitionSupervisor::requestDirectGateQuery()
{
  if (direct_gate_query_attempted_) {return;}
  direct_gate_query_attempted_ = true;
  post_reverse_handoff_goal_.reset();
  // A legal QR result ends P->QR.  The Direct LUT is retained as an offline
  // diagnostic asset, but must not delay or veto the immediate Reverse
  // handoff.  Gate feasibility is decided once, after Reverse, from the
  // actual vehicle pose by the existing Smac Gate plan.
  publishAudit(
    "direct_gate_lut_bypassed",
    "force_reverse_after_qr=true; direct LUT retained as offline diagnostic only");
  failDirectGateToReverse("qr_force_reverse");
}

std::vector<CompetitionSupervisor::Pose2D> CompetitionSupervisor::gateCandidates() const
{
  if (post_reverse_handoff_goal_) {
    return {*post_reverse_handoff_goal_};
  }
  const double y = std::clamp(
    0.5 * (gate_region_.y_min + gate_region_.y_max),
    gate_region_.y_min, gate_region_.y_max);
  const double center_x = 0.5 * (gate_region_.x_min + gate_region_.x_max);
  // Gate is a tolerant entry region, not a single navigation pose. Keep three
  // bounded x/y representatives for Smac, but use one geometric reference yaw
  // only to seed the request. Gate acceptance is x/y based; terminal yaw is
  // deliberately not a success condition.
  const Pose2D current = currentPose();
  const double reference_yaw = std::isfinite(current.x) && std::isfinite(current.y) ?
    std::atan2(y - current.y, center_x - current.x) : current.yaw;
  return {
    Pose2D{std::clamp(center_x - 0.10, gate_region_.x_min, gate_region_.x_max), y,
      reference_yaw},
    Pose2D{center_x, y, reference_yaw},
    Pose2D{std::clamp(center_x + 0.10, gate_region_.x_min, gate_region_.x_max), y,
      reference_yaw}};
}

void CompetitionSupervisor::requestNextGatePlan(bool post_reverse)
{
  const State expected_state = post_reverse ? State::GATE_PLAN : State::FORWARD_GATE_PLAN;
  if (state_ != expected_state) {return;}
  const auto candidates = gateCandidates();
  if (gate_candidate_index_ >= candidates.size()) {
    const std::string reason = post_reverse ?
      "all post-Reverse CHANNEL_GATE_IN candidates failed planning or safety audit" :
      "all Direct CHANNEL_GATE_IN candidates failed planning or safety audit";
    if (post_reverse) {
      publishAudit(
        "post_reverse_planner_result",
        "post_reverse_planner_result=failed,reason=" + reason);
      publishAudit(
        "post_reverse_gate_skipped_to_channel_entry",
        "mission_hold=false,reason=" + reason);
      startPostGateFlow();
    } else {
      failDirectGateToReverse("all_gate_candidates_failed:" + reason);
    }
    return;
  }

  const std::size_t candidate_index = gate_candidate_index_++;
  const Pose2D candidate = candidates[candidate_index];
  const auto started = std::chrono::steady_clock::now();
  std::ostringstream request_detail;
  request_detail << "post_reverse=" << (post_reverse ? "true" : "false")
                 << ",gate_candidate_index=" << candidate_index
                 << ",goal=" << candidate.x << "," << candidate.y << "," << candidate.yaw;
  publishAudit("gate_planner_request", request_detail.str());
  const std::string tag = std::string(post_reverse ? "post_reverse_gate_" :
    "direct_gate_") + std::to_string(candidate_index);
  const bool plan_while_qr = !post_reverse && qr_latched_.load() &&
    arbiter_->motionActive();
  auto on_plan = [this, post_reverse, candidate, candidate_index, started](
        std::uint64_t, bool ok, const nav_msgs::msg::Path & path,
        const std::string & planner_detail)
      {
        const State callback_state =
          post_reverse ? State::GATE_PLAN : State::FORWARD_GATE_PLAN;
        if (state_ != callback_state) {return;}
        const double planning_time_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
        if (!ok) {
          std::ostringstream result_detail;
          result_detail << std::fixed << std::setprecision(3)
                        << "gate_candidate_index=" << candidate_index
                        << ",planner_result=failed"
                        << ",planning_time_ms=" << planning_time_ms
                        << ",path_pose_count=" << path.poses.size()
                        << ",path_audit_result=not_run"
                        << ",planner_detail=" << planner_detail;
          publishAudit(
            post_reverse ? "post_reverse_planner_result" : "forward_gate_planner_result",
            result_detail.str());
          requestNextGatePlan(post_reverse);
          return;
        }
        std::string path_reason;
        bool accepted = validateNav2PathLight(path, candidate, path_reason, kPi);
        if (accepted) {
          const auto & terminal = path.poses.back().pose;
          if (distance(terminal.position.x, terminal.position.y,
              candidate.x, candidate.y) > kGateGoalXyToleranceM)
          {
            path_reason = "Gate path terminal pose is outside the selected witness tolerance";
            accepted = false;
          }
        }
        std::ostringstream result_detail;
        result_detail << std::fixed << std::setprecision(3)
                      << "gate_candidate_index=" << candidate_index
                      << ",planner_result=success"
                      << ",planning_time_ms=" << planning_time_ms
                      << ",path_pose_count=" << path.poses.size()
                      << ",light_path_goal_check=" << (accepted ? "accepted" :
          "rejected:" + path_reason)
                      << ",planner_detail=" << planner_detail;
        publishAudit(
          post_reverse ? "post_reverse_planner_result" :
          "forward_gate_planner_result", result_detail.str());
        if (!accepted) {
          requestNextGatePlan(post_reverse);
          return;
        }
        startGatePath(path, post_reverse, candidate_index);
      };
  const bool plan_sent = plan_while_qr ?
    arbiter_->computePathWhileFollowing(
      poseStamped(candidate), "GridBased", tag, std::move(on_plan)) :
    arbiter_->computePath(
      poseStamped(candidate), "GridBased", tag, std::move(on_plan));
  if (!plan_sent)
  {
    const std::string reason = "MotionActionArbiter refused ComputePathToPose";
    publishAudit(
      post_reverse ? "post_reverse_planner_result" : "forward_gate_planner_result",
      "gate_candidate_index=" + std::to_string(candidate_index) +
      ",planner_result=arbiter_refused,path_pose_count=0,path_audit_result=not_run");
    if (post_reverse) {
      publishAudit(
        "post_reverse_gate_skipped_to_channel_entry",
        "mission_hold=false,reason=" + reason);
      startPostGateFlow();
    } else {
      failDirectGateToReverse("direct_planner_arbiter_refused:" + reason);
    }
  }
}

void CompetitionSupervisor::startGatePath(
  const nav_msgs::msg::Path & path, bool post_reverse,
  std::size_t candidate_index)
{
  enter(
    post_reverse ? State::GATE_FOLLOW : State::DIRECT_GATE_FOLLOW,
    std::string(post_reverse ? "post_reverse_gate_follow_" : "direct_gate_follow_") +
    std::to_string(candidate_index));
  const std::string tag = std::string(post_reverse ? "post_reverse_gate:" :
    "direct_gate:") + std::to_string(candidate_index);
  const std::string goal_checker = post_reverse ?
    "GateEntryGoalChecker" : "general_goal_checker";
  auto on_gate_result = [this, post_reverse, candidate_index](
        std::uint64_t, rclcpp_action::ResultCode code, const std::string & detail)
      {
        publishAudit(
          post_reverse ? "post_reverse_gate_action_terminal" :
          "direct_gate_action_terminal",
          "gate_candidate_index=" + std::to_string(candidate_index) + "," + detail);
        if (code == rclcpp_action::ResultCode::SUCCEEDED) {
          gate_handoff_cancel_pending_ = false;
          startPostGateFlow();
        } else if (post_reverse && gate_handoff_cancel_pending_) {
          publishAudit(
            "post_reverse_gate_expected_cancel",
            "gate_handoff_region_reached=true,terminal=" + detail);
        } else if (post_reverse) {
          publishAudit(
            "post_reverse_gate_follow_recover_to_channel_entry",
            "mission_hold=false,terminal=" + detail);
          startPostGateFlow();
        } else {
          failDirectGateToReverse("direct_gate_follow_failed:" + detail);
        }
      };
  bool sent = false;
  const bool replace_qr = !post_reverse && qr_latched_.load() &&
    arbiter_->motionActive();
  // Gate and the Gate-to-Center connector remain on the formal MPPI
  // controller. FollowPathChannel is reserved for paths already inside the
  // Tube, including lane transitions and Center progress.
  const std::string controller_id = "FollowPath";
  if (replace_qr) {
    sent = arbiter_->replaceFollowPath(
      path, "FollowPath", goal_checker, tag, std::move(on_gate_result));
  } else {
    sent = arbiter_->sendFollowPath(
      path, controller_id, goal_checker, tag, std::move(on_gate_result));
  }
  if (!sent)
  {
    if (post_reverse) {
      publishAudit(
        "post_reverse_gate_follow_recover_to_channel_entry",
        "mission_hold=false,terminal=arbiter_refused");
      startPostGateFlow();
    } else {
      failDirectGateToReverse(
        "direct_gate_follow_arbiter_refused:MotionActionArbiter refused FollowPath");
    }
  }
}

CompetitionSupervisor::ReverseSolveResult
CompetitionSupervisor::makeReverseCandidatesLayered(const Pose2D & frozen_qr_pose) const
{
  using SteadyClock = std::chrono::steady_clock;
  const auto total_started = SteadyClock::now();
  ReverseSolveResult result;
  // The worker is deliberately a pure function of the scan-time QR pose plus
  // one frozen obstacle snapshot.  Never read latest_map_pose_ here.
  const Pose2D start = frozen_qr_pose;
  nav_msgs::msg::Odometry::ConstSharedPtr odom;
  GateSearchSnapshot snapshot;
  std::chrono::steady_clock::time_point global_receipt;
  std::chrono::steady_clock::time_point local_receipt;
  bool have_map_pose = false;
  bool global_payload_valid = false;
  bool local_payload_valid = false;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    have_map_pose = std::isfinite(start.x) && std::isfinite(start.y) &&
      std::isfinite(start.yaw);
    odom = latest_odom_;
    snapshot.global_costmap = latest_costmap_;
    snapshot.local_costmap = latest_local_costmap_;
    snapshot.map_to_local = latest_map_to_local_;
    global_receipt = latest_costmap_steady_receipt_;
    local_receipt = latest_local_costmap_steady_receipt_;
    global_payload_valid = latest_costmap_payload_valid_;
    local_payload_valid = latest_local_costmap_payload_valid_;
  }
  result.steering_source = "zero_curvature_contract";
  result.initial_steering_levels = {0.0};
  const auto snapshot_now = now();
  const double global_age = steadyAgeSec(global_receipt);
  const double local_age = steadyAgeSec(local_receipt);
  if (!have_map_pose || !snapshot.global_costmap || !global_payload_valid ||
    snapshot.global_costmap->header.frame_id != map_frame_ ||
    (!race_continue_enabled_ && global_age > snapshot_global_hard_age_sec_) || !odom ||
    (snapshot_now - rclcpp::Time(odom->header.stamp)).seconds() > snapshot_odom_hard_age_sec_)
  {
    result.failure_reason =
      "SENSOR_STALE_HARD:pose/odom/global snapshot unavailable after refresh";
    result.total_solver_ms = std::chrono::duration<double, std::milli>(
      SteadyClock::now() - total_started).count();
    return result;
  }
  if (!snapshot.local_costmap || !local_payload_valid ||
    local_age > snapshot_local_warn_age_sec_ ||
    (snapshot.local_costmap->header.frame_id != odom_frame_ &&
    snapshot.local_costmap->header.frame_id != map_frame_))
  {
    // The global snapshot remains the authoritative hard obstacle field when
    // the rolling local snapshot is briefly unavailable at the handoff.
    snapshot.local_costmap = snapshot.global_costmap;
    snapshot.map_to_local = GateTransform2D{};
  }

  struct Family
  {
    const char * name;
    double radius_m;
    // No geometric return segment is appended to a Reverse primitive.  These
    // fields remain in the small local descriptor so the propagation code can
    // keep the same shape as the previously validated implementation.
    double ramp_down_start_m;
    double ramp_down_end_m;
  };
  // The fan is deliberately ordered from the lowest curvature to the most
  // aggressive backup.  Every family is advanced at each distance layer; the
  // order is only a deterministic tie-break, never an early return condition.
  const std::array<Family, 11> families{{
    {"straight", std::numeric_limits<double>::infinity(), 0.0, 0.0},
    {"radius_10m", 10.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()},
    {"radius_6m", 6.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()},
    {"radius_4m", 4.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()},
    {"radius_3m", 3.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()},
    {"radius_2m", 2.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()},
    {"radius_1p5m", 1.5, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()},
    {"radius_1p2m", 1.2, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()},
    {"radius_1m", 1.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()},
    {"radius_0p8m", 0.8, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()},
    {"radius_0p5m", 0.5, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()}}};
  const double wheelbase = gate_vehicle_geometry_.wheelbase_m;
  const double max_steering = std::atan(
    wheelbase * std::min(1.0 / min_turning_radius_,
    std::tan(gate_vehicle_geometry_.right_mechanical_limit_rad) / wheelbase));
  const double speed = std::max(0.05, reverse_design_speed_mps_);
  const double stop_reserve = speed * actuation_latency_sec_ +
    speed * speed / (2.0 * braking_deceleration_mps2_) + braking_distance_margin_m_;
  const auto move_toward = [](double value, double target, double limit) {
      return value < target ? std::min(value + limit, target) :
             std::max(value - limit, target);
    };
  const auto target_steering = [wheelbase, max_steering](const Family & family) {
      if (!std::isfinite(family.radius_m)) {return 0.0;}
      return std::min(max_steering, std::atan(wheelbase / family.radius_m));
    };
  const auto desired_steering = [&target_steering](const Family & family, double distance) {
      if (!std::isfinite(family.radius_m)) {return 0.0;}
      const double ramp_up = std::clamp(distance / 0.24, 0.0, 1.0);
      double ramp_down = 1.0;
      if (distance > family.ramp_down_start_m) {
        ramp_down = std::clamp(
          (family.ramp_down_end_m - distance) /
          (family.ramp_down_end_m - family.ramp_down_start_m), 0.0, 1.0);
      }
      return target_steering(family) * ramp_up * ramp_down;
    };
  const auto body_lateral = [&start](const Pose2D & pose) {
      const double dx = pose.x - start.x;
      const double dy = pose.y - start.y;
      return -std::sin(start.yaw) * dx + std::cos(start.yaw) * dy;
    };
  const auto audit_segment = [this, &snapshot](const Pose2D & previous,
      const Pose2D & current, std::string & reason) {
      return auditReverseSegmentHardSafety(previous, current, snapshot, reason);
    };

  const auto record_rejection = [&result](
      const std::string & category, const std::string & reason) {
      ++result.rejection_counts[category];
      if (result.first_rejection_reason.empty()) {
        result.first_rejection_reason = category +
          (reason.empty() ? std::string{} : ":" + reason);
      }
    };

  struct Branch
  {
    const Family * family{nullptr};
    Pose2D pose;
    double steering{0.0};
    double distance{0.0};
    bool alive{true};
    std::vector<Pose2D> command_poses;
  };
  std::vector<Branch> branches(families.size());
  for (std::size_t i = 0; i < families.size(); ++i) {
    branches[i].family = &families[i];
    branches[i].pose = start;
    branches[i].command_poses.push_back(start);
  }

  const auto build_candidate = [this, &start, &snapshot, &body_lateral,
      &move_toward, &record_rejection, &result, wheelbase, speed, stop_reserve](
      const Branch & branch,
      bool require_gate_entry) -> std::optional<ReverseCandidate> {
      if (!branch.alive || branch.command_poses.size() < 2U) {return std::nullopt;}
      ReverseCandidate candidate;
      candidate.family = branch.family->name;
      candidate.family_class = std::string(branch.family->name) == "straight" ?
        "straight" : "right_reverse";
      candidate.command_distance = branch.distance;
      candidate.command_path.header.frame_id = map_frame_;
      candidate.command_path.header.stamp = now();
      for (const auto & pose : branch.command_poses) {
        candidate.command_path.poses.push_back(poseStamped(pose));
      }
      candidate.safety_path = candidate.command_path;
      Pose2D stop_pose = branch.pose;
      double steering = branch.steering;
      double travelled = 0.0;
      while (travelled + 1.0e-9 < stop_reserve) {
        const double step = std::min(0.015, stop_reserve - travelled);
        const double remaining_speed = std::sqrt(std::max(
          0.0, speed * speed - 2.0 * braking_deceleration_mps2_ *
          std::max(0.0, travelled - speed * actuation_latency_sec_)));
        const double dt = step / std::max(0.03, remaining_speed);
        steering = move_toward(steering, 0.0, steering_return_rate_radps_ * dt);
        const double curvature = -std::tan(steering) / wheelbase;
        const double ds = -step;
        const double mid_yaw = stop_pose.yaw + 0.5 * ds * curvature;
        const Pose2D next{
          stop_pose.x + ds * std::cos(mid_yaw),
          stop_pose.y + ds * std::sin(mid_yaw),
          normalize(stop_pose.yaw + ds * curvature)};
        std::string reason;
        auto segment = candidate.safety_path;
        segment.poses.clear();
        segment.header = candidate.safety_path.header;
        segment.poses.push_back(poseStamped(stop_pose));
        segment.poses.push_back(poseStamped(next));
        if (body_lateral(next) > 0.02) {
          record_rejection("braking_body_lateral", "lateral_limit");
          return std::nullopt;
        }
        if (!auditReverseSegmentHardSafety(stop_pose, next, snapshot, reason)) {
          record_rejection("braking_segment_audit", reason);
          return std::nullopt;
        }
        candidate.safety_path.poses.push_back(poseStamped(next));
        stop_pose = next;
        travelled += step;
      }
      candidate.command_endpoint = branch.pose;
      candidate.worst_case_stop = stop_pose;
      candidate.predicted_stop = stop_pose;
      candidate.predicted_stop_steering = steering;
      candidate.command_endpoint_clearance = endpointClearanceInGrid(
        branch.pose, *snapshot.global_costmap);
      candidate.worst_case_stop_clearance = endpointClearanceInGrid(
        stop_pose, *snapshot.global_costmap);
      candidate.endpoint_clearance = candidate.command_endpoint_clearance;
      if (candidate.command_endpoint_clearance < 0.0 ||
        candidate.worst_case_stop_clearance < 0.0) {
        record_rejection("endpoint_clearance", "negative_global_clearance");
        return std::nullopt;
      }
      ++result.hard_safe_endpoints;
      if (require_gate_entry) {
        // The command path ends at branch.pose.  The braking extension is a
        // safety-only worst case and must never be used as the LUT query pose.
        const auto match = reverseGateEntryMatch(branch.pose);
        if (!match) {
          ++result.lut_range_misses;
          record_rejection(
            "reverse_gate_clean_lut", "nearest_cell_not_feasible");
          return std::nullopt;
        }
        candidate.lut_match = *match;
        candidate.goal_id = match->goal_id;
        candidate.gate_goal = match->gate_goal;
        ++result.lut_interior_hits;
      }
      candidate.score = candidate.command_distance -
        0.25 * candidate.command_endpoint_clearance;
      return candidate;
    };

  const auto search_started = SteadyClock::now();
  const std::size_t max_layers = static_cast<std::size_t>(std::ceil(
    reverse_max_search_distance_m_ / reverse_search_step_m_));
  const auto family_rank = [](const std::string & family) {
      static const std::array<std::string, 11> order{{
        "straight", "radius_10m", "radius_6m", "radius_4m", "radius_3m",
        "radius_2m", "radius_1p5m", "radius_1p2m", "radius_1m",
        "radius_0p8m", "radius_0p5m"}};
      const auto it = std::find(order.begin(), order.end(), family);
      return it == order.end() ? static_cast<int>(order.size()) :
        static_cast<int>(std::distance(order.begin(), it));
  };
  const auto low_curvature_better = [&family_rank](
      const ReverseCandidate & lhs, const ReverseCandidate & rhs) {
      return family_rank(lhs.family) < family_rank(rhs.family);
    };
  const auto yaw_error_to_target = [](const ReverseCandidate & candidate) {
      return std::abs(kPi / 2.0 - normalize(candidate.predicted_stop.yaw));
    };
  const auto yaw_target_better = [&yaw_error_to_target, &low_curvature_better](
      const ReverseCandidate & lhs, const ReverseCandidate & rhs) {
      const double lhs_error = yaw_error_to_target(lhs);
      const double rhs_error = yaw_error_to_target(rhs);
      if (std::abs(lhs_error - rhs_error) > 1.0e-9) {
        return lhs_error < rhs_error;
      }
      return low_curvature_better(lhs, rhs);
    };
  std::vector<ReverseCandidate> fallback_candidates;
  for (std::size_t layer = 1U; layer <= max_layers; ++layer) {
    const double distance = std::min(
      reverse_max_search_distance_m_, layer * reverse_search_step_m_);
    std::vector<ReverseCandidate> layer_candidates;
    for (auto & branch : branches) {
      if (!branch.alive) {continue;}
      const Pose2D previous = branch.pose;
      const double desired = desired_steering(*branch.family, distance);
      branch.steering = move_toward(
        branch.steering, desired, steering_build_rate_radps_ *
        reverse_search_step_m_ / speed);
      const double curvature = -std::tan(branch.steering) / wheelbase;
      const double ds = -(distance - branch.distance);
      const double mid_yaw = previous.yaw + 0.5 * ds * curvature;
      branch.pose = Pose2D{
        previous.x + ds * std::cos(mid_yaw),
        previous.y + ds * std::sin(mid_yaw),
        normalize(previous.yaw + ds * curvature)};
      branch.distance = distance;
      ++result.state_propagations;
      std::string reason;
      if (body_lateral(branch.pose) > 0.02) {
        record_rejection("branch_body_lateral", branch.family->name);
        branch.alive = false;
        continue;
      }
      if (!audit_segment(previous, branch.pose, reason)) {
        record_rejection("branch_segment_audit:" + std::string(branch.family->name), reason);
        branch.alive = false;
        continue;
      }
      branch.command_poses.push_back(branch.pose);
    }
    for (auto & branch : branches) {
      if (!branch.alive) {continue;}
      // The Direct LUT is bypassed after QR, but the Reverse-Gate LUT is the
      // terminal condition for layered Reverse search.  A candidate is
      // executable only when its predicted stop pose is a validated
      // forward-entry state; post-Reverse uses the actual pose and one Gate
      // The command endpoint is the LUT decision point.  The post-Reverse
      // handoff still uses one real-pose Gate Smac plan, without querying this
      // LUT again.
      auto candidate = build_candidate(branch, true);
      if (!candidate) {continue;}
      layer_candidates.push_back(std::move(*candidate));
      ++result.paths_built;
      ++result.strict_passed;
    }
    result.light_candidates += layer_candidates.size();
    std::vector<ReverseCandidate> target_band_candidates;
    for (const auto & candidate : layer_candidates) {
      const double stop_yaw = normalize(candidate.predicted_stop.yaw);
      if (stop_yaw >= kPi / 3.0 && stop_yaw <= kPi / 2.0) {
        target_band_candidates.push_back(candidate);
      }
      if (stop_yaw <= kPi / 2.0) {
        fallback_candidates.push_back(candidate);
      }
    }
    if (!target_band_candidates.empty()) {
      std::sort(
        target_band_candidates.begin(), target_band_candidates.end(), yaw_target_better);
      result.candidates = std::move(target_band_candidates);
      result.state_search_ms = std::chrono::duration<double, std::milli>(
        SteadyClock::now() - search_started).count();
      result.total_solver_ms = std::chrono::duration<double, std::milli>(
        SteadyClock::now() - total_started).count();
      return result;
    }
    if (std::none_of(branches.begin(), branches.end(),
        [](const Branch & branch) {return branch.alive;})) {
      break;
    }
  }

  result.state_search_ms = std::chrono::duration<double, std::milli>(
    SteadyClock::now() - search_started).count();
  if (!fallback_candidates.empty()) {
    std::sort(fallback_candidates.begin(), fallback_candidates.end(), yaw_target_better);
    result.candidates = std::move(fallback_candidates);
  }
  if (result.candidates.empty()) {
    result.failure_reason = result.hard_safe_endpoints > 0U ?
      "REVERSE_NO_CLEAN_LUT_MATCH:hard_safe_endpoints_without_gate_witness" :
      "REVERSE_NO_HARD_SAFE_BRANCH:hard_or_braking_check_rejected";
  }
  result.total_solver_ms = std::chrono::duration<double, std::milli>(
    SteadyClock::now() - total_started).count();
  return result;
}

std::optional<CompetitionSupervisor::ReverseCandidate>
CompetitionSupervisor::rebuildReverseCandidateFromPose(
  const ReverseCandidate & base, const Pose2D & start, const double execution_distance,
  const GateSearchSnapshot & snapshot, std::string & reason,
  GridFailureDiagnostic * const diagnostic) const
{
  // This is deliberately a small re-anchor, not a second LUT search.  The
  // worker chose the primitive and base distance from the frozen QR pose;
  // after braking we only regenerate that same primitive from the measured
  // stop pose and extend its distance by the measured forward slide.
  double radius = std::numeric_limits<double>::infinity();
  if (base.family == "radius_10m") {radius = 10.0;}
  else if (base.family == "radius_6m") {radius = 6.0;}
  else if (base.family == "radius_4m") {radius = 4.0;}
  else if (base.family == "radius_3m") {radius = 3.0;}
  else if (base.family == "radius_2m") {radius = 2.0;}
  else if (base.family == "radius_1p5m") {radius = 1.5;}
  else if (base.family == "radius_1p2m") {radius = 1.2;}
  else if (base.family == "radius_1m") {radius = 1.0;}
  else if (base.family == "radius_0p8m") {radius = 0.8;}
  else if (base.family == "radius_0p5m") {radius = 0.5;}
  else if (base.family != "straight") {
    reason = "unknown reverse primitive: " + base.family;
    return std::nullopt;
  }
  if (!std::isfinite(start.x) || !std::isfinite(start.y) ||
    !std::isfinite(start.yaw) || !std::isfinite(execution_distance) ||
    execution_distance <= 0.0)
  {
    reason = "invalid actual stop pose or execution distance";
    return std::nullopt;
  }
  const double distance_limit = std::max(reverse_search_step_m_, reverse_max_search_distance_m_);
  // A retry may legitimately be shorter than the layered-search quantum; the
  // initial QR candidate is still quantized by startSelectedReverse().
  const double total_distance = std::clamp(execution_distance, 1.0e-3, distance_limit);
  const double wheelbase = gate_vehicle_geometry_.wheelbase_m;
  const double max_steering = std::atan(
    wheelbase * std::min(1.0 / min_turning_radius_,
    std::tan(gate_vehicle_geometry_.right_mechanical_limit_rad) / wheelbase));
  const double speed = std::max(0.05, reverse_design_speed_mps_);
  const double step_limit = std::min(0.02, reverse_search_step_m_);
  const auto move_toward = [](double value, double target, double limit) {
      return value < target ? std::min(value + limit, target) :
             std::max(value - limit, target);
    };
  const auto target_steering = [&]() {
      if (!std::isfinite(radius)) {return 0.0;}
      return std::min(max_steering, std::atan(wheelbase / radius));
    }();

  ReverseCandidate rebuilt = base;
  rebuilt.command_distance = total_distance;
  rebuilt.command_path = nav_msgs::msg::Path{};
  rebuilt.command_path.header.frame_id = map_frame_;
  rebuilt.command_path.header.stamp = now();
  rebuilt.command_path.poses.push_back(poseStamped(start));
  Pose2D pose = start;
  double steering = 0.0;
  double travelled = 0.0;
  while (travelled + 1.0e-9 < total_distance) {
    const double ds_abs = std::min(step_limit, total_distance - travelled);
    steering = move_toward(
      steering, target_steering, steering_build_rate_radps_ * ds_abs / speed);
    const double curvature = -std::tan(steering) / wheelbase;
    const double ds = -ds_abs;
    const double mid_yaw = pose.yaw + 0.5 * ds * curvature;
    pose = Pose2D{
      pose.x + ds * std::cos(mid_yaw),
      pose.y + ds * std::sin(mid_yaw),
      normalize(pose.yaw + ds * curvature)};
    rebuilt.command_path.poses.push_back(poseStamped(pose));
    travelled += ds_abs;
  }
  rebuilt.command_endpoint = pose;
  rebuilt.safety_path = rebuilt.command_path;

  // Preserve the existing braking envelope, but audit the complete regenerated
  // path once after it is built.  No curvature/radius post-filter is used.
  const double stop_reserve = speed * actuation_latency_sec_ +
    speed * speed / (2.0 * braking_deceleration_mps2_) + braking_distance_margin_m_;
  double stop_travelled = 0.0;
  while (stop_travelled + 1.0e-9 < stop_reserve) {
    const double ds_abs = std::min(0.015, stop_reserve - stop_travelled);
    const double remaining_speed = std::sqrt(std::max(
      0.0, speed * speed - 2.0 * braking_deceleration_mps2_ *
      std::max(0.0, stop_travelled - speed * actuation_latency_sec_)));
    const double dt = ds_abs / std::max(0.03, remaining_speed);
    steering = move_toward(steering, 0.0, steering_return_rate_radps_ * dt);
    const double curvature = -std::tan(steering) / wheelbase;
    const double ds = -ds_abs;
    const double mid_yaw = pose.yaw + 0.5 * ds * curvature;
    pose = Pose2D{
      pose.x + ds * std::cos(mid_yaw),
      pose.y + ds * std::sin(mid_yaw),
      normalize(pose.yaw + ds * curvature)};
    rebuilt.safety_path.poses.push_back(poseStamped(pose));
    stop_travelled += ds_abs;
  }
  rebuilt.worst_case_stop = pose;
  rebuilt.predicted_stop = pose;
  rebuilt.predicted_stop_steering = steering;

  for (std::size_t i = 0; i + 1U < rebuilt.safety_path.poses.size(); ++i) {
    const auto & a = rebuilt.safety_path.poses[i].pose;
    const auto & b = rebuilt.safety_path.poses[i + 1U].pose;
    const Pose2D previous{a.position.x, a.position.y, yawOf(a.orientation)};
    const Pose2D current{b.position.x, b.position.y, yawOf(b.orientation)};
    if (!auditReverseSegmentHardSafety(previous, current, snapshot, reason, diagnostic)) {
      return std::nullopt;
    }
  }
  rebuilt.command_endpoint_clearance = endpointClearanceInGrid(
    rebuilt.command_endpoint, *snapshot.global_costmap);
  rebuilt.worst_case_stop_clearance = endpointClearanceInGrid(
    rebuilt.worst_case_stop, *snapshot.global_costmap);
  rebuilt.endpoint_clearance = rebuilt.command_endpoint_clearance;
  if (rebuilt.command_endpoint_clearance < 0.0 ||
    rebuilt.worst_case_stop_clearance < 0.0)
  {
    reason = "regenerated endpoint has no global clearance";
    return std::nullopt;
  }
  reason.clear();
  return rebuilt;
}

void CompetitionSupervisor::publishReverseSolveMetrics(const ReverseSolveResult & result)
{
  std::ostringstream detail;
  detail << std::fixed << std::setprecision(3)
         << "reverse_state_search_ms=" << result.state_search_ms
         << ",reverse_state_propagations=" << result.state_propagations
         << ",reverse_light_filter_ms=" << result.light_filter_ms
         << ",reverse_light_candidates=" << result.light_candidates
         << ",reverse_candidates_built=" << result.paths_built
         << ",reverse_full_paths_built=" << result.paths_built
         << ",reverse_strict_passed=" << result.strict_passed
         << ",reverse_hard_safe_endpoints=" << result.hard_safe_endpoints
         << ",reverse_lut_range_misses=" << result.lut_range_misses
         << ",reverse_lut_boundary_hits=" << result.lut_boundary_hits
         << ",reverse_lut_interior_hits=" << result.lut_interior_hits
         << ",reverse_path_build_ms=" << result.path_build_ms
         << ",reverse_strict_audit_ms=" << result.strict_audit_ms
         << ",reverse_total_solver_ms=" << result.total_solver_ms
         << ",reverse_search_step_m=" << reverse_search_step_m_
         << ",reverse_max_search_distance_m=" << reverse_max_search_distance_m_
         << ",reverse_family_count=11"
         << ",reverse_probe_budget=" <<
            static_cast<std::size_t>(std::ceil(
              reverse_max_search_distance_m_ / reverse_search_step_m_)) * 11U
         << ",reverse_primitives=straight|radius_10m|radius_6m|radius_4m|radius_3m|"
            "radius_2m|radius_1p5m|radius_1p2m|radius_1m|radius_0p8m|radius_0p5m"
         << ",gate_entry_terminal_yaw_required=false"
         << ",steering_source=" << result.steering_source
         << ",steering_estimate_deg=" << result.steering_estimate_rad * 180.0 / kPi
         << ",steering_uncertainty_deg=" << result.steering_uncertainty_rad * 180.0 / kPi
         << ",last_valid_command_age=" << result.last_valid_command_age_sec
         << ",initial_steering_levels=";
  for (std::size_t i = 0; i < result.initial_steering_levels.size(); ++i) {
    if (i != 0U) {detail << '|';}
    detail << result.initial_steering_levels[i] * 180.0 / kPi;
  }
  detail << ",search_states=" << result.state_propagations
         << ",reject_reason=" << result.failure_reason
         << ",first_rejection=" << result.first_rejection_reason
         << ",rejection_counts=";
  bool first_count = true;
  for (const auto & item : result.rejection_counts) {
    if (!first_count) {detail << '|';}
    first_count = false;
    detail << item.first << '=' << item.second;
  }
  publishAudit("reverse_solver_metrics", detail.str());
  std_msgs::msg::String metric_message;
  metric_message.data = detail.str();
  if (!competition_quiet_mode_) {
    reverse_diag_pub_->publish(metric_message);
    perf_trace_pub_->publish(metric_message);
  }
  const double lightweight_total_ms = result.state_search_ms + result.light_filter_ms;
  if (result.state_search_ms > 5.0 || lightweight_total_ms > 10.0 ||
    result.strict_audit_ms > 50.0)
  {
    RCLCPP_WARN(
      get_logger(),
      "reverse solver budget exceeded: state=%.3f ms light_total=%.3f ms strict=%.3f ms",
      result.state_search_ms, lightweight_total_ms, result.strict_audit_ms);
  }
}

void CompetitionSupervisor::startSelectedReverse()
{
  if (!selected_reverse_) {
    return;
  }
  if (reverse_actions_ >= 2) {
    publishAudit(
      "reverse_action_limit_continue_forward",
      "mission_hold=false,actions=" + std::to_string(reverse_actions_));
    requestPostReverseForwardPath();
    return;
  }
  // Candidate construction is allowed to overlap cancellation of the old QR
  // goal. Submission itself remains ownership-safe: do not send a reverse
  // action until the old action/plan is released. The speed smoother and
  // command limiter handle the sign transition after submission.
  if (arbiter_->motionActive() || arbiter_->planActive()) {
    return;
  }
  // The QR action must have actually stopped before the measured slide is
  // converted into extra reverse distance.  This is a live speed check only:
  // no stationary sample window, yaw stabilization, or map-pose epoch gate.
  nav_msgs::msg::Odometry::ConstSharedPtr odom;
  Pose2D actual_stop;
  nav2_msgs::msg::Costmap::ConstSharedPtr global_costmap;
  nav2_msgs::msg::Costmap::ConstSharedPtr local_costmap;
  GateTransform2D map_to_local;
  std::chrono::steady_clock::time_point global_receipt;
  std::chrono::steady_clock::time_point local_receipt;
  bool global_valid = false;
  bool local_valid = false;
  bool have_actual_pose = false;
  rclcpp::Time actual_pose_stamp(0, 0, get_clock()->get_clock_type());
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    odom = latest_odom_;
    actual_stop = latest_map_pose_;
    global_costmap = latest_costmap_;
    local_costmap = latest_local_costmap_;
    map_to_local = latest_map_to_local_;
    global_receipt = latest_costmap_steady_receipt_;
    local_receipt = latest_local_costmap_steady_receipt_;
    global_valid = latest_costmap_payload_valid_;
    local_valid = latest_local_costmap_payload_valid_;
    have_actual_pose = have_map_pose_;
    actual_pose_stamp = latest_map_pose_stamp_;
  }
  if (!odom || !have_actual_pose ||
    actual_pose_stamp.nanoseconds() == 0 ||
    (now() - actual_pose_stamp).seconds() > snapshot_tf_hard_age_sec_ ||
    (now() - rclcpp::Time(odom->header.stamp)).seconds() > snapshot_odom_hard_age_sec_ ||
    std::abs(odom->twist.twist.linear.x) > stationary_linear_threshold_mps_ ||
    std::abs(odom->twist.twist.angular.z) > stationary_angular_threshold_radps_)
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "reverse_waiting_for_qr_stop: owner_released=true; waiting for live velocity to reach zero");
    return;
  }
  if (!global_costmap || !global_valid ||
    (!race_continue_enabled_ &&
    steadyAgeSec(global_receipt) > snapshot_global_hard_age_sec_))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "reverse_waiting_for_valid_global_costmap: mission_hold=false");
    return;
  }
  if (!local_costmap || !local_valid ||
    steadyAgeSec(local_receipt) > snapshot_local_hard_age_sec_)
  {
    // The global map remains authoritative when the rolling local map is
    // briefly unavailable at handoff; the hard checker still protects the
    // static map and all known hard obstacles.
    local_costmap = global_costmap;
    map_to_local = GateTransform2D{};
  }
  const bool retry_suffix = reverse_retry_remaining_distance_m_.has_value();
  const double slide = retry_suffix ? 0.0 : std::max(
    0.0,
    (actual_stop.x - qr_frozen_pose_.x) * std::cos(qr_frozen_pose_.yaw) +
    (actual_stop.y - qr_frozen_pose_.y) * std::sin(qr_frozen_pose_.yaw));
  GateSearchSnapshot snapshot;
  snapshot.global_costmap = global_costmap;
  snapshot.local_costmap = local_costmap;
  snapshot.map_to_local = map_to_local;
  const auto costAtMapPoint = [this](
      const nav2_msgs::msg::Costmap & grid, const GateTransform2D & map_to_grid,
      const double map_x, const double map_y) {
      if (grid.metadata.resolution <= 0.0F || grid.metadata.size_x == 0U ||
        grid.metadata.size_y == 0U)
      {
        return -1;
      }
      double grid_x = map_x;
      double grid_y = map_y;
      if (grid.header.frame_id == odom_frame_) {
        const double c = std::cos(map_to_grid.yaw);
        const double s = std::sin(map_to_grid.yaw);
        grid_x = map_to_grid.x + c * map_x - s * map_y;
        grid_y = map_to_grid.y + s * map_x + c * map_y;
      } else if (grid.header.frame_id != map_frame_) {
        return -1;
      }
      const int mx = static_cast<int>(std::floor(
        (grid_x - grid.metadata.origin.position.x) / grid.metadata.resolution));
      const int my = static_cast<int>(std::floor(
        (grid_y - grid.metadata.origin.position.y) / grid.metadata.resolution));
      if (mx < 0 || my < 0 || mx >= static_cast<int>(grid.metadata.size_x) ||
        my >= static_cast<int>(grid.metadata.size_y))
      {
        return -1;
      }
      const auto index = static_cast<std::size_t>(my) * grid.metadata.size_x +
        static_cast<std::size_t>(mx);
      return index < grid.data.size() ? static_cast<int>(grid.data[index]) : -1;
    };
  const auto logFailure = [this, &costAtMapPoint, &global_costmap, &local_costmap,
      &map_to_local](
      const std::size_t candidate_index, const std::string & safety_reason,
      GridFailureDiagnostic & safety_diagnostic) {
      if (!safety_diagnostic.valid) {return;}
      safety_diagnostic.peer_global_cost = costAtMapPoint(
        *global_costmap, map_to_local,
        safety_diagnostic.world_x_m, safety_diagnostic.world_y_m);
      safety_diagnostic.peer_local_cost = costAtMapPoint(
        *local_costmap, map_to_local,
        safety_diagnostic.world_x_m, safety_diagnostic.world_y_m);
      double nearest_cone_distance = std::numeric_limits<double>::infinity();
      std::size_t cone_count = 0U;
      std::vector<std::pair<double, double>> cone_centers;
      {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        cone_centers = latest_cone_centers_map_;
      }
      cone_count = cone_centers.size();
      for (const auto & cone : cone_centers) {
        nearest_cone_distance = std::min(
          nearest_cone_distance,
          std::hypot(
            cone.first - safety_diagnostic.world_x_m,
            cone.second - safety_diagnostic.world_y_m));
      }
      std::ostringstream collision_detail;
      collision_detail << std::fixed << std::setprecision(4)
                       << "candidate_index=" << candidate_index
                       << ",source=" << safety_diagnostic.source
                       << ",frame=" << safety_diagnostic.frame
                       << ",segment=" << safety_diagnostic.segment_index << "/"
                       << safety_diagnostic.segment_count
                       << ",sample=" << safety_diagnostic.sample_index << "/"
                       << safety_diagnostic.sample_count
                       << ",base_pose_map=(" << safety_diagnostic.base_pose_map.x << ","
                       << safety_diagnostic.base_pose_map.y << ","
                       << safety_diagnostic.base_pose_map.yaw * 180.0 / kPi << "deg)"
                       << ",checked_pose_" << safety_diagnostic.frame << "=("
                       << safety_diagnostic.checked_pose.x << ","
                       << safety_diagnostic.checked_pose.y << ","
                       << safety_diagnostic.checked_pose.yaw * 180.0 / kPi << "deg)"
                       << ",footprint_sample_local=("
                       << safety_diagnostic.footprint_sample_x_m << ","
                       << safety_diagnostic.footprint_sample_y_m << ")"
                       << ",grid_frame_point=(" << safety_diagnostic.grid_frame_x_m << ","
                       << safety_diagnostic.grid_frame_y_m << ")"
                       << ",world=(" << safety_diagnostic.world_x_m << ","
                       << safety_diagnostic.world_y_m << ")"
                       << ",grid=(" << safety_diagnostic.grid_x << ","
                       << safety_diagnostic.grid_y << ")"
                       << ",cost=" << safety_diagnostic.cost
                       << ",reason=" << safety_diagnostic.reason
                       << ",peer_global_cost=" << safety_diagnostic.peer_global_cost
                       << ",peer_local_cost=" << safety_diagnostic.peer_local_cost
                       << ",nearest_cone_distance_m=";
      if (std::isfinite(nearest_cone_distance)) {
        collision_detail << nearest_cone_distance;
      } else {
        collision_detail << "inf";
      }
      collision_detail << ",cone_count=" << cone_count;
      const std::string event = candidate_index == reverse_candidate_index_ ?
        "reverse_final_first_failure" : "reverse_final_candidate_failure";
      publishAudit(event, collision_detail.str());
      RCLCPP_ERROR(get_logger(), "%s %s", event.c_str(), collision_detail.str().c_str());
      (void)safety_reason;
    };
  std::optional<ReverseCandidate> rebuilt;
  std::string last_safety_reason;
  double exec_distance = 0.0;
  double base_distance = 0.0;
  const std::size_t candidate_count = reverse_candidates_.empty() ? 1U : reverse_candidates_.size();
  const std::size_t first_candidate = std::min(reverse_candidate_index_, candidate_count - 1U);
  const std::size_t candidate_limit = retry_suffix ? first_candidate + 1U : candidate_count;
  for (std::size_t candidate_index = first_candidate;
    candidate_index < candidate_limit; ++candidate_index)
  {
    const ReverseCandidate & nominal = reverse_candidates_.empty() ?
      *nominal_reverse_ : reverse_candidates_[candidate_index];
    base_distance = nominal.command_distance;
    const double requested_distance = retry_suffix ?
      *reverse_retry_remaining_distance_m_ : base_distance + slide;
    exec_distance = std::min(
      reverse_max_search_distance_m_, std::max(1.0e-3, requested_distance));
    if (!retry_suffix) {
      exec_distance = std::max(reverse_search_step_m_, exec_distance);
    }
    std::string safety_reason;
    GridFailureDiagnostic safety_diagnostic;
    const auto attempt = rebuildReverseCandidateFromPose(
      nominal, actual_stop, exec_distance, snapshot, safety_reason, &safety_diagnostic);
    if (attempt) {
      rebuilt = *attempt;
      reverse_candidate_index_ = candidate_index;
      nominal_reverse_ = nominal;
      break;
    }
    last_safety_reason = safety_reason;
    logFailure(candidate_index, safety_reason, safety_diagnostic);
    publishAudit(
      "reverse_final_candidate_failed",
      "candidate_index=" + std::to_string(candidate_index) +
      ",nominal_distance_m=" + std::to_string(base_distance) +
      ",exec_distance_m=" + std::to_string(exec_distance) +
      ",reason=" + safety_reason);
  }
  if (!rebuilt) {
    publishAudit(
      "reverse_final_hard_check_failed",
      "candidate_count=" + std::to_string(candidate_limit - first_candidate) +
      ",reason=" + last_safety_reason + ",slide_m=" + std::to_string(slide));
    publishAudit(
      "reverse_final_hard_check_continue_forward",
      "mission_hold=false,smac_required=true,reason=" + last_safety_reason);
    requestPostReverseForwardPath();
    return;
  }
  selected_reverse_ = *rebuilt;
  reverse_execution_distance_m_ = exec_distance;
  reverse_retry_remaining_distance_m_.reset();
  publishAudit(
    "reverse_exec_reanchored",
    "qr_pose=" + std::to_string(qr_frozen_pose_.x) + "," +
    std::to_string(qr_frozen_pose_.y) + "," + std::to_string(qr_frozen_pose_.yaw) +
    ",actual_stop=" + std::to_string(actual_stop.x) + "," +
    std::to_string(actual_stop.y) + "," + std::to_string(actual_stop.yaw) +
    ",slide_m=" + std::to_string(slide) +
    ",base_distance_m=" + std::to_string(base_distance) +
    ",exec_distance_m=" + std::to_string(exec_distance) +
    ",retry_suffix=" + (retry_suffix ? "true" : "false") +
    ",reverse_candidate_index=" + std::to_string(reverse_candidate_index_) +
    ",map_pose_epoch_ignored=true");
  ++reverse_actions_;
  enter(State::REVERSE_FOLLOW, "reverse_joint_" + selected_reverse_->family);
  if (!arbiter_->sendFollowPath(
    selected_reverse_->command_path, "FollowPathReverse", "general_goal_checker", "reverse",
    [this](std::uint64_t, rclcpp_action::ResultCode code, const std::string & detail) {
      if (state_ != State::REVERSE_FOLLOW) {return;}
      publishAudit("reverse_action_terminal", detail);
      if (code == rclcpp_action::ResultCode::SUCCEEDED) {
        stopped_samples_ = 0;
        enter(State::WAIT_STOP_AFTER_REVERSE, "reverse_terminal_success");
      } else {
        reverse_execution_failed_ = true;
        stopped_samples_ = 0;
        enter(State::WAIT_STOP_AFTER_REVERSE,
          "reverse_terminal_failure_wait_actual_stationary_pose");
      }
    }))
  {
    publishAudit(
      "reverse_action_refused_continuing_forward",
      "mission_hold=false,reason=arbiter_refused");
    requestPostReverseForwardPath();
  }
}

void CompetitionSupervisor::requestGatePath()
{
  if (!selected_reverse_) {
    publishAudit(
      "post_reverse_context_missing_continue_to_entry",
      "mission_hold=false,reason=missing_reverse_completion_context");
    startPostGateFlow();
    return;
  }
  const Pose2D actual = currentPose();
  // Use the actual post-Reverse pose as the single Smac start and preserve
  // the clean LUT's RGEG2 witness without consulting any Tube asset.
  if (selected_reverse_->goal_id == 255U ||
    selected_reverse_->goal_id >= reverse_gate_handoff_goals_.size())
  {
    publishAudit(
      "post_reverse_context_missing_continue_to_entry",
      "mission_hold=false,reason=reverse_candidate_has_no_gate_witness");
    startPostGateFlow();
    return;
  }
  // The RGEG2 goal_id remains Reverse endpoint evidence only.  The actual
  // post-Reverse Gate request is deliberately fixed so every run enters the
  // same central handoff region before the direction-specific Entry Lock.
  post_reverse_handoff_goal_ = Pose2D{
    kPostReverseGateHandoffX, kPostReverseGateHandoffY,
    kPostReverseGateHandoffYaw};
  gate_handoff_cancel_pending_ = false;
  std::ostringstream detail;
  detail << std::fixed << std::setprecision(3)
         << "post_reverse_handoff_pose_x=" << actual.x
         << ",post_reverse_handoff_pose_y=" << actual.y
         << ",post_reverse_handoff_pose_yaw=" << actual.yaw
         << ",gate_goal_id=" << static_cast<unsigned int>(selected_reverse_->goal_id)
         << ",gate_goal_source=fixed_post_reverse_handoff"
         << ",gate_goal_x=" << post_reverse_handoff_goal_->x
         << ",gate_goal_y=" << post_reverse_handoff_goal_->y
         << ",gate_goal_yaw=" << post_reverse_handoff_goal_->yaw;
  publishAudit("post_reverse_gate_plan_start", detail.str());
  gate_candidate_index_ = 0U;
  enter(State::GATE_PLAN, "post_reverse_compute_path_to_gate");
  requestNextGatePlan(true);
}

void CompetitionSupervisor::requestPostReverseForwardPath()
{
  if (!direct_reverse_to_channel_entry_enabled_ || mission_scope_ != "full") {
    requestGatePath();
    return;
  }
  direct_entry_attempt_active_ = true;
  publishAudit(
    "direct_reverse_to_channel_entry_start",
    "source=actual_stationary_reverse_pose,entry=S1_then_S2,gate=fallback");
  requestChannelEntry(false);
}


const CompetitionSupervisor::ChannelTube * CompetitionSupervisor::channelTube(
  ChannelLane lane) const
{
  for (const auto & tube : channel_tube_library_) {
    if (tube.clockwise == clockwise_ && tube.lane == lane) {return &tube;}
  }
  return nullptr;
}

CompetitionSupervisor::ChannelProjection CompetitionSupervisor::projectToChannelTube(
  const ChannelTube & tube, const Pose2D & pose) const
{
  ChannelProjection result;
  for (std::size_t i = 0U; i < tube.path.poses.size(); ++i) {
    const auto & point = tube.path.poses[i].pose.position;
    const double d = distance(pose.x, pose.y, point.x, point.y);
    if (d < result.distance_m) {
      result.index = i;
      result.s_mod_m = tube.s_m[i];
      result.distance_m = d;
    }
  }
  return result;
}

CompetitionSupervisor::ChannelProjection CompetitionSupervisor::projectToOpenPath(
  const nav_msgs::msg::Path & path, const Pose2D & pose) const
{
  ChannelProjection projection;
  double cumulative_m = 0.0;
  for (std::size_t index = 0U; index + 1U < path.poses.size(); ++index) {
    const auto & a = path.poses[index].pose.position;
    const auto & b = path.poses[index + 1U].pose.position;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_sq = dx * dx + dy * dy;
    const double length_m = std::sqrt(length_sq);
    if (length_sq <= 1.0e-12) {continue;}
    const double t = std::clamp(
      ((pose.x - a.x) * dx + (pose.y - a.y) * dy) / length_sq, 0.0, 1.0);
    const double projected_x = a.x + t * dx;
    const double projected_y = a.y + t * dy;
    const double cross_track_m = distance(
      pose.x, pose.y, projected_x, projected_y);
    if (cross_track_m < projection.distance_m) {
      projection.index = t >= 0.5 ? index + 1U : index;
      projection.s_mod_m = cumulative_m + t * length_m;
      projection.distance_m = cross_track_m;
    }
    cumulative_m += length_m;
  }
  return projection;
}

double CompetitionSupervisor::centerForwardDistance(double from_s, double to_s) const
{
  const auto * center = channelTube(ChannelLane::CENTER);
  if (!center || center->loop_length_m <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  double delta = to_s - from_s;
  while (delta < 0.0) {delta += center->loop_length_m;}
  while (delta >= center->loop_length_m) {delta -= center->loop_length_m;}
  return delta;
}

nav_msgs::msg::Path CompetitionSupervisor::cyclicChannelSuffix(
  const ChannelTube & tube, std::size_t start_index,
  std::size_t stop_index, bool require_wrap) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = map_frame_;
  path.header.stamp = now();
  if (tube.path.poses.empty()) {return path;}
  start_index %= tube.path.poses.size();
  stop_index %= tube.path.poses.size();
  std::size_t index = start_index;
  bool wrapped = false;
  // A wrapped Side bypass may start after its split index and must reach that
  // index only after crossing index 0.  The old N+1 bound truncated such a
  // path before the split, so FollowPath legitimately SUCCEEDED at a point
  // with no return connector and the vehicle appeared to stop.  Permit one
  // complete loop plus the remaining suffix when wrap is explicitly needed.
  const std::size_t max_count = require_wrap ?
    2U * tube.path.poses.size() + 1U : tube.path.poses.size() + 1U;
  for (std::size_t count = 0U; count <= max_count; ++count) {
    auto stamped = tube.path.poses[index];
    stamped.header = path.header;
    path.poses.push_back(std::move(stamped));
    if (index == stop_index && count > 0U && (!require_wrap || wrapped)) {break;}
    const std::size_t next = (index + 1U) % tube.path.poses.size();
    if (next == 0U) {wrapped = true;}
    index = next;
  }
  return path;
}

nav_msgs::msg::Path CompetitionSupervisor::centerPathToExit(
  std::size_t start_index) const
{
  const auto * center = channelTube(ChannelLane::CENTER);
  if (!center) {return nav_msgs::msg::Path{};}
  const bool wraps = channel_exit_center_index_ <= start_index;
  auto path = cyclicChannelSuffix(
    *center, start_index, channel_exit_center_index_, wraps);
  const auto & branch = clockwise_ ? channel_exit_cw_path_ : channel_exit_ccw_path_;
  if (branch.poses.size() >= 2U) {
    for (std::size_t i = 1U; i < branch.poses.size(); ++i) {
      auto stamped = branch.poses[i];
      stamped.header = path.header;
      path.poses.push_back(std::move(stamped));
    }
  }
  // Keep the verified straight suffix after the physical y=2.30 Gate-Out
  // handoff in this fallback path.  The handoff detector still replaces this
  // action as soon as the vehicle is clear of the Channel, but RPP now sees
  // admitted straight runway instead of treating the handoff itself as
  // its goal.  This prevents approach-velocity scaling from braking almost to
  // zero before the already-prepared Return path is hot-replaced, and gives
  // the final Channel bend room to straighten.  If Return preparation is
  // unavailable, the same admitted suffix remains a bounded safe fallback.
  return path;
}

bool CompetitionSupervisor::channelStraightTransitionEligible(
  const ChannelTube & tube, std::size_t start_index, std::string & reason) const
{
  if (tube.path.poses.size() < 2U) {reason = "empty active Tube"; return false;}
  start_index %= tube.path.poses.size();
  const double start_yaw = yawOf(tube.path.poses[start_index].pose.orientation);
  if (std::abs(std::sin(start_yaw)) > std::sin(5.0 * kPi / 180.0)) {
    reason = "lane changes are limited to top/bottom horizontal straights";
    return false;
  }
  double remaining = 0.0;
  std::size_t index = start_index;
  for (std::size_t count = 0U; count < tube.path.poses.size(); ++count) {
    const std::size_t next = (index + 1U) % tube.path.poses.size();
    const double next_yaw = yawOf(tube.path.poses[next].pose.orientation);
    if (std::abs(normalize(next_yaw - start_yaw)) > 5.0 * kPi / 180.0) {break;}
    const auto & a = tube.path.poses[index].pose.position;
    const auto & b = tube.path.poses[next].pose.position;
    remaining += distance(a.x, a.y, b.x, b.y);
    if (remaining + 1.0e-9 >= kChannelMinimumStraightRemainingM) {
      reason.clear(); return true;
    }
    index = next;
  }
  std::ostringstream detail;
  detail << "remaining straight=" << remaining << "m,required="
         << kChannelMinimumStraightRemainingM << "m";
  reason = detail.str();
  return false;
}

bool CompetitionSupervisor::buildChannelTransition(
  ChannelLane target_lane, nav_msgs::msg::Path & path,
  ChannelActionKind & kind, std::string & reason)
{
  if (channel_avoidance_v2_enabled_) {
    return buildChannelTransitionV2(target_lane, path, kind, reason);
  }
  if (active_channel_lane_ == target_lane ||
    (active_channel_lane_ != ChannelLane::CENTER && target_lane != ChannelLane::CENTER))
  {
    reason = "only Center<->Side transitions are legal";
    return false;
  }
  const auto * active = channelTube(active_channel_lane_);
  const auto * target = channelTube(target_lane);
  if (!active || !target) {reason = "direction-matched Tube unavailable"; return false;}
  const Pose2D actual = currentPose();
  const auto active_projection = projectToChannelTube(*active, actual);
  if (!channelStraightTransitionEligible(*active, active_projection.index, reason)) {
    return false;
  }
  const auto endpoint = [&actual](double lateral) {
      return Pose2D{
        actual.x + kChannelTransitionLengthM * std::cos(actual.yaw) -
        lateral * std::sin(actual.yaw),
        actual.y + kChannelTransitionLengthM * std::sin(actual.yaw) +
        lateral * std::cos(actual.yaw), actual.yaw};
    };
  const bool bottom_straight = actual.y < 0.5 * (3.18 + 4.34);
  const double straight_offset = bottom_straight ?
    kChannelBottomStraightOffsetM : kChannelTopStraightOffsetM;
  const auto plus_projection = projectToChannelTube(*target, endpoint(straight_offset));
  const auto minus_projection = projectToChannelTube(*target, endpoint(-straight_offset));
  const bool use_plus = plus_projection.distance_m <= minus_projection.distance_m;
  const auto & transition = bottom_straight ?
    (use_plus ? channel_transition_bottom_plus_ : channel_transition_bottom_minus_) :
    (use_plus ? channel_transition_top_plus_ : channel_transition_top_minus_);
  const auto target_projection = use_plus ? plus_projection : minus_projection;
  if (transition.size() < 2U || target_projection.distance_m > 0.025) {
    reason = "0.52m transition endpoint does not meet destination Tube";
    return false;
  }
  path = nav_msgs::msg::Path{};
  path.header.frame_id = map_frame_;
  path.header.stamp = now();
  const double c = std::cos(actual.yaw), s = std::sin(actual.yaw);
  for (const auto & local : transition) {
    auto stamped = poseStamped(Pose2D{
      actual.x + c * local.x_local_m - s * local.y_local_m,
      actual.y + s * local.x_local_m + c * local.y_local_m,
      normalize(actual.yaw + local.yaw_local_rad)});
    stamped.header = path.header;
    path.poses.push_back(std::move(stamped));
  }
  nav_msgs::msg::Path suffix;
  if (target_lane == ChannelLane::CENTER) {
    suffix = centerPathToExit(target_projection.index);
    kind = active_channel_lane_ == ChannelLane::INNER ?
      ChannelActionKind::RETURN_FROM_INNER : ChannelActionKind::RETURN_FROM_OUTER;
  } else {
    const Pose2D split = clockwise_ ? Pose2D{2.961454, 3.18, kPi} :
      Pose2D{2.038546, 3.18, 0.0};
    const auto stop = projectToChannelTube(*target, split);
    const double side_forward = stop.s_mod_m >= target_projection.s_mod_m ?
      stop.s_mod_m - target_projection.s_mod_m :
      target->loop_length_m - target_projection.s_mod_m + stop.s_mod_m;
    const double center_remaining = std::max(
      0.0, channel_exit_unwrapped_s_m_ - channel_center_unwrapped_s_m_);
    const double route_to_split_m = pathLengthM(path) + side_forward;
    suffix = cyclicChannelSuffix(
      *target, target_projection.index, stop.index,
      route_to_split_m + 0.20 < center_remaining);
    kind = target_lane == ChannelLane::INNER ?
      ChannelActionKind::SWITCH_TO_INNER : ChannelActionKind::SWITCH_TO_OUTER;
  }
  if (suffix.poses.size() >= 2U) {
    path.poses.insert(path.poses.end(), suffix.poses.begin() + 1U, suffix.poses.end());
  }
  reason.clear();
  return path.poses.size() >= 2U;
}

bool CompetitionSupervisor::buildChannelTransitionV2(
  ChannelLane target_lane, nav_msgs::msg::Path & path,
  ChannelActionKind & kind, std::string & reason)
{
  if (active_channel_lane_ == target_lane ||
    (active_channel_lane_ != ChannelLane::CENTER && target_lane != ChannelLane::CENTER))
  {
    reason = "only Center<->Side transitions are legal"; return false;
  }
  const auto * target = channelTube(target_lane);
  if (!target) {reason = "direction-matched destination Tube unavailable"; return false;}
  const Pose2D actual = currentPose();
  const auto * active = channelTube(active_channel_lane_);
  if (!active) {reason = "direction-matched source Tube unavailable"; return false;}
  const auto active_projection = projectToChannelTube(*active, actual);
  const ChannelConnector * selected = nullptr;
  double best_score = std::numeric_limits<double>::infinity();
  double selected_forward_s_m = 0.0;
  for (const auto & connector : channel_v2_connectors_) {
    if (connector.clockwise != clockwise_ || connector.source_lane != active_channel_lane_ ||
      connector.target_lane != target_lane || connector.path.poses.empty()) {continue;}
    const auto & start = connector.path.poses.front().pose;
    const double xy = distance(actual.x, actual.y, start.position.x, start.position.y);
    const double yaw_error = std::abs(normalize(
      actual.yaw - yawOf(start.orientation)));
    const double forward_s = connector.source_s_m + 1.0e-9 >= active_projection.s_mod_m ?
      connector.source_s_m - active_projection.s_mod_m :
      active->loop_length_m - active_projection.s_mod_m + connector.source_s_m;
    const double score = forward_s + 0.10 * xy + 0.10 * yaw_error;
    if (forward_s <= 0.12 && xy <= 0.20 &&
      yaw_error <= 12.0 * kPi / 180.0 && score < best_score)
    {
      selected = &connector;
      selected_forward_s_m = forward_s;
      best_score = score;
    }
  }
  if (!selected) {
    reason = "no admitted forward-s connector anchor within 0.12m/0.20m/12deg";
    return false;
  }
  path = selected->path;
  path.header.stamp = now();
  for (auto & pose : path.poses) {pose.header = path.header;}
  const auto & finish = selected->path.poses.back().pose;
  const auto target_projection = projectToChannelTube(*target, Pose2D{
    finish.position.x, finish.position.y, yawOf(finish.orientation)});
  if (target_projection.distance_m > 0.025) {
    reason = "precomputed connector endpoint misses destination Tube"; return false;
  }
  nav_msgs::msg::Path suffix;
  if (target_lane == ChannelLane::CENTER) {
    suffix = centerPathToExit(target_projection.index);
    kind = active_channel_lane_ == ChannelLane::INNER ?
      ChannelActionKind::RETURN_FROM_INNER : ChannelActionKind::RETURN_FROM_OUTER;
  } else {
    const Pose2D split = clockwise_ ? Pose2D{2.961454, 3.18, kPi} :
      Pose2D{2.038546, 3.18, 0.0};
    const auto stop = projectToChannelTube(*target, split);
    const double side_forward = stop.s_mod_m >= target_projection.s_mod_m ?
      stop.s_mod_m - target_projection.s_mod_m :
      target->loop_length_m - target_projection.s_mod_m + stop.s_mod_m;
    const double center_remaining = std::max(
      0.0, channel_exit_unwrapped_s_m_ - channel_center_unwrapped_s_m_);
    // Decide whether a second Side lap is semantically necessary from the
    // complete forward route, not from the Side suffix alone.  Omitting the
    // current->anchor and connector distances made the retry-2 #0040 action
    // append a duplicate 8.92 m Outer lap even though its 0.66 m connector
    // already carried the route beyond the Center distance to the exit.
    const double route_to_split_m =
      selected_forward_s_m + pathLengthM(path) + side_forward;
    suffix = cyclicChannelSuffix(*target, target_projection.index, stop.index,
      route_to_split_m + 0.20 < center_remaining);
    kind = target_lane == ChannelLane::INNER ?
      ChannelActionKind::SWITCH_TO_INNER : ChannelActionKind::SWITCH_TO_OUTER;
  }
  if (suffix.poses.size() >= 2U) {
    path.poses.insert(path.poses.end(), suffix.poses.begin() + 1U, suffix.poses.end());
  }
  active_channel_connector_points_ = selected->path.poses.size();
  reason.clear();
  return path.poses.size() >= 2U;
}

std::uint8_t CompetitionSupervisor::coneSideClass(double x, double y) const
{
  const int ix = static_cast<int>(std::floor(x / kChannelConeLutResolutionM));
  const int iy = static_cast<int>(std::floor(y / kChannelConeLutResolutionM));
  if (ix < 0 || iy < 0 || ix >= static_cast<int>(kChannelConeLutWidth) ||
    iy >= static_cast<int>(kChannelConeLutHeight))
  {
    return 0U;
  }
  const auto index = static_cast<std::size_t>(iy) * kChannelConeLutWidth +
    static_cast<std::size_t>(ix);
  if (channel_avoidance_v2_enabled_) {
    const auto & lut = clockwise_ ? channel_avoid_lut_cw_ : channel_avoid_lut_ccw_;
    return index < lut.size() ? lut[index] : 0U;
  }
  return index < channel_cone_side_lut_.size() ? channel_cone_side_lut_[index] : 0U;
}

std::optional<std::pair<double, CompetitionSupervisor::ChannelLane>>
CompetitionSupervisor::nearestForwardConeTrigger()
{
  channel_v2_invalid_forward_cone_ = false;
  const auto * center = channelTube(ChannelLane::CENTER);
  if (!center) {return std::nullopt;}
  std::vector<std::pair<double, double>> cones;
  bool valid = false;
  rclcpp::Time receipt(0, 0, get_clock()->get_clock_type());
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    cones = latest_cone_centers_map_;
    valid = latest_cones_message_valid_;
    receipt = latest_cones_receipt_;
  }
  if (!valid || receipt.nanoseconds() == 0 ||
    (now() - receipt).seconds() > cones_freshness_sec_)
  {
    return std::nullopt;
  }
  std::optional<std::pair<double, ChannelLane>> best;
  for (const auto & cone : cones) {
    const auto projection = projectToChannelTube(
      *center, Pose2D{cone.first, cone.second, 0.0});
    const double forward = centerForwardDistance(
      channel_center_s_mod_m_, projection.s_mod_m);
    const double unwrapped = channel_center_unwrapped_s_m_ + forward;
    if (forward > channel_cone_lookahead_m_ ||
      unwrapped <= last_handled_cone_unwrapped_s_ + 0.03 ||
      projection.distance_m > channel_cone_lateral_relevance_m_)
    {
      continue;
    }
    const std::uint8_t code = coneSideClass(cone.first, cone.second);
    if (code == 0U) {
      if (channel_avoidance_v2_enabled_) {channel_v2_invalid_forward_cone_ = true;}
      continue;
    }
    const ChannelLane required = channel_avoidance_v2_enabled_ ?
      // V2 encodes cone position, not destination: cone on Inner side -> Outer.
      (code == 1U ? ChannelLane::OUTER : ChannelLane::INNER) :
      (code == 3U ? ChannelLane::INNER : ChannelLane::OUTER);
    if (!best || unwrapped < best->first) {best = std::make_pair(unwrapped, required);}
  }
  return best;
}

void CompetitionSupervisor::requestChannelEntry(bool fallback)
{
  const Pose2D goal = clockwise_ ?
    (fallback ? Pose2D{1.745932, kChannelEntryY, kPi} :
      Pose2D{1.895787, kChannelEntryY, kPi}) :
    (fallback ? Pose2D{3.254473, kChannelEntryY, 0.0} :
      Pose2D{3.104618, kChannelEntryY, 0.0});
  enter(State::GATE_PLAN, fallback ? "channel_entry_S2_GridBased" :
    "channel_entry_S1_GridBased");
  if (!arbiter_->computePath(
      poseStamped(goal), "GridBased", fallback ? "channel_entry_S2" : "channel_entry_S1",
      [this, fallback, goal](std::uint64_t, bool ok, const nav_msgs::msg::Path & connector,
        const std::string & detail) {
        publishAudit("channel_entry_plan_terminal", detail);
        std::string reason;
        const bool accepted = ok && validateNav2PathLight(connector, goal, reason);
        if (!accepted) {
          publishAudit(fallback ? "channel_entry_S2_rejected" : "channel_entry_S1_rejected", reason);
          if (!fallback) {requestChannelEntry(true);}
          else if (direct_entry_attempt_active_) {
            direct_entry_attempt_active_ = false;
            publishAudit(
              "direct_reverse_to_channel_entry_fallback_gate",
              "both direct Entry anchors rejected:" + reason);
            requestGatePath();
          } else {
            abandonChannelToReturn(
              "both verified GridBased Center entry anchors rejected:" + reason);
          }
          return;
        }
        direct_entry_attempt_active_ = false;
        beginChannelAtEntry(connector, fallback);
      }))
  {
    if (!fallback) {requestChannelEntry(true);}
    else if (direct_entry_attempt_active_) {
      direct_entry_attempt_active_ = false;
      publishAudit(
        "direct_reverse_to_channel_entry_fallback_gate",
        "GridBased arbiter refused both direct Entry anchors");
      requestGatePath();
    } else {
      abandonChannelToReturn("GridBased arbiter refused both Center entry anchors");
    }
  }
}

void CompetitionSupervisor::beginChannelAtEntry(
  const nav_msgs::msg::Path & connector, bool fallback)
{
  if (!channel_runtime_admitted_) {
    abandonChannelToReturn("verified Center/Side assets lack runtime admission");
    return;
  }
  const auto * center = channelTube(ChannelLane::CENTER);
  if (!center || connector.poses.size() < 2U) {
    abandonChannelToReturn("Center Tube or Gate-to-Center connector unavailable");
    return;
  }
  const std::size_t entry_index = clockwise_ ? (fallback ? 810U : 795U) :
    (fallback ? 205U : 190U);
  std::size_t handoff_index = entry_index;
  std::size_t runway_index = entry_index;
  double straight_advance_m = 0.0;
  const double runway_advance_m =
    kChannelEntryHandoffAdvanceM + kChannelEntryRunwayAfterHandoffM;
  for (std::size_t count = 0U; count < center->path.poses.size(); ++count) {
    const std::size_t next = (runway_index + 1U) % center->path.poses.size();
    const auto & a = center->path.poses[runway_index].pose.position;
    const auto & b = center->path.poses[next].pose.position;
    straight_advance_m += distance(a.x, a.y, b.x, b.y);
    runway_index = next;
    if (handoff_index == entry_index &&
      straight_advance_m + 1.0e-9 >= kChannelEntryHandoffAdvanceM)
    {
      handoff_index = runway_index;
    }
    if (straight_advance_m + 1.0e-9 >= runway_advance_m) {break;}
  }
  if (handoff_index == entry_index || runway_index == entry_index) {
    abandonChannelToReturn("Center entry straight lacks admitted handoff runway");
    return;
  }
  channel_entry_center_index_ = handoff_index;
  const Pose2D split = clockwise_ ? Pose2D{2.961454, 3.18, kPi} :
    Pose2D{2.038546, 3.18, 0.0};
  const auto exit_projection = projectToChannelTube(*center, split);
  channel_exit_center_index_ = exit_projection.index;
  channel_entry_s_m_ = center->s_m.at(entry_index);
  const auto & entry_pose = center->path.poses.at(handoff_index).pose;
  channel_entry_pose_ = Pose2D{
    entry_pose.position.x, entry_pose.position.y, yawOf(entry_pose.orientation)};
  channel_center_entry_reached_ = false;
  channel_center_s_mod_m_ = channel_entry_s_m_;
  channel_center_last_s_mod_m_ = channel_entry_s_m_;
  channel_center_unwrapped_s_m_ = channel_entry_s_m_;
  channel_exit_unwrapped_s_m_ = channel_entry_s_m_ +
    centerForwardDistance(channel_entry_s_m_, exit_projection.s_mod_m);
  channel_ring_complete_ = false;
  channel_gate_out_ = false;
  last_channel_abort_pose_.reset();
  last_channel_abort_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  return_plan_pending_ = false;
  return_preplan_attempted_ = false;
  return_plan_attempts_ = 0U;
  prepared_return_path_.reset();
  return_handoff_speed_limited_ = false;
  deferred_channel_lane_.reset();
  active_cone_trigger_unwrapped_s_.reset();
  last_handled_cone_unwrapped_s_ = -std::numeric_limits<double>::infinity();
  channel_lane_switch_count_ = 0U;
  channel_failed_recovery_count_ = 0U;
  pending_channel_retry_path_.reset();
  vlm_cache_armed_ = true;
  vlm_cache_armed_at_ = now();
  vlm_capture_window_active_ = false;
  vlm_capture_window_entered_at_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  publishGates();
  auto extended_connector = connector;
  std::size_t index = (entry_index + 1U) % center->path.poses.size();
  for (std::size_t count = 0U; count < center->path.poses.size(); ++count) {
    auto stamped = center->path.poses[index];
    stamped.header = extended_connector.header;
    extended_connector.poses.push_back(std::move(stamped));
    if (index == runway_index) {break;}
    index = (index + 1U) % center->path.poses.size();
  }
  std::ostringstream detail;
  detail << "entry=" << (fallback ? "S2" : "S1")
         << ",smac_entry_index=" << entry_index
         << ",rpp_handoff_index=" << handoff_index
         << ",runway_index=" << runway_index
         << ",handoff_advance_m=" << kChannelEntryHandoffAdvanceM
         << ",runway_after_handoff_m=" << kChannelEntryRunwayAfterHandoffM
         << ",center_s=" << channel_entry_s_m_
         << ",exit_index=" << channel_exit_center_index_
         << ",exit_unwrapped_s=" << channel_exit_unwrapped_s_m_;
  publishAudit("channel_entry_ready_for_connector", detail.str());
  publishChannelSpeedLimit(0.0);
  startChannelEntryConnector(extended_connector);
}

void CompetitionSupervisor::startChannelEntryConnector(
  const nav_msgs::msg::Path & connector)
{
  pending_channel_retry_path_.reset();
  std::string reason;
  if (!validateChannelPathLight(connector, reason)) {
    abandonChannelToReturn("Entry connector light path check rejected:" + reason);
    return;
  }
  auto path = connector;
  path.header.frame_id = map_frame_;
  path.header.stamp = now();
  for (auto & stamped : path.poses) {stamped.header = path.header;}
  channel_template_ = path;
  active_channel_path_ = std::make_shared<const nav_msgs::msg::Path>(path);
  active_channel_lane_ = ChannelLane::CENTER;
  active_channel_action_ = ChannelActionKind::ENTRY_CENTER;
  channel_progress_ = 0U;
  channel_entry_connector_active_ = true;
  channel_entry_path_progress_m_ = 0.0;
  channel_entry_handoff_path_s_m_ = 0.0;
  std::size_t handoff_index = 0U;
  double handoff_distance_m = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < path.poses.size(); ++index) {
    const auto & point = path.poses[index].pose.position;
    const double d = distance(
      point.x, point.y, channel_entry_pose_.x, channel_entry_pose_.y);
    if (d < handoff_distance_m) {
      handoff_distance_m = d;
      handoff_index = index;
    }
  }
  for (std::size_t index = 0U; index < handoff_index; ++index) {
    const auto & a = path.poses[index].pose.position;
    const auto & b = path.poses[index + 1U].pose.position;
    channel_entry_handoff_path_s_m_ += distance(a.x, a.y, b.x, b.y);
  }
  enter(State::CHANNEL_FOLLOW, "channel_entry_connector_follow");
  publishAudit("channel_entry_connector_sent", "controller=FollowPath");
  if (!arbiter_->sendFollowPath(
      path, "FollowPath", "general_goal_checker", "channel_entry",
      [this](std::uint64_t, rclcpp_action::ResultCode code, const std::string & detail) {
        if (state_ != State::CHANNEL_FOLLOW) {return;}
        if (code != rclcpp_action::ResultCode::SUCCEEDED) {
          if (race_continue_enabled_ && resumeChannelActivePath(detail)) {return;}
          channel_entry_connector_active_ = false;
          abandonChannelToReturn("Entry connector FollowPath failed:" + detail);
          return;
        }
        publishAudit("channel_entry_action_succeeded", detail);
        startCenterTubeAfterEntry(false);
      }))
  {
    pending_channel_retry_path_ = path;
    pending_channel_retry_lane_ = ChannelLane::CENTER;
    pending_channel_retry_kind_ = ChannelActionKind::ENTRY_CENTER;
    pending_channel_retry_replace_active_ = false;
    pending_channel_retry_not_before_ = now() + rclcpp::Duration::from_seconds(0.10);
    publishAudit(
      "channel_entry_action_deferred",
      "arbiter_refused=true,retry_in_sec=0.10,no_hold=true");
  }
}

void CompetitionSupervisor::startCenterTubeAfterEntry(bool replace_active)
{
  if (!channel_entry_connector_active_) {return;}
  const auto * center = channelTube(ChannelLane::CENTER);
  if (!center) {
    abandonChannelToReturn("Center canonical asset unavailable after Entry");
    return;
  }
  auto suffix = centerPathToExit(channel_entry_center_index_);
  if (suffix.poses.size() < 2U) {
    abandonChannelToReturn("Center canonical suffix unavailable after Entry");
    return;
  }
  channel_entry_connector_active_ = false;
  publishAudit(
    "channel_entry_reached",
    replace_active ? "pose=near_entry,hot_replace=true" :
    "pose=actual_connector_goal,hot_replace=false");
  std::ostringstream detail;
  detail << "controller=FollowPathChannel,entry_index=" << channel_entry_center_index_;
  publishAudit("center_tube_started", detail.str());
  startChannelPath(
    suffix, ChannelLane::CENTER, ChannelActionKind::CENTER_CRUISE, replace_active);
}

void CompetitionSupervisor::startPostGateFlow()
{
  if (mission_scope_ == "qr_reverse_gate") {
    enter(State::COMPLETE, "qr_gate_scope_complete");
    return;
  }
  // Gate has already reached its tolerant RGEG2 witness.  Channel always
  // begins with a new GridBased plan from the actual Gate terminal pose to a
  // verified Center Entry Lock; Reverse predictions are never reused here.
  requestChannelEntry(false);
}

void CompetitionSupervisor::abandonChannelToReturn(const std::string & reason)
{
  ++channel_failed_recovery_count_;
  publishAudit(
    "channel_abandon_to_return",
    "mission_hold=false,blind_motion=false,reason=" + reason);
  // If a verified Channel action is still alive, preserve it. This helper is
  // also used by replacement validation: canceling that old action would
  // manufacture the very stop this recovery policy is meant to avoid.
  if (arbiter_->motionActive() && active_channel_path_) {
    publishAudit(
      "channel_verified_path_preserved",
      "active_action=true,failed_replacement_discarded=true");
    return;
  }
  pending_channel_retry_path_.reset();
  channel_entry_connector_active_ = false;
  if (mission_scope_ == "channel_validation") {
    enter(State::COMPLETE, "channel_validation_complete_with_recovery_penalty");
    return;
  }
  // No verified Channel action remains. Continue through the ordinary Smac
  // Return-to-P path instead of issuing raw velocity or entering a terminal state.
  channel_ring_complete_ = true;
  channel_gate_out_ = true;
  publishEvent("C_RING_COMPLETE_RECOVERY");
  publishEvent("GATE_OUT_RECOVERY");
  requestReturnPath();
}

bool CompetitionSupervisor::requestChannelPathReplacement(
  const nav_msgs::msg::Path & path, ChannelLane target_lane,
  ChannelActionKind kind, const std::string & reason)
{
  if (state_ != State::CHANNEL_FOLLOW || channel_gate_out_ || return_plan_pending_) {
    return false;
  }
  std::string path_reason;
  if (!validateChannelPathLight(path, path_reason)) {
    publishAudit("channel_transition_light_check_rejected", path_reason);
    return false;
  }
  publishChannelSpeedLimit(channel_avoidance_v2_enabled_ ?
    channel_lane_change_ceiling_mps_ : 0.42);
  ++channel_lane_switch_count_;
  publishAudit("channel_lane_switch_hot_replace", reason +
    ",switch_count=" + std::to_string(channel_lane_switch_count_));
  startChannelPath(path, target_lane, kind, true);
  return true;
}

bool CompetitionSupervisor::resumeChannelActivePath(
  const std::string & terminal_detail)
{
  if (!race_continue_enabled_ || !active_channel_path_ ||
    active_channel_path_->poses.size() < 2U)
  {
    return false;
  }
  const Pose2D pose = currentPose();
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw)) {
    return false;
  }

  const auto & active = *active_channel_path_;
  std::size_t nearest = std::min(channel_progress_, active.poses.size() - 1U);
  double nearest_distance = std::numeric_limits<double>::infinity();
  bool found_forward = false;
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  for (std::size_t index = nearest; index < active.poses.size(); ++index) {
    const auto & point = active.poses[index].pose.position;
    const double dx = point.x - pose.x;
    const double dy = point.y - pose.y;
    const double forward = dx * c + dy * s;
    const double separation = std::hypot(dx, dy);
    if (forward >= -0.02 && separation < nearest_distance) {
      nearest = index;
      nearest_distance = separation;
      found_forward = true;
    }
  }
  if (!found_forward) {
    for (std::size_t index = std::min(channel_progress_, active.poses.size() - 1U);
      index < active.poses.size(); ++index)
    {
      const auto & point = active.poses[index].pose.position;
      const double separation = distance(pose.x, pose.y, point.x, point.y);
      if (separation < nearest_distance) {
        nearest = index;
        nearest_distance = separation;
      }
    }
  }

  const bool repeated_same_pose = last_channel_abort_pose_ &&
    last_channel_abort_stamp_.nanoseconds() != 0 &&
    (now() - last_channel_abort_stamp_).seconds() <= 2.0 &&
    distance(pose.x, pose.y, last_channel_abort_pose_->x,
      last_channel_abort_pose_->y) <= 0.10;
  const double skip_distance = repeated_same_pose ? 0.15 : 0.03;
  double skipped = 0.0;
  while (nearest + 2U < active.poses.size() && skipped < skip_distance) {
    const auto & a = active.poses[nearest].pose.position;
    const auto & b = active.poses[nearest + 1U].pose.position;
    skipped += distance(a.x, a.y, b.x, b.y);
    ++nearest;
  }

  nav_msgs::msg::Path suffix;
  suffix.header.frame_id = map_frame_;
  suffix.header.stamp = now();
  for (std::size_t index = nearest; index < active.poses.size(); ++index) {
    auto stamped = active.poses[index];
    stamped.header = suffix.header;
    suffix.poses.push_back(std::move(stamped));
  }
  std::string path_reason;
  if (!validateChannelPathLight(suffix, path_reason)) {return false;}

  last_channel_abort_pose_ = pose;
  last_channel_abort_stamp_ = now();
  ++channel_failed_recovery_count_;
  publishAudit(
    "channel_abort_resume_current_path",
    "lane_and_action_preserved=true,start_index=" + std::to_string(nearest) +
    ",nearest_distance_m=" + std::to_string(nearest_distance) +
    ",same_pose_skip=" + (repeated_same_pose ? "true" : "false") +
    ",skip_m=" + std::to_string(skipped) +
    ",speed_schedule=unchanged,terminal=" + terminal_detail);
  if (active_channel_action_ == ChannelActionKind::ENTRY_CENTER &&
    channel_entry_connector_active_)
  {
    startChannelEntryConnector(suffix);
  } else {
    startChannelPath(
      suffix, active_channel_lane_, active_channel_action_, false);
  }
  return true;
}

bool CompetitionSupervisor::continueActiveSideTube(
  const std::string & terminal_detail)
{
  if (!race_continue_enabled_ || active_channel_lane_ == ChannelLane::CENTER) {
    return false;
  }
  const auto * tube = channelTube(active_channel_lane_);
  if (!tube || tube->path.poses.size() < 3U) {return false;}
  const auto projection = projectToChannelTube(*tube, currentPose());
  const std::size_t start = (projection.index + 1U) % tube->path.poses.size();
  auto continuation = cyclicChannelSuffix(*tube, start, projection.index, true);
  std::string reason;
  if (!validateChannelPathLight(continuation, reason)) {return false;}
  ++channel_failed_recovery_count_;
  publishAudit(
    "channel_side_continue_current_tube",
    "return_connector_temporarily_unavailable=true,start_index=" +
    std::to_string(start) + ",speed_schedule=unchanged,terminal=" + terminal_detail);
  startChannelPath(
    continuation, active_channel_lane_, active_channel_action_, false);
  return true;
}

void CompetitionSupervisor::maybeHandleChannelCone()
{
  const double remaining_to_exit = channel_exit_unwrapped_s_m_ -
    channel_center_unwrapped_s_m_;
  if (active_channel_lane_ != ChannelLane::CENTER &&
    (remaining_to_exit <= channel_cone_lookahead_m_ ||
    (active_cone_trigger_unwrapped_s_ &&
    channel_center_unwrapped_s_m_ >=
    *active_cone_trigger_unwrapped_s_ + channel_cone_passed_margin_m_)))
  {
    nav_msgs::msg::Path path;
    ChannelActionKind kind;
    std::string reason;
    if (buildChannelTransition(ChannelLane::CENTER, path, kind, reason)) {
      requestChannelPathReplacement(path, ChannelLane::CENTER, kind,
        "passed cone trigger or approaching Center-only exit");
    }
    return;
  }
  const auto trigger = nearestForwardConeTrigger();
  if (channel_v2_invalid_forward_cone_) {
    std::uint64_t cone_sequence = 0U;
    {
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      cone_sequence = latest_cones_receive_sequence_;
    }
    if (cone_sequence != channel_last_cone_sequence_) {
      channel_last_cone_sequence_ = cone_sequence;
      publishAudit(
        "channel_invalid_forward_cone_ignored",
        "continue_current_verified_path=true,sequence=" + std::to_string(cone_sequence));
    }
    return;
  }
  if (!trigger || remaining_to_exit <= channel_cone_lookahead_m_) {
    if (active_channel_lane_ == ChannelLane::CENTER && deferred_channel_lane_) {
      publishAudit("channel_deferred_lane_cleared", "cone no longer forward/relevant");
      deferred_channel_lane_.reset();
    }
    return;
  }
  const auto [trigger_s, required_lane] = *trigger;
  if (active_channel_lane_ == ChannelLane::CENTER && deferred_channel_lane_) {
    if (*deferred_channel_lane_ != required_lane) {
      publishAudit("channel_deferred_lane_replaced", "nearest forward cone side changed");
      deferred_channel_lane_ = required_lane;
    } else {
      publishAudit("channel_deferred_lane_consumed", "Side->Center handoff complete");
    }
  }
  if (active_channel_lane_ == required_lane) {
    active_cone_trigger_unwrapped_s_ = trigger_s;
    last_handled_cone_unwrapped_s_ = std::max(last_handled_cone_unwrapped_s_, trigger_s);
    publishAudit("channel_same_side_cone", "remain on current Side Tube");
    return;
  }
  if (active_channel_lane_ != ChannelLane::CENTER) {
    deferred_channel_lane_ = required_lane;
    nav_msgs::msg::Path path;
    ChannelActionKind kind;
    std::string reason;
    if (buildChannelTransition(ChannelLane::CENTER, path, kind, reason)) {
      requestChannelPathReplacement(path, ChannelLane::CENTER, kind,
        "opposite cone requires Side->Center before later switch");
    }
    return;
  }
  nav_msgs::msg::Path path;
  ChannelActionKind kind;
  std::string reason;
  if (!buildChannelTransition(required_lane, path, kind, reason)) {
    publishAudit("channel_transition_deferred", reason);
    return;
  }
  if (requestChannelPathReplacement(
      path, required_lane, kind,
      channel_avoidance_v2_enabled_ ?
      (required_lane == ChannelLane::INNER ? "cone OUTER_SIDE -> Inner" :
      "cone INNER_SIDE -> Outer") :
      (required_lane == ChannelLane::INNER ? "cone class 3 -> Inner" :
      "cone class 1/2 -> Outer")))
  {
    active_cone_trigger_unwrapped_s_ = trigger_s;
    last_handled_cone_unwrapped_s_ = trigger_s;
    deferred_channel_lane_.reset();
  }
}

bool CompetitionSupervisor::returnHandoffProgressReached(
  const Pose2D & pose, ChannelProjection * projection_out,
  double * anchor_station_out) const
{
  const auto & branch = clockwise_ ? channel_exit_cw_path_ : channel_exit_ccw_path_;
  if (branch.poses.size() < 2U) {return false;}
  // The physical-clearance anchor remains tied to the admitted exit asset,
  // but reaching it is monotonic path progress rather than a one-sample
  // distance-and-yaw coincidence.  This prevents a small tracking offset from
  // making the vehicle miss Gate-Out forever and chase an exhausted path.
  auto it = std::find_if(
    branch.poses.begin(), branch.poses.end(), [](const auto & stamped) {
      return stamped.pose.position.y < 2.30;
    });
  if (it == branch.poses.end()) {it = std::prev(branch.poses.end());}
  const std::size_t anchor_index = static_cast<std::size_t>(
    std::distance(branch.poses.begin(), it));

  const ChannelProjection projection = projectToOpenPath(branch, pose);
  double anchor_station_m = 0.0;
  for (std::size_t index = 0U; index < anchor_index; ++index) {
    const auto & a = branch.poses[index].pose.position;
    const auto & b = branch.poses[index + 1U].pose.position;
    anchor_station_m += distance(a.x, a.y, b.x, b.y);
  }
  if (projection_out) {*projection_out = projection;}
  if (anchor_station_out) {*anchor_station_out = anchor_station_m;}
  return std::isfinite(projection.distance_m) &&
         projection.s_mod_m + 1.0e-3 >= anchor_station_m;
}

void CompetitionSupervisor::updateChannelProgress()
{
  if (channel_entry_connector_active_) {
    const Pose2D pose = currentPose();
    if (active_channel_path_) {
      const auto projection = projectToOpenPath(*active_channel_path_, pose);
      if (std::isfinite(projection.s_mod_m)) {
        channel_entry_path_progress_m_ = std::max(
          channel_entry_path_progress_m_, projection.s_mod_m);
      }
    }
    // Smac owns the turn and the first 25 cm of the admitted straight.  Its
    // path continues for another 15 cm, so crossing this one monotonic station
    // can hot-replace RPP without a goal slowdown or any yaw/distance/timer gate.
    if (channel_entry_path_progress_m_ + 1.0e-3 >=
      channel_entry_handoff_path_s_m_)
    {
      std::ostringstream detail;
      detail << "entry_progress_m=" << channel_entry_path_progress_m_
             << ",handoff_station_m=" << channel_entry_handoff_path_s_m_
             << ",hot_replace=true,no_wait=true";
      publishAudit("channel_entry_handoff_ready", detail.str());
      startCenterTubeAfterEntry(true);
    }
    return;
  }
  if (state_ != State::CHANNEL_FOLLOW)
  {
    return;
  }
  const auto * center = channelTube(ChannelLane::CENTER);
  if (!center) {
    abandonChannelToReturn("Center canonical progress asset unavailable");
    return;
  }
  const Pose2D pose = currentPose();
  if (!channel_center_entry_reached_) {
    // The only route into this state is the latched entry-path station above
    // (or successful completion of its forward runway), so no second spatial
    // gate is needed to begin Center progress accounting.
    channel_center_entry_reached_ = true;
    channel_center_s_mod_m_ = channel_entry_s_m_;
    channel_center_last_s_mod_m_ = channel_entry_s_m_;
    channel_center_unwrapped_s_m_ = channel_entry_s_m_;
    publishEvent("C_RING_ENTRY");
  }
  const auto center_projection = projectToChannelTube(*center, pose);
  const bool correction_event = bev_correction_pending_.exchange(
    false, std::memory_order_acq_rel);
  double delta = center_projection.s_mod_m - channel_center_last_s_mod_m_;
  if (delta > center->loop_length_m / 2.0) {delta -= center->loop_length_m;}
  if (delta < -center->loop_length_m / 2.0) {delta += center->loop_length_m;}
  if (delta >= -0.03) {
    channel_center_unwrapped_s_m_ += std::max(0.0, delta);
    channel_center_s_mod_m_ = center_projection.s_mod_m;
    channel_center_last_s_mod_m_ = center_projection.s_mod_m;
  } else if (correction_event) {
    publishAudit("bev_channel_reproject", "Center s reprojected without semantic rollback");
    channel_center_s_mod_m_ = center_projection.s_mod_m;
    channel_center_last_s_mod_m_ = center_projection.s_mod_m;
  }
  if (!channel_ring_complete_ &&
    channel_center_unwrapped_s_m_ + 0.03 >= channel_exit_unwrapped_s_m_)
  {
    channel_ring_complete_ = true;
    publishEvent("C_RING_COMPLETE");
  }
  const double lap_span = std::max(0.01, channel_exit_unwrapped_s_m_ - channel_entry_s_m_);
  const double fraction = (channel_center_unwrapped_s_m_ - channel_entry_s_m_) / lap_span;
  const bool inside_vlm_straight = !vlm_received_ &&
    fraction > kVlmStraightFractionMin && fraction < kVlmStraightFractionMax &&
    pose.y > kVlmStraightYMinM;
  if (inside_vlm_straight != vlm_capture_window_active_) {
    vlm_capture_window_active_ = inside_vlm_straight;
    if (inside_vlm_straight) {
      vlm_capture_window_entered_at_ = now();
      publishAudit(
        "vlm_detector_window_open",
        "top_straight=true,navigation_control=false");
    } else {
      publishAudit(
        "vlm_detector_window_closed",
        "left_top_straight=true,inflight_vlm_continues=true");
    }
    publishGates();
  }
  if (vlm_cache_armed_ && !vlm_triggered_ &&
    (now() - vlm_cache_armed_at_).seconds() >= 0.30 &&
    vlm_capture_window_active_ &&
    (now() - vlm_capture_window_entered_at_).seconds() >= kVlmDetectorGateSettleSec)
  {
    vlm_triggered_ = true;
    publishGates();
    std_msgs::msg::String trigger;
    trigger.data = "describe_patient";
    vlm_trigger_pub_->publish(trigger);
    publishEvent("C_VLM_TRIGGER");
  }
  maybeHandleChannelCone();

  if (active_channel_path_ && !active_channel_path_->poses.empty()) {
    std::size_t best = 0U;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0U; i < active_channel_path_->poses.size(); ++i) {
      const auto & point = active_channel_path_->poses[i].pose.position;
      const double d = distance(pose.x, pose.y, point.x, point.y);
      if (d < best_distance) {best_distance = d; best = i;}
    }
    channel_progress_ = std::max(channel_progress_, best);
  }
  // The outbound Connector+Side bypass is already one continuous RPP path.
  // Start the Side->Center replacement before that path reaches its terminal
  // pose so Nav2 can accept the next goal while the old command remains live.
  // This removes the last normal lane-change send-after-terminal gap without
  // changing Tube geometry, controller tuning, or the speed schedule.
  if (active_channel_path_ &&
    (active_channel_action_ == ChannelActionKind::SWITCH_TO_INNER ||
    active_channel_action_ == ChannelActionKind::SWITCH_TO_OUTER))
  {
    double remaining_path_m = 0.0;
    for (std::size_t index = std::min(
        channel_progress_, active_channel_path_->poses.size() - 1U);
      index + 1U < active_channel_path_->poses.size(); ++index)
    {
      const auto & a = active_channel_path_->poses[index].pose.position;
      const auto & b = active_channel_path_->poses[index + 1U].pose.position;
      remaining_path_m += distance(a.x, a.y, b.x, b.y);
    }
    if (remaining_path_m <= 0.30) {
      nav_msgs::msg::Path return_path;
      ChannelActionKind return_kind;
      std::string return_reason;
      const auto old_connector_points = active_channel_connector_points_;
      if (buildChannelTransition(
          ChannelLane::CENTER, return_path, return_kind, return_reason))
      {
        if (requestChannelPathReplacement(
            return_path, ChannelLane::CENTER, return_kind,
            "proactive Side->Center hot handoff,remaining_m=" +
            std::to_string(remaining_path_m)))
        {
          return;
        }
        active_channel_connector_points_ = old_connector_points;
      }
    }
  }
  bool transition_phase = false;
  switch (active_channel_action_) {
    case ChannelActionKind::SWITCH_TO_INNER:
    case ChannelActionKind::SWITCH_TO_OUTER:
    case ChannelActionKind::RETURN_FROM_INNER:
    case ChannelActionKind::RETURN_FROM_OUTER:
      transition_phase = channel_progress_ < (channel_avoidance_v2_enabled_ ?
        active_channel_connector_points_ : channel_transition_bottom_plus_.size());
      break;
    default: break;
  }
  bool corner_phase = false;
  if (const auto * active_tube = channelTube(active_channel_lane_)) {
    const auto projection = projectToChannelTube(*active_tube, pose);
    if (channel_corner_last_tube_ != active_tube) {
      channel_corner_last_tube_ = active_tube;
      channel_corner_hold_remaining_m_ = 0.0;
      channel_corner_projection_valid_ = false;
    }
    if (channel_corner_projection_valid_) {
      double delta = projection.s_mod_m - channel_corner_last_s_m_;
      if (delta > active_tube->loop_length_m / 2.0) {
        delta -= active_tube->loop_length_m;
      } else if (delta < -active_tube->loop_length_m / 2.0) {
        delta += active_tube->loop_length_m;
      }
      if (delta >= -0.03) {
        channel_corner_hold_remaining_m_ = std::max(
          0.0, channel_corner_hold_remaining_m_ - std::max(0.0, delta));
      } else {
        channel_corner_hold_remaining_m_ = 0.0;
      }
    }
    channel_corner_last_s_m_ = projection.s_mod_m;
    channel_corner_projection_valid_ = true;
    std::size_t index = projection.index;
    double preview_distance = 0.0;
    double last_high_curvature_distance = 0.0;
    bool high_curvature_seen = false;
    for (std::size_t count = 0U; count < active_tube->path.poses.size(); ++count) {
      if (index < active_tube->kappa_1pm.size() &&
        std::abs(active_tube->kappa_1pm[index]) >
        channel_corner_curvature_threshold_1pm_)
      {
        corner_phase = true;
        high_curvature_seen = true;
        last_high_curvature_distance = preview_distance;
      }
      const std::size_t next = (index + 1U) % active_tube->path.poses.size();
      const auto & a = active_tube->path.poses[index].pose.position;
      const auto & b = active_tube->path.poses[next].pose.position;
      preview_distance += distance(a.x, a.y, b.x, b.y);
      if (preview_distance >= channel_corner_preview_distance_m_) {break;}
      index = next;
    }
    if (high_curvature_seen) {
      channel_corner_hold_remaining_m_ = std::max(
        channel_corner_hold_remaining_m_,
        last_high_curvature_distance + channel_corner_exit_hold_distance_m_);
    }
    corner_phase = corner_phase || channel_corner_hold_remaining_m_ > 0.0;
  }
  // After the Center lap completes, the active action continues on the
  // admitted exit branch rather than the cyclic Tube.  Evaluate that actual
  // path around the monotonic active-path projection so the final bend keeps
  // the 0.50 m/s corner ceiling and releases to 0.70 only after straightening.
  // A bounded look-behind supplies the existing exit hold without state reset.
  if (channel_ring_complete_ && active_channel_lane_ == ChannelLane::CENTER &&
    active_channel_path_ && active_channel_path_->poses.size() >= 3U)
  {
    const auto & path = *active_channel_path_;
    const std::size_t progress = std::min(
      channel_progress_, path.poses.size() - 2U);
    const auto segment_is_corner = [this, &path](std::size_t index) {
        if (index + 1U >= path.poses.size()) {return false;}
        const auto & a = path.poses[index].pose;
        const auto & b = path.poses[index + 1U].pose;
        const double ds = distance(
          a.position.x, a.position.y, b.position.x, b.position.y);
        if (ds <= 1.0e-6) {return false;}
        const double curvature = std::abs(normalize(
          yawOf(b.orientation) - yawOf(a.orientation))) / ds;
        return curvature > channel_corner_curvature_threshold_1pm_;
      };

    double preview_m = 0.0;
    for (std::size_t index = progress;
      index + 1U < path.poses.size() &&
      preview_m <= channel_corner_preview_distance_m_; ++index)
    {
      corner_phase = corner_phase || segment_is_corner(index);
      const auto & a = path.poses[index].pose.position;
      const auto & b = path.poses[index + 1U].pose.position;
      preview_m += distance(a.x, a.y, b.x, b.y);
    }
    double lookbehind_m = 0.0;
    for (std::size_t index = progress; index > 0U &&
      lookbehind_m <= channel_corner_exit_hold_distance_m_; --index)
    {
      corner_phase = corner_phase || segment_is_corner(index - 1U);
      const auto & a = path.poses[index - 1U].pose.position;
      const auto & b = path.poses[index].pose.position;
      lookbehind_m += distance(a.x, a.y, b.x, b.y);
    }
  }
  if (channel_avoidance_v2_enabled_) {
    publishChannelSpeedLimit(transition_phase ? channel_lane_change_ceiling_mps_ :
      (corner_phase ? channel_corner_ceiling_mps_ : channel_straight_ceiling_mps_));
  } else {
    publishChannelSpeedLimit(transition_phase || corner_phase ? 0.42 : 0.0);
  }
  if (channel_ring_complete_ && active_channel_lane_ == ChannelLane::CENTER &&
    !return_preplan_attempted_ && !return_plan_pending_ && !prepared_return_path_)
  {
    prepareReturnPathWhileFollowing();
  }
  ChannelProjection exit_projection;
  double handoff_station_m = 0.0;
  if (channel_ring_complete_ && active_channel_lane_ == ChannelLane::CENTER &&
    returnHandoffProgressReached(pose, &exit_projection, &handoff_station_m))
  {
    if (!channel_gate_out_) {
      channel_gate_out_ = true;
      const auto & branch = clockwise_ ? channel_exit_cw_path_ : channel_exit_ccw_path_;
      const auto & projected_pose = branch.poses.at(std::min(
          exit_projection.index, branch.poses.size() - 1U)).pose;
      std::ostringstream detail;
      detail << "exit_station_m=" << exit_projection.s_mod_m
             << ",handoff_station_m=" << handoff_station_m
             << ",cross_track_m=" << exit_projection.distance_m
             << ",yaw_error_rad=" << std::abs(normalize(
          pose.yaw - yawOf(projected_pose.orientation)))
             << ",latched=true";
      publishAudit("channel_gate_out_progress_latched", detail.str());
      publishEvent("GATE_OUT");
    }
    if (prepared_return_path_) {
      auto path = std::move(*prepared_return_path_);
      prepared_return_path_.reset();
      startReturnPath(path, true);
    } else if (!return_plan_pending_) {
      // The single speculative preplan may legitimately fail while Channel is
      // still active (for example while the live start footprint is lethal).
      // Do not retry it at the 20 Hz progress rate. At the actual handoff use
      // the existing bounded live-pose Return planner instead.
      requestReturnPath();
    }
  }
}

void CompetitionSupervisor::publishChannelSpeedLimit(double limit_mps)
{
  if (!channel_speed_limit_pub_ ||
    (last_channel_speed_limit_mps_ &&
    std::abs(*last_channel_speed_limit_mps_ - limit_mps) <= 1.0e-9))
  {
    return;
  }
  nav2_msgs::msg::SpeedLimit limit;
  limit.header.frame_id = map_frame_;
  limit.header.stamp = now();
  limit.percentage = false;
  // Nav2 defines zero as removal of the dynamic speed restriction. The
  // command limiter still enforces the launch-selected absolute ceiling.
  limit.speed_limit = limit_mps;
  channel_speed_limit_pub_->publish(limit);
  last_channel_speed_limit_mps_ = limit_mps;
  std::ostringstream detail;
  detail << "limit_mps=" << limit_mps << ",mode=" <<
    (channel_avoidance_v2_enabled_ ? "v2_three_ceiling" : "legacy");
  publishAudit("channel_speed_limit", detail.str());
}

void CompetitionSupervisor::startChannelPath(
  const nav_msgs::msg::Path & path, ChannelLane lane, ChannelActionKind kind,
  bool replace_active)
{
  pending_channel_retry_path_.reset();
  std::string reason;
  if (!validateChannelPathLight(path, reason))
  {
    abandonChannelToReturn("light path check rejected Channel path:" + reason);
    return;
  }
  active_channel_lane_ = lane;
  active_channel_action_ = kind;
  channel_template_ = path;
  active_channel_path_ = std::make_shared<const nav_msgs::msg::Path>(path);
  channel_corner_hold_remaining_m_ = 0.0;
  channel_corner_last_s_m_ = 0.0;
  channel_corner_last_tube_ = nullptr;
  channel_corner_projection_valid_ = false;
  channel_progress_ = 0U;
  enter(State::CHANNEL_FOLLOW, "channel_center_side_follow");
  const bool gate_to_center_entry = kind == ChannelActionKind::ENTRY_CENTER;
  const std::string controller_id = gate_to_center_entry ?
    "FollowPath" : "FollowPathChannel";
  auto on_result =
    [this, path, lane, kind, replace_active](
      std::uint64_t, rclcpp_action::ResultCode code,
      const std::string & detail) {
        if (state_ != State::CHANNEL_FOLLOW) {return;}
        publishAudit("channel_action_terminal", detail);
        if (replace_active &&
          detail.find("old_goal_completed_before_accept") != std::string::npos)
        {
          publishAudit(
            "channel_hot_replace_race_fallback",
            "old action completed first; sending prepared path without a release poll");
          startChannelPath(path, lane, kind, false);
          return;
        }
        if (code == rclcpp_action::ResultCode::SUCCEEDED &&
          (active_channel_action_ == ChannelActionKind::SWITCH_TO_INNER ||
          active_channel_action_ == ChannelActionKind::SWITCH_TO_OUTER ||
          active_channel_action_ == ChannelActionKind::RETURN_FROM_INNER ||
          active_channel_action_ == ChannelActionKind::RETURN_FROM_OUTER))
        {
          // A replacement FollowPath is a bounded connector/Tube segment.
          // For an outbound Center->Side segment, immediately chain the
          // precomputed Side->Center return connector; otherwise the vehicle
          // would legitimately reach the segment endpoint with no active
          // action and appear to stop at the cone.
          if (active_channel_action_ == ChannelActionKind::SWITCH_TO_INNER ||
            active_channel_action_ == ChannelActionKind::SWITCH_TO_OUTER)
          {
            nav_msgs::msg::Path return_path;
            ChannelActionKind return_kind;
            std::string return_reason;
            if (!buildChannelTransition(
                ChannelLane::CENTER, return_path, return_kind, return_reason))
            {
              if (!continueActiveSideTube(return_reason)) {
                abandonChannelToReturn(
                  "Side Tube continuation unavailable:" + return_reason);
              }
              return;
            }
            publishAudit("channel_side_bypass_succeeded", detail);
            startChannelPath(return_path, ChannelLane::CENTER, return_kind);
            return;
          }
          publishAudit("channel_lane_switch_succeeded", detail);
          const Pose2D pose = currentPose();
          if (channel_ring_complete_ && returnHandoffProgressReached(pose)) {
            channel_gate_out_ = true;
            publishEvent("GATE_OUT");
            requestReturnPath();
          }
          return;
        }
        if (code == rclcpp_action::ResultCode::SUCCEEDED &&
          mission_scope_ == "channel_validation")
        {
          enter(State::COMPLETE, "channel_validation_scope_complete");
          return;
        }
        const Pose2D pose = currentPose();
        // The canonical Center action contains the complete unwrapped Tube
        // suffix followed by the gate-exit branch to the Return handoff.  On
        // Humble, the action result can arrive before the periodic progress
        // callback observes the final Center projection, so C_RING_COMPLETE
        // may still be false even though the terminal pose is valid. Recover
        // that semantic edge only for the verified Center cruise action after
        // its projection crosses the admitted exit-branch handoff station;
        // every other terminal remains isolated from this shortcut.
        if (code == rclcpp_action::ResultCode::SUCCEEDED &&
          active_channel_action_ == ChannelActionKind::CENTER_CRUISE &&
          active_channel_lane_ == ChannelLane::CENTER &&
          returnHandoffProgressReached(pose))
        {
          if (!channel_ring_complete_) {
            channel_ring_complete_ = true;
            publishEvent("C_RING_COMPLETE");
          }
          channel_gate_out_ = true;
          publishEvent("GATE_OUT");
          requestReturnPath();
          return;
        }
        if (code == rclcpp_action::ResultCode::SUCCEEDED && channel_ring_complete_ &&
          active_channel_lane_ == ChannelLane::CENTER)
        {
          channel_gate_out_ = true;
          publishEvent("GATE_OUT");
          publishAudit(
            "channel_center_terminal_handoff",
            "verified_center_exit_action_succeeded=true,full_tube_replay=false");
          requestReturnPath();
          return;
        }
        if (race_continue_enabled_) {
          if (!resumeChannelActivePath(detail)) {
            publishAudit(
              "channel_terminal_suffix_exhausted_return",
              "suffix_unavailable=true,full_tube_replay=false,mission_hold=false,terminal=" +
              detail);
            // Replaying `path` here starts a completed cyclic Tube from index
            // zero. The physical run then drove another lap and entered an
            // ABORT/resend loop. With no forward suffix left, hand the actual
            // live pose to the bounded Return planner instead.
            channel_ring_complete_ = true;
            channel_gate_out_ = true;
            publishEvent("C_RING_COMPLETE_RECOVERY");
            publishEvent("GATE_OUT_RECOVERY");
            requestReturnPath();
          }
          return;
        }
        abandonChannelToReturn("unexpected FollowPathChannel terminal:" + detail);
      };
  const bool sent = replace_active ?
    arbiter_->replaceFollowPath(
      path, controller_id, "general_goal_checker", "channel", std::move(on_result)) :
    arbiter_->sendFollowPath(
      path, controller_id, "general_goal_checker", "channel", std::move(on_result));
  if (!sent)
  {
    pending_channel_retry_path_ = path;
    pending_channel_retry_lane_ = lane;
    pending_channel_retry_kind_ = kind;
    pending_channel_retry_replace_active_ = replace_active;
    pending_channel_retry_not_before_ = now() + rclcpp::Duration::from_seconds(0.10);
    publishAudit(
      "channel_action_deferred",
      "arbiter_refused=true,retry_in_sec=0.10,no_hold=true,replace_active=" +
      std::string(replace_active ? "true" : "false"));
  }
}

void CompetitionSupervisor::requestReturnPath(bool backup)
{
  (void)backup;
  if (mission_scope_ != "qr_reverse_gate" &&
    !(channel_ring_complete_ && channel_gate_out_))
  {
    publishAudit(
      "return_request_deferred",
      "mission_hold=false,reason=waiting_for_C_RING_COMPLETE_and_GATE_OUT");
    return;
  }
  if (state_ == State::RETURN_FOLLOW) {return;}
  if (prepared_return_path_) {
    auto path = std::move(*prepared_return_path_);
    prepared_return_path_.reset();
    startReturnPath(path, arbiter_->motionActive());
    return;
  }
  if (return_plan_pending_) {
    publishAudit("return_plan_wait", "background Return plan is still pending");
    return;
  }
  return_action_succeeded_ = false;
  enter(State::RETURN_PLAN, "online_return_to_P_plan_fallback");
  // P is a parking region, not a single-heading pose.  ReturnPGoalChecker
  // accepts the full yaw circle while the footprint/position gates remain
  // strict; use the calibrated natural P heading only as the planner hint.
  // The measured outer white P frame is centred at (0.50, 0.20).  Return
  // approaches from the channel along -Y, so keep the planner hint aligned
  // with that entry direction.  ReturnPGoalChecker still accepts any yaw.
  const Pose2D finish{0.50, 0.20, -kPi / 2.0};
  ++return_plan_attempts_;
  const std::string tag = "return_P_attempt_" + std::to_string(return_plan_attempts_);
  if (!arbiter_->computePath(poseStamped(finish), "GridBased", tag,
    [this, tag, finish](std::uint64_t, bool ok, const nav_msgs::msg::Path & path,
      const std::string & detail) {
      publishAudit("return_plan_terminal", detail);
      std::string reason;
      // Return uses the same planner/controller chain as P->QR.  A stale
      // global-costmap generation during the handoff must not suppress a
      // planner result and falsely end the mission before FollowPath starts.
      const bool accepted = ok && validateNav2PathLight(
        path, finish, reason, kPi, false);
      if (!accepted) {
        publishAudit("return_path_rejected", reason);
        if (race_continue_enabled_ && return_plan_attempts_ < 3U) {
          requestReturnPath();
          return;
        }
        enter(State::COMPLETE, "mission_complete_with_return_P_plan_penalty");
        return;
      }
      startReturnPath(path, false);
  })) {
    publishAudit("return_plan_arbiter_refused", tag);
    if (race_continue_enabled_ && return_plan_attempts_ < 3U) {
      requestReturnPath();
      return;
    }
    enter(State::COMPLETE, "mission_complete_with_return_P_arbiter_penalty");
  }
}

void CompetitionSupervisor::prepareReturnPathWhileFollowing()
{
  if (state_ != State::CHANNEL_FOLLOW || !arbiter_->motionActive() ||
    return_preplan_attempted_ || return_plan_pending_ || prepared_return_path_)
  {
    return;
  }
  const Pose2D finish{0.50, 0.20, -kPi / 2.0};
  const std::string tag = "return_P_preplan";
  // This is a speculative optimization, not a retry loop. A failure leaves
  // the verified Channel action untouched and is retried only by the bounded
  // normal Return planner after the physical Gate-Out handoff.
  return_preplan_attempted_ = true;
  return_plan_pending_ = true;
  publishAudit(
    "return_preplan_requested",
    "source=current_live_pose,attempt=1,max_attempts=1,old_channel_continues=true");
  if (!arbiter_->computePathWhileFollowing(
      poseStamped(finish), "GridBased", tag,
      [this, finish](std::uint64_t, bool ok, const nav_msgs::msg::Path & path,
        const std::string & detail) {
        return_plan_pending_ = false;
        if (state_ != State::CHANNEL_FOLLOW) {return;}
        std::string reason;
        const bool accepted = ok && validateNav2PathLight(
          path, finish, reason, kPi, false);
        publishAudit(
          "return_preplan_terminal",
          accepted ? detail + ",accepted=true" : detail + ",accepted=false,reason=" + reason);
        if (!accepted) {
          if (channel_gate_out_) {requestReturnPath();}
          return;
        }
        prepared_return_path_ = path;
        if (channel_gate_out_ && state_ == State::CHANNEL_FOLLOW) {
          auto ready = std::move(*prepared_return_path_);
          prepared_return_path_.reset();
          startReturnPath(ready, arbiter_->motionActive());
        }
      }))
  {
    return_plan_pending_ = false;
    publishAudit("return_preplan_arbiter_refused", tag);
  }
}

void CompetitionSupervisor::startReturnPath(
  const nav_msgs::msg::Path & path, bool replace_active)
{
  const std::string tag = "return_P";
  publishChannelSpeedLimit(0.50);
  return_handoff_speed_limited_ = true;
  return_handoff_start_pose_ = currentPose();
  return_action_succeeded_ = false;
  enter(State::RETURN_FOLLOW, replace_active ?
    "return_to_P_hot_follow" : "return_to_P_follow");
  if (!competition_display_started_) {
    competition_display_started_ = true;
    publishCompetitionDisplay(vlm_received_ ? vlm_response_ : "识别中...");
  }
  auto on_result =
    [this, path, replace_active](std::uint64_t, rclcpp_action::ResultCode code,
      const std::string & action_detail) {
      if (state_ != State::RETURN_FOLLOW) {return;}
      publishAudit("return_action_terminal", action_detail);
      if (replace_active &&
        action_detail.find("old_goal_completed_before_accept") != std::string::npos)
      {
        publishAudit(
          "return_hot_replace_race_fallback",
          "Channel ended before replacement acceptance; sending prepared Return directly");
        startReturnPath(path, false);
        return;
      }
      if (code == rclcpp_action::ResultCode::SUCCEEDED) {
        if (!vlm_received_) {
          publishCompetitionDisplay("未识别");
          publishAudit(
            "subtask2_failed_vlm_missing",
            "judge_penalty=50s; continue independent subtask3/P verification");
        }
        return_action_succeeded_ = true;
        stopped_samples_ = 0;
      } else if (action_detail.find("action_hard_timeout") != std::string::npos) {
        return_action_succeeded_ = true;
        stopped_samples_ = 0;
        publishAudit("return_action_timeout_deferred_to_stationary_gate", action_detail);
      } else {
        enter(State::COMPLETE, "mission_complete_with_return_P_action_penalty");
      }
    };
  const bool sent = replace_active ?
    arbiter_->replaceFollowPath(
      path, "FollowPath", "ReturnPGoalChecker", tag, std::move(on_result)) :
    arbiter_->sendFollowPath(
      path, "FollowPath", "ReturnPGoalChecker", tag, std::move(on_result));
  if (!sent) {
    publishAudit("return_arbiter_refused", tag);
    enter(State::COMPLETE, "mission_complete_with_return_P_arbiter_penalty");
  }
}

void CompetitionSupervisor::updateReturnHandoffSpeedLimit()
{
  if (!return_handoff_speed_limited_) {return;}
  const Pose2D pose = currentPose();
  const double travelled = distance(
    pose.x, pose.y, return_handoff_start_pose_.x, return_handoff_start_pose_.y);
  if (travelled < 0.75) {return;}
  return_handoff_speed_limited_ = false;
  publishChannelSpeedLimit(0.0);
  publishAudit(
    "return_handoff_speed_released",
    "travelled_m=" + std::to_string(travelled) + ",race_profile_restored=true");
}

double CompetitionSupervisor::footprintFractionInParkingRegion(const Pose2D & pose) const
{
  // Deterministic 1 cm area sampling of the padded footprint against the
  // measured outer-white-frame P rectangle. The state machine applies the
  // configured minimum fraction (currently one half).
  const double front = gate_vehicle_geometry_.footprint_front_m +
    gate_vehicle_geometry_.footprint_padding_m;
  const double rear = gate_vehicle_geometry_.footprint_rear_m +
    gate_vehicle_geometry_.footprint_padding_m;
  const double half_width = gate_vehicle_geometry_.footprint_half_width_m +
    gate_vehicle_geometry_.footprint_padding_m;
  constexpr double step = 0.01;
  std::size_t total = 0U;
  std::size_t inside = 0U;
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  for (double x = -rear; x <= front + 1.0e-9; x += step) {
    for (double y = -half_width; y <= half_width + 1.0e-9; y += step) {
      const double mx = pose.x + c * x - s * y;
      const double my = pose.y + s * x + c * y;
      ++total;
      if (mx >= parking_x_min_ && mx <= parking_x_max_ &&
        my >= parking_y_min_ && my <= parking_y_max_)
      {
        ++inside;
      }
    }
  }
  return total == 0U ? 0.0 : static_cast<double>(inside) / static_cast<double>(total);
}

double CompetitionSupervisor::yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion CompetitionSupervisor::quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q; q.z = std::sin(yaw / 2.0); q.w = std::cos(yaw / 2.0); return q;
}

double CompetitionSupervisor::normalize(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

const char * CompetitionSupervisor::stateName(State state)
{
  switch (state) {
    case State::WAIT_READY: return "WAIT_READY"; case State::QR_SELECT: return "QR_SELECT";
    case State::QR_PLAN: return "QR_PLAN"; case State::QR_FOLLOW: return "QR_FOLLOW";
    case State::QR_WAIT_CODE: return "QR_WAIT_CODE";
    case State::QR_FAILED: return "QR_FAILED";
    case State::CANCEL_FOR_QR: return "CANCEL_FOR_QR";
    case State::DIRECT_GATE_QUERY: return "DIRECT_GATE_QUERY";
    case State::FORWARD_GATE_PLAN: return "FORWARD_GATE_PLAN";
    case State::DIRECT_GATE_FOLLOW: return "DIRECT_GATE_FOLLOW";
    case State::REVERSE_SELECT: return "REVERSE_SELECT";
    case State::REVERSE_FOLLOW: return "REVERSE_FOLLOW";
    case State::WAIT_STOP_AFTER_REVERSE: return "WAIT_STOP_AFTER_REVERSE";
    case State::GATE_PLAN: return "GATE_PLAN"; case State::GATE_FOLLOW: return "GATE_FOLLOW";
    case State::CHANNEL_FOLLOW: return "CHANNEL_FOLLOW"; case State::RETURN_PLAN: return "RETURN_PLAN";
    case State::RETURN_FOLLOW: return "RETURN_FOLLOW"; case State::COMPLETE: return "COMPLETE";
  }
  return "UNKNOWN";
}

}  // namespace hobot_nav
