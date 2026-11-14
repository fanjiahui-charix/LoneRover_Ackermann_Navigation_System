#ifndef HOBOT_NAV__COMPETITION_SUPERVISOR_HPP_
#define HOBOT_NAV__COMPETITION_SUPERVISOR_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hbmem_wechatcode_msgs/msg/qr_code.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "ollama_msgs/msg/ollama_response.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "hobot_nav/motion_action_arbiter.hpp"
#include "hobot_nav/gate_region_reachability.hpp"
#include "hobot_nav/qr_lane_reference.hpp"

namespace hobot_nav
{

class CompetitionSupervisor : public rclcpp::Node
{
public:
  CompetitionSupervisor();
  ~CompetitionSupervisor() override;

private:
  enum class State
  {
    WAIT_READY, QR_SELECT, QR_PLAN, QR_FOLLOW, QR_WAIT_CODE, QR_FAILED,
    CANCEL_FOR_QR, DIRECT_GATE_QUERY, FORWARD_GATE_PLAN, DIRECT_GATE_FOLLOW,
    REVERSE_SELECT,
    REVERSE_FOLLOW,
    WAIT_STOP_AFTER_REVERSE, GATE_PLAN, GATE_FOLLOW, CHANNEL_FOLLOW,
    RETURN_PLAN, RETURN_FOLLOW, COMPLETE
  };

  enum class DirectGateDecision {DIRECT, REVERSE, INVALID};
  enum class QrLaneBiasState {UNINITIALIZED, NEUTRAL, ACTIVE};
  enum class ChannelLane {INNER, CENTER, OUTER};
  enum class ChannelActionKind
  {
    ENTRY_CENTER, CENTER_CRUISE, SWITCH_TO_INNER, INNER_BYPASS,
    RETURN_FROM_INNER, SWITCH_TO_OUTER, OUTER_BYPASS,
    RETURN_FROM_OUTER, EXIT_CENTER
  };

  struct Pose2D {double x{0.0}; double y{0.0}; double yaw{0.0};};
  struct GridFailureDiagnostic
  {
    bool valid{false};
    std::string source;
    std::string frame;
    std::size_t segment_index{0U};
    std::size_t segment_count{0U};
    int sample_index{0};
    int sample_count{0};
    Pose2D base_pose_map{};
    Pose2D checked_pose{};
    double footprint_sample_x_m{0.0};
    double footprint_sample_y_m{0.0};
    double grid_frame_x_m{0.0};
    double grid_frame_y_m{0.0};
    double world_x_m{0.0};
    double world_y_m{0.0};
    int grid_x{-1};
    int grid_y{-1};
    int cost{-1};
    int peer_global_cost{-1};
    int peer_local_cost{-1};
    std::string reason;
  };
  struct DirectFeasibilityResult
  {
    DirectGateDecision decision{DirectGateDecision::INVALID};
    std::size_t ix{0};
    std::size_t iy{0};
    std::size_t iyaw{0};
    std::string region_id;
    std::string reason;
  };
  struct QrObservationCandidate
  {
    std::string id;
    std::string region;
    Pose2D pose;
  };
  struct QrRankResult
  {
    std::vector<QrObservationCandidate> candidates;
    std::size_t light_candidate_checks{0};
    double selection_total_ms{0.0};
    std::string failure_reason;
  };
  struct LifecycleProbe
  {
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future;
    bool active{false};
  };
  struct QrAudit
  {
    rclcpp::Time source_stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time receipt_stamp{0, 0, RCL_ROS_TIME};
    Pose2D pose;
    bool pose_valid{false};
    double linear_speed{0.0};
    double angular_speed{0.0};
    float cx{0.0F}; float cy{0.0F}; float width{0.0F}; float height{0.0F};
    std::uint64_t generation{0};
    std::string raw;
    int number{0};
  };
  struct ReverseGateMatch
  {
    std::size_t linear_index{0U};
    std::uint8_t goal_id{255U};
    Pose2D gate_goal;
    double xy_error_m{0.0};
    double yaw_error_rad{0.0};
  };
  struct ReverseCandidate
  {
    nav_msgs::msg::Path command_path;
    nav_msgs::msg::Path safety_path;
    Pose2D command_endpoint;
    Pose2D worst_case_stop;
    // Kept as a compatibility alias for existing audit consumers.  It is the
    // worst-case stopping pose, not the FollowPath command endpoint.
    Pose2D predicted_stop;
    double predicted_stop_steering{0.0};
    std::string family;
    std::string family_class;
    double command_distance{0.0};
    double command_endpoint_clearance{0.0};
    double worst_case_stop_clearance{0.0};
    // Compatibility alias for the old endpoint-clearance audit field.
    double endpoint_clearance{0.0};
    double score{0.0};
    std::optional<ReverseGateMatch> lut_match;
    // Preserve the exact RGEG2 witness selected by the clean LUT. Reverse and
    // Gate do not reinterpret this id as a Channel Tube index.
    std::uint8_t goal_id{255U};
    Pose2D gate_goal;
  };
  struct ReverseSolveResult
  {
    std::vector<ReverseCandidate> candidates;
    std::size_t state_propagations{0};
    std::size_t light_candidates{0};
    std::size_t paths_built{0};
    std::size_t strict_passed{0};
    std::size_t hard_safe_endpoints{0};
    std::size_t lut_range_misses{0};
    std::size_t lut_boundary_hits{0};
    std::size_t lut_interior_hits{0};
    double state_search_ms{0.0};
    double light_filter_ms{0.0};
    double path_build_ms{0.0};
    double strict_audit_ms{0.0};
    std::string steering_source;
    double steering_estimate_rad{0.0};
    double steering_uncertainty_rad{0.0};
    double last_valid_command_age_sec{0.0};
    std::vector<double> initial_steering_levels;
    double total_solver_ms{0.0};
    std::string failure_reason;
    // Reverse candidate rejection is intentionally exposed as structured
    // diagnostics.  Previously every null candidate was collapsed into
    // "all branches blocked", which could not distinguish a real footprint
    // collision from a braking-reserve or LUT handoff rejection.
    std::map<std::string, std::size_t> rejection_counts;
    std::string first_rejection_reason;
  };
  struct ChannelTube
  {
    nav_msgs::msg::Path path;
    std::vector<double> s_m;
    std::vector<double> kappa_1pm;
    std::string id;
    ChannelLane lane{ChannelLane::CENTER};
    bool clockwise{true};
    double loop_length_m{0.0};
  };
  struct ChannelProjection
  {
    std::size_t index{0U};
    double s_mod_m{0.0};
    double distance_m{std::numeric_limits<double>::infinity()};
  };
  struct ChannelTransitionPoint
  {
    double x_local_m{0.0};
    double y_local_m{0.0};
    double yaw_local_rad{0.0};
  };
  struct ChannelConnector
  {
    nav_msgs::msg::Path path;
    std::string id;
    ChannelLane source_lane{ChannelLane::CENTER};
    ChannelLane target_lane{ChannelLane::CENTER};
    bool clockwise{true};
    std::size_t source_index{0U};
    double source_s_m{0.0};
  };
  void onTick();
  void refreshMapPose();
  void onOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void onCostmap(nav2_msgs::msg::Costmap::ConstSharedPtr msg);
  void onLocalCostmap(nav2_msgs::msg::Costmap::ConstSharedPtr msg);
  void onStaticMap(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  void onStartSignal(std_msgs::msg::String::ConstSharedPtr msg);
  void onQr(hbmem_wechatcode_msgs::msg::QrCode::ConstSharedPtr msg);
  void onVlm(ollama_msgs::msg::OllamaResponse::ConstSharedPtr msg);
  void onBevAccepted(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void onBevCorrectionEvent(std_msgs::msg::String::ConstSharedPtr msg);
  void onSteeringAngle(std_msgs::msg::Float32::ConstSharedPtr msg);
  void onSafeCommand(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void onCones(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void maybeBeginQrHandoff();

  bool loadChannelTubeLibrary(const std::string & directory);
  bool loadChannelConeSideLut(const std::string & path);
  bool loadChannelV2Assets(
    const std::string & directory, const std::string & admission_sha256);
  bool loadChannelTransitionTemplate(
    const std::string & path, const std::string & expected_sha256,
    std::vector<ChannelTransitionPoint> & points);
  bool loadChannelBranch(const std::string & path, nav_msgs::msg::Path & branch);
  bool ready(std::string & reason);
  bool lifecycleActive(
    LifecycleProbe & probe, const std::string & node_name,
    std::string & reason);
  void enter(State state, const std::string & event);
  void publishGates();
  void publishStartupReady(bool ready);
  void publishCompetitionDisplay(const std::string & vlm_result);
  bool loadQrLaneBiasAsset(const std::string & path, std::string & reason);
  void setQrLaneBiasState(QrLaneBiasState desired);
  void publishEvent(const std::string & event);
  void publishAudit(const std::string & event, const std::string & detail);
  void enqueueWorker(std::function<std::function<void()>()> compute);
  void drainWorkerCompletion();
  void invalidateWorker();
  void workerLoop();

  Pose2D currentPose() const;
  geometry_msgs::msg::PoseStamped poseStamped(const Pose2D & pose) const;
  // Formal P->QR entry uses two fixed map goals.  Their yaw is derived from
  // the calibrated base->camera TF and the board pose at runtime; the target
  // coordinates are fixed so QR execution cannot jump between observation
  // regions while cones are being updated.
  QrRankResult makeFixedQrGoals();
  bool validateNav2PathLight(
    const nav_msgs::msg::Path & path, const Pose2D & expected_goal,
    std::string & reason, double yaw_tolerance_rad = 0.35,
    bool require_current_costmap = true) const;
  bool validateChannelPathLight(
    const nav_msgs::msg::Path & path, std::string & reason) const;
  bool footprintFreeInGrid(
    const Pose2D & pose, const nav2_msgs::msg::Costmap & grid,
    GridFailureDiagnostic * diagnostic = nullptr) const;
  bool qrGoalFootprintLegal(
    const QrObservationCandidate & candidate, std::string & reason) const;
  bool qrGoalFootprintLegal(
    const QrObservationCandidate & candidate,
    const nav2_msgs::msg::Costmap & grid, std::string & reason) const;
  double endpointClearanceInGrid(
    const Pose2D & pose, const nav2_msgs::msg::Costmap & grid) const;
  bool auditPathAgainstGrid(
    const nav_msgs::msg::Path & path, bool reverse,
    const nav2_msgs::msg::Costmap & grid, std::string & reason,
    bool enforce_curvature = true, std::string * curvature_diagnostic = nullptr,
    GridFailureDiagnostic * diagnostic = nullptr) const;
  bool auditReverseSegmentHardSafety(
    const Pose2D & previous, const Pose2D & current,
    const GateSearchSnapshot & snapshot, std::string & reason,
    GridFailureDiagnostic * diagnostic = nullptr) const;
  void requestNextQrPlan();
  void retryQrSelectionWithoutHold(const std::string & reason);
  void startQrPath(
    const nav_msgs::msg::Path & path, const std::string & seed_id,
    const std::string & region, bool reset_midcourse = true);
  void maybeStartQrMidcourseReplan();
  void requestQrMidcourseReplan(const QrObservationCandidate & seed);
  void replaceQrPathMidcourse(
    const nav_msgs::msg::Path & path, const QrObservationCandidate & seed);
  void recoverQrFollowFailure(
    const std::string & failed_seed_id, const std::string & detail);
  bool loadDirectGateRobustMask(const std::string & path, std::string & reason);
  DirectFeasibilityResult directForwardFeasible(const Pose2D & pose) const;
  bool loadReverseGateEntryMask(const std::string & path, std::string & reason);
  bool loadReverseGateGoalIds(const std::string & path, std::string & reason);
  bool reverseGateEntryMaskFeasible(
    const Pose2D & pose, std::size_t * linear_index = nullptr) const;
  std::optional<Pose2D> reverseGateEntryGoal(const Pose2D & pose) const;
  std::optional<ReverseGateMatch> reverseGateEntryMatch(
    const Pose2D & pose) const;
  // Static Gate-entry feasibility lookup used only while selecting a Reverse
  // endpoint. It keeps start yaw in the lookup but does not impose a Gate
  // terminal-yaw condition.
  bool gateEntryFeasible(const Pose2D & pose) const;
  void requestDirectGateQuery();
  void failDirectGateToReverse(const std::string & reason);
  std::vector<Pose2D> gateCandidates() const;
  void requestNextGatePlan(bool post_reverse);
  void startGatePath(
    const nav_msgs::msg::Path & path, bool post_reverse,
    std::size_t candidate_index);
  bool gateHandoffRegionReached(const Pose2D & pose, std::string & detail) const;

  ReverseSolveResult makeReverseCandidatesLayered(const Pose2D & frozen_qr_pose) const;
  std::optional<ReverseCandidate> rebuildReverseCandidateFromPose(
    const ReverseCandidate & base, const Pose2D & start, double distance,
    const GateSearchSnapshot & snapshot, std::string & reason,
    GridFailureDiagnostic * diagnostic = nullptr) const;
  void publishReverseSolveMetrics(const ReverseSolveResult & result);
  void startSelectedReverse();
  const ChannelTube * channelTube(ChannelLane lane) const;
  ChannelProjection projectToChannelTube(
    const ChannelTube & tube, const Pose2D & pose) const;
  ChannelProjection projectToOpenPath(
    const nav_msgs::msg::Path & path, const Pose2D & pose) const;
  double centerForwardDistance(double from_s, double to_s) const;
  nav_msgs::msg::Path cyclicChannelSuffix(
    const ChannelTube & tube, std::size_t start_index,
    std::size_t stop_index, bool require_wrap) const;
  nav_msgs::msg::Path centerPathToExit(std::size_t start_index) const;
  bool channelStraightTransitionEligible(
    const ChannelTube & tube, std::size_t start_index,
    std::string & reason) const;
  bool buildChannelTransition(
    ChannelLane target_lane, nav_msgs::msg::Path & path,
    ChannelActionKind & kind, std::string & reason);
  bool buildChannelTransitionV2(
    ChannelLane target_lane, nav_msgs::msg::Path & path,
    ChannelActionKind & kind, std::string & reason);
  std::optional<std::pair<double, ChannelLane>> nearestForwardConeTrigger();
  std::uint8_t coneSideClass(double x, double y) const;
  void requestChannelEntry(bool fallback);
  void beginChannelAtEntry(const nav_msgs::msg::Path & connector, bool fallback);
  void startChannelEntryConnector(const nav_msgs::msg::Path & connector);
  void startCenterTubeAfterEntry(bool replace_active);
  bool requestChannelPathReplacement(
    const nav_msgs::msg::Path & path, ChannelLane target_lane,
    ChannelActionKind kind, const std::string & reason);
  bool resumeChannelActivePath(const std::string & terminal_detail);
  bool continueActiveSideTube(const std::string & terminal_detail);
  void maybeHandleChannelCone();
  bool returnHandoffProgressReached(
    const Pose2D & pose, ChannelProjection * projection = nullptr,
    double * anchor_station_m = nullptr) const;
  void publishChannelSpeedLimit(double limit_mps);
  void updateChannelProgress();
  void startChannelPath(
    const nav_msgs::msg::Path & path, ChannelLane lane,
    ChannelActionKind kind, bool replace_active = false);
  void requestGatePath();
  void requestPostReverseForwardPath();
  void startPostGateFlow();
  void abandonChannelToReturn(const std::string & reason);
  // The bool is retained only for source/API compatibility with old logs;
  // Return now always uses the single measured P target.
  void requestReturnPath(bool backup = false);
  void prepareReturnPathWhileFollowing();
  void startReturnPath(const nav_msgs::msg::Path & path, bool replace_active);
  void updateReturnHandoffSpeedLimit();
  double footprintFractionInParkingRegion(const Pose2D & pose) const;
  bool stationaryReady() const;
  std::optional<std::pair<double, double>> steeringEstimateLocked(
    const rclcpp::Time & stamp, std::string & source) const;

  static double yawOf(const geometry_msgs::msg::Quaternion & q);
  static geometry_msgs::msg::Quaternion quaternion(double yaw);
  static double normalize(double angle);
  static const char * stateName(State state);

  std::unique_ptr<MotionActionArbiter> arbiter_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_sub_;
  rclcpp::Subscription<hbmem_wechatcode_msgs::msg::QrCode>::SharedPtr qr_sub_;
  rclcpp::Subscription<ollama_msgs::msg::OllamaResponse>::SharedPtr vlm_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr bev_accepted_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr bev_correction_event_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_angle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cones_sub_;
  LifecycleProbe planner_state_probe_;
  LifecycleProbe controller_state_probe_;
  LifecycleProbe smoother_state_probe_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bev_compute_gate_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bev_gate_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vlm_gate_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vlm_detector_gate_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reverse_gate_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr direct_gate_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vlm_trigger_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr display_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr direction_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr audit_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reverse_diag_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr perf_trace_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr startup_ready_pub_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr channel_speed_limit_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr qr_lane_bias_pub_;

  mutable std::mutex snapshot_mutex_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_;
  nav2_msgs::msg::Costmap::ConstSharedPtr latest_costmap_;
  nav2_msgs::msg::Costmap::ConstSharedPtr latest_local_costmap_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_static_map_;
  rclcpp::Time latest_costmap_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_local_costmap_receipt_{0, 0, RCL_ROS_TIME};
  // Humble /costmap_raw messages leave metadata.update_time at zero.  Use
  // local monotonic receipt time for freshness decisions instead.
  std::chrono::steady_clock::time_point latest_costmap_steady_receipt_{};
  std::chrono::steady_clock::time_point latest_local_costmap_steady_receipt_{};
  bool latest_costmap_geometry_valid_{false};
  bool latest_costmap_payload_valid_{false};
  bool latest_costmap_all_unknown_{true};
  bool latest_local_costmap_geometry_valid_{false};
  bool latest_local_costmap_payload_valid_{false};
  bool latest_local_costmap_all_unknown_{true};
  std::chrono::steady_clock::time_point last_global_snapshot_diag_{};
  std::uint64_t global_costmap_receive_sequence_{0};
  std::uint64_t local_costmap_receive_sequence_{0};
  std::uint64_t qr_lane_bias_publish_sequence_{0};
  Pose2D latest_map_pose_;
  rclcpp::Time latest_map_pose_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_map_tf_probe_{0, 0, RCL_ROS_TIME};
  std::uint64_t map_pose_epoch_{0};
  bool have_map_pose_{false};
  std::optional<double> steering_angle_rad_;
  rclcpp::Time steering_angle_stamp_{0, 0, RCL_ROS_TIME};
  std::uint64_t steering_epoch_{0};
  geometry_msgs::msg::Twist latest_safe_cmd_;
  rclcpp::Time latest_safe_cmd_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_cones_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_cones_source_stamp_{0, 0, RCL_ROS_TIME};
  std::vector<std::pair<double, double>> latest_cone_centers_map_;
  bool latest_cones_message_valid_{false};
  std::uint64_t latest_cones_receive_sequence_{0};
  double observer_steering_rad_{0.0};
  double observer_uncertainty_rad_{0.35};
  rclcpp::Time observer_stamp_{0, 0, RCL_ROS_TIME};
  bool have_last_valid_command_steering_{false};
  double last_valid_command_steering_rad_{0.0};
  double last_valid_command_uncertainty_rad_{0.35};
  rclcpp::Time last_valid_command_stamp_{0, 0, RCL_ROS_TIME};
  GateTransform2D latest_map_to_local_;
  rclcpp::Time latest_map_to_local_stamp_{0, 0, RCL_ROS_TIME};
  std::atomic<bool> qr_latched_{false};
  // QR is latched independently of motion readiness.  Readiness only controls
  // when the saved result may start the Direct/Reverse handoff.
  bool motion_stack_ready_{false};
  bool startup_ready_{false};
  bool qr_handoff_started_{false};
  std::shared_ptr<const QrAudit> qr_audit_;
  // Immutable handoff anchor captured at the instant the legal QR result is
  // accepted.  Reverse solving never re-anchors to a later map-pose epoch.
  Pose2D qr_frozen_pose_{};
  bool qr_frozen_pose_valid_{false};

  State state_{State::WAIT_READY};
  rclcpp::Time state_entered_;
  std::vector<QrObservationCandidate> qr_ranked_;
  bool qr_fixed_goals_built_{false};
  std::size_t qr_candidate_index_{0};
  int qr_plan_attempts_{0};
  int qr_motion_actions_{0};
  double qr_stage_timeout_sec_{30.0};
  rclcpp::Time qr_stage_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time qr_retry_not_before_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Path active_qr_path_;
  std::string active_qr_goal_id_;
  std::uint64_t active_qr_path_revision_{0};
  bool qr_midcourse_triggered_{false};
  bool qr_midcourse_plan_sent_{false};
  int last_bev_compute_gate_{-1};
  int last_bev_gate_{-1};
  int last_vlm_gate_{-1};
  int last_vlm_detector_gate_{-1};
  int last_reverse_gate_{-1};
  int last_direct_gate_{-1};
  bool direct_gate_query_attempted_{false};
  // Keep the calibrated Direct-Gate LUT as the default decision source, but
  // allow a controlled vehicle A/B run to bypass it and enter the existing
  // Reverse search directly.  This is intentionally scoped to QR exit only.
  Pose2D direct_gate_qr_pose_;
  std::vector<std::uint8_t> direct_gate_robust_mask_;
  std::string direct_gate_robust_mask_path_;
  std::string direct_gate_robust_mask_sha256_;
  std::size_t direct_gate_robust_mask_file_size_{0};
  std::string direct_gate_robust_mask_error_;
  std::vector<std::uint8_t> reverse_gate_entry_mask_;
  std::vector<std::uint8_t> reverse_gate_goal_ids_;
  std::vector<Pose2D> reverse_gate_handoff_goals_;
  std::string reverse_gate_entry_mask_path_;
  std::string reverse_gate_goal_ids_path_;
  std::string reverse_gate_entry_mask_sha256_;
  std::string reverse_gate_goal_ids_sha256_;
  std::string reverse_gate_entry_mask_error_;
  std::string reverse_gate_goal_ids_error_;
  std::size_t reverse_gate_entry_mask_file_size_{0};
  std::size_t reverse_gate_goal_ids_file_size_{0};
  std::size_t reverse_gate_entry_size_x_{0};
  std::size_t reverse_gate_entry_size_y_{0};
  std::size_t reverse_gate_entry_yaw_bins_{0};
  double reverse_gate_entry_x_min_m_{0.0};
  double reverse_gate_entry_y_min_m_{0.0};
  double reverse_gate_entry_xy_resolution_m_{0.0};
  int reverse_gate_entry_yaw_min_deg_{0};
  int reverse_gate_entry_yaw_step_deg_{0};
  std::optional<Pose2D> post_reverse_handoff_goal_;
  bool gate_handoff_cancel_pending_{false};
  std::size_t gate_candidate_index_{0};
  std::string reverse_trigger_reason_;
  int stopped_samples_{0};
  rclcpp::Time stationary_started_{0, 0, RCL_ROS_TIME};
  std::vector<ReverseCandidate> reverse_candidates_;
  std::optional<ReverseCandidate> nominal_reverse_;
  std::size_t reverse_candidate_index_{0U};
  std::optional<double> reverse_retry_remaining_distance_m_;
  double reverse_execution_distance_m_{0.0};
  int reverse_actions_{0};
  int reverse_recomputations_{0};
  bool reverse_execution_failed_{false};
  std::optional<ReverseCandidate> selected_reverse_;
  std::vector<ChannelTube> channel_tube_library_;
  std::vector<ChannelTransitionPoint> channel_transition_bottom_plus_;
  std::vector<ChannelTransitionPoint> channel_transition_bottom_minus_;
  std::vector<ChannelTransitionPoint> channel_transition_top_plus_;
  std::vector<ChannelTransitionPoint> channel_transition_top_minus_;
  nav_msgs::msg::Path channel_exit_cw_path_;
  nav_msgs::msg::Path channel_exit_ccw_path_;
  std::vector<std::uint8_t> channel_cone_side_lut_;
  std::vector<std::uint8_t> channel_avoid_lut_cw_;
  std::vector<std::uint8_t> channel_avoid_lut_ccw_;
  std::vector<ChannelConnector> channel_v2_connectors_;
  std::size_t active_channel_connector_points_{0U};
  nav_msgs::msg::Path channel_template_;
  std::shared_ptr<const nav_msgs::msg::Path> active_channel_path_;
  std::size_t channel_progress_{0};
  ChannelLane active_channel_lane_{ChannelLane::CENTER};
  ChannelActionKind active_channel_action_{ChannelActionKind::ENTRY_CENTER};
  bool channel_entry_connector_active_{false};
  double channel_entry_path_progress_m_{0.0};
  double channel_entry_handoff_path_s_m_{0.0};
  bool direct_entry_attempt_active_{false};
  bool return_plan_pending_{false};
  bool return_preplan_attempted_{false};
  std::uint32_t return_plan_attempts_{0U};
  std::optional<nav_msgs::msg::Path> prepared_return_path_;
  bool return_handoff_speed_limited_{false};
  Pose2D return_handoff_start_pose_{};
  std::optional<ChannelLane> deferred_channel_lane_;
  std::optional<double> active_cone_trigger_unwrapped_s_;
  std::optional<double> last_channel_speed_limit_mps_;
  double last_handled_cone_unwrapped_s_{-
    std::numeric_limits<double>::infinity()};
  double channel_center_s_mod_m_{0.0};
  double channel_center_unwrapped_s_m_{0.0};
  double channel_center_last_s_mod_m_{0.0};
  double channel_exit_unwrapped_s_m_{0.0};
  double channel_entry_s_m_{0.0};
  std::size_t channel_entry_center_index_{0U};
  Pose2D channel_entry_pose_{};
  bool channel_center_entry_reached_{false};
  std::size_t channel_exit_center_index_{0U};
  std::uint64_t channel_last_cone_sequence_{0U};
  std::uint64_t channel_lane_switch_count_{0U};
  std::uint64_t channel_failed_recovery_count_{0U};
  std::optional<Pose2D> last_channel_abort_pose_;
  rclcpp::Time last_channel_abort_stamp_{0, 0, RCL_ROS_TIME};
  std::optional<nav_msgs::msg::Path> pending_channel_retry_path_;
  ChannelLane pending_channel_retry_lane_{ChannelLane::CENTER};
  ChannelActionKind pending_channel_retry_kind_{ChannelActionKind::CENTER_CRUISE};
  bool pending_channel_retry_replace_active_{false};
  rclcpp::Time pending_channel_retry_not_before_{0, 0, RCL_ROS_TIME};
  std::atomic<bool> bev_correction_pending_{false};
  bool channel_ring_complete_{false};
  bool channel_gate_out_{false};
  bool clockwise_{true};
  bool vlm_triggered_{false};
  bool vlm_received_{false};
  bool competition_display_started_{false};
  std::string vlm_response_;
  bool vlm_cache_armed_{false};
  bool vlm_capture_window_active_{false};
  rclcpp::Time vlm_capture_window_entered_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time vlm_cache_armed_at_{0, 0, RCL_ROS_TIME};
  bool return_action_succeeded_{false};
  bool start_mission_{false};
  bool race_continue_enabled_{true};
  bool competition_quiet_mode_{false};
  bool direct_reverse_to_channel_entry_enabled_{true};
  bool channel_runtime_admitted_{false};
  bool channel_avoidance_v2_enabled_{false};
  bool channel_v2_invalid_forward_cone_{false};
  std::string channel_v2_asset_directory_;
  std::string channel_v2_admission_sha256_;
  double channel_straight_ceiling_mps_{0.60};
  double channel_corner_ceiling_mps_{0.42};
  double channel_lane_change_ceiling_mps_{0.42};
  double channel_corner_preview_distance_m_{0.45};
  double channel_corner_exit_hold_distance_m_{0.0};
  double channel_corner_curvature_threshold_1pm_{0.15};
  double channel_corner_hold_remaining_m_{0.0};
  double channel_corner_last_s_m_{0.0};
  const ChannelTube * channel_corner_last_tube_{nullptr};
  bool channel_corner_projection_valid_{false};
  double channel_cone_lookahead_m_{1.25};
  double channel_cone_passed_margin_m_{0.45};
  double channel_cone_lateral_relevance_m_{0.27};
  std::string mission_scope_{"full"};
  std::string base_frame_;
  std::string camera_frame_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string safe_cmd_topic_{"/cmd_vel_safe"};
  double min_turning_radius_{0.35};
  double yellow_x_min_{4.5};
  double yellow_x_max_{5.0};
  double yellow_y_min_{1.6};
  double yellow_y_max_{2.0};
  double parking_x_min_{0.0};
  double parking_x_max_{1.00};
  double parking_y_min_{0.0};
  double parking_y_max_{0.40};
  double parking_footprint_fraction_min_{0.50};
  double reverse_design_speed_mps_{0.35};
  double actuation_latency_sec_{0.27};
  double braking_deceleration_mps2_{0.70};
  double braking_distance_margin_m_{0.04};
  double steering_build_rate_radps_{1.20};
  double steering_return_rate_radps_{1.00};
  double steering_delay_distance_m_{0.12};
  double reverse_search_step_m_{0.10};
  double reverse_max_search_distance_m_{1.50};
  double steering_feedback_timeout_sec_{0.15};
  double reverse_speed_limit_mps_{0.35};
  double stationary_linear_threshold_mps_{0.02};
  double stationary_angular_threshold_radps_{0.05};
  double stationary_minimum_duration_sec_{0.15};
  int stationary_minimum_samples_{4};
  double stationary_maximum_odom_age_sec_{0.10};
  double observer_uncertainty_limit_rad_{3.0 * 3.14159265358979323846 / 180.0};
  double observer_uncertainty_growth_radps_{0.02};
  double snapshot_odom_warn_age_sec_{0.30};
  double snapshot_tf_warn_age_sec_{0.30};
  double snapshot_local_warn_age_sec_{0.60};
  double snapshot_global_warn_age_sec_{1.00};
  double snapshot_odom_hard_age_sec_{0.80};
  double snapshot_tf_hard_age_sec_{0.80};
  double snapshot_local_hard_age_sec_{1.50};
  double snapshot_global_hard_age_sec_{2.00};
  double qr_board_center_x_{4.75};
  double qr_board_center_y_{1.80};
  double qr_board_center_z_{0.18};
  double qr_board_normal_yaw_rad_{3.14159265358979323846};
  double qr_board_width_m_{0.210};
  double qr_board_height_m_{0.148};
  double qr_camera_fx_{682.7522450473028};
  double qr_camera_fy_{684.6825030858382};
  double qr_camera_cx_{948.0233927822394};
  double qr_camera_cy_{552.5465008451092};
  int qr_camera_image_width_{1920};
  int qr_camera_image_height_{1080};
  double qr_min_projected_width_px_{70.0};
  double qr_image_border_fraction_{0.08};
  double qr_max_slant_angle_rad_{60.0 * 3.14159265358979323846 / 180.0};
  // QR carries both the source camera-frame time and the decoder completion
  // time. Gate them independently: a fresh delivery must not be rejected
  // merely because WeChat spent time decoding the source frame.
  double qr_primary_x_{3.60};
  double qr_primary_y_{1.20};
  double qr_backup_x_{3.60};
  double qr_backup_y_{0.80};
  double qr_midcourse_replan_x_{1.60};
  double channel_heading_limit_rad_{8.0 * 3.14159265358979323846 / 180.0};
  double cones_freshness_sec_{0.50};
  QrLaneReference qr_lane_reference_;
  std::vector<std::uint8_t> qr_lane_bias_active_data_;
  nav_msgs::msg::OccupancyGrid qr_lane_bias_active_map_;
  nav_msgs::msg::OccupancyGrid qr_lane_bias_neutral_map_;
  QrLaneBiasState qr_lane_bias_state_{QrLaneBiasState::UNINITIALIZED};
  std::string qr_lane_bias_asset_path_;
  std::string qr_lane_bias_asset_sha256_;
  std::string qr_lane_bias_asset_error_;
  bool qr_lane_bias_asset_loaded_{false};
  bool qr_lane_bias_built_{false};
  bool qr_lane_bias_ready_{false};
  // Kept as an explicit admission switch so the historical lane-bias asset
  // remains available for A/B rollback without being part of the formal
  // competition QR path.
  bool qr_lane_bias_enabled_{false};
  bool diagnostics_path_geometry_enabled_{false};
  GateEntryRegion gate_region_;
  GateVehicleGeometry gate_vehicle_geometry_;

  // One bounded latest-only worker for the one-shot Reverse feasibility solve.
  std::thread worker_thread_;
  std::mutex worker_mutex_;
  std::condition_variable worker_cv_;
  bool worker_stop_{false};
  std::uint64_t worker_generation_{0};
  std::optional<std::pair<std::uint64_t,
    std::function<std::function<void()>()>>> pending_worker_job_;
  std::optional<std::pair<std::uint64_t, std::function<void()>>> worker_completion_;
  bool reverse_build_pending_{false};
};

}  // namespace hobot_nav

#endif  // HOBOT_NAV__COMPETITION_SUPERVISOR_HPP_
