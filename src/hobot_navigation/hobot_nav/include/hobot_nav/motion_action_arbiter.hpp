#ifndef HOBOT_NAV__MOTION_ACTION_ARBITER_HPP_
#define HOBOT_NAV__MOTION_ACTION_ARBITER_HPP_

#include <cstdint>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>

#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

namespace hobot_nav
{

// The only class in the competition process allowed to call async_send_goal.
// Every callback carries the generation that created it; callbacks belonging
// to a retired goal are ignored.  Cancellation never blocks a ROS callback.
class MotionActionArbiter
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using ComputePath = nav2_msgs::action::ComputePathToPose;
  using MotionResult = std::function<void(
      std::uint64_t, rclcpp_action::ResultCode, const std::string &)>;
  using PlanResult = std::function<void(
      std::uint64_t, bool, const nav_msgs::msg::Path &, const std::string &)>;

  explicit MotionActionArbiter(rclcpp::Node * node);

  bool serversReady() const;
  bool motionActive() const;
  bool planActive() const;
  // True only after the previous action has reached a terminal callback and
  // neither motion nor planning owns the Nav2 clients.  Channel lane changes
  // use this as the send-side gate after an asynchronous expected cancel.
  bool controlsReleased() const;
  std::uint64_t generation() const;

  bool sendFollowPath(
    const nav_msgs::msg::Path & path, const std::string & controller_id,
    const std::string & goal_checker_id, const std::string & tag,
    MotionResult callback);
  bool computePath(
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & planner_id, const std::string & tag,
    PlanResult callback);
  // Bounded preflight planning from a predicted state.  Nav2 receives the
  // supplied pose as ComputePathToPose.start (use_start=true), so callers can
  // validate a candidate reverse endpoint without moving the vehicle.
  bool computePath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & planner_id, const std::string & tag,
    PlanResult callback);
  // Motion-preserving planning. It is allowed while a FollowPath remains
  // active; ordinary mission planning keeps the motion/plan mutex below.
  bool computePathWhileFollowing(
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & planner_id, const std::string & tag,
    PlanResult callback);
  // Motion-preserving path replacement. The old FollowPath is deliberately
  // kept authoritative until Nav2 accepts the replacement goal.
  bool replaceFollowPath(
    const nav_msgs::msg::Path & path, const std::string & controller_id,
    const std::string & goal_checker_id, const std::string & tag,
    MotionResult callback);
  // Invalidate and cancel the active QR motion immediately. QR planning and
  // FollowPath callbacks created before this call are stale and must not
  // regain ownership. The physical cancel remains asynchronous so callers
  // can prepare Direct/Reverse work in parallel.
  void cancelQrFollowPathAsync(const std::string & reason);
  void cancelAllAsync(const std::string & reason);

private:
  using FollowHandle = rclcpp_action::ClientGoalHandle<FollowPath>;
  using PlanHandle = rclcpp_action::ClientGoalHandle<ComputePath>;

  bool computePathImpl(
    const geometry_msgs::msg::PoseStamped * start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & planner_id, const std::string & tag,
    PlanResult callback);
  bool computeQrPathImpl(
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & planner_id, const std::string & tag,
    PlanResult callback);
  void onDeadlineTick();
  void publishAudit(
    const std::string & event, const std::string & detail,
    std::uint64_t generation) const;
  static double pathLength(const nav_msgs::msg::Path & path);

  rclcpp::Node * node_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;
  rclcpp_action::Client<ComputePath>::SharedPtr plan_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_audit_pub_;
  mutable std::mutex mutex_;
  std::uint64_t generation_{0};
  bool motion_active_{false};
  bool plan_active_{false};
  bool terminal_release_confirmed_{true};
  bool motion_cancel_pending_{false};
  bool plan_cancel_pending_{false};
  bool qr_plan_active_{false};
  bool qr_plan_cancel_pending_{false};
  bool quiet_mode_{false};
  bool qr_plan_goal_response_pending_{false};
  bool qr_plan_warning_emitted_{false};
  std::uint64_t qr_plan_generation_{0};
  std::string motion_tag_;
  std::string plan_tag_;
  std::string qr_plan_tag_;
  FollowHandle::SharedPtr follow_handle_;
  PlanHandle::SharedPtr plan_handle_;
  PlanHandle::SharedPtr qr_plan_handle_;
  MotionResult motion_callback_;
  PlanResult plan_callback_;
  PlanResult qr_plan_callback_;
  rclcpp::TimerBase::SharedPtr deadline_timer_;
  std::chrono::steady_clock::time_point active_started_{};
  std::chrono::steady_clock::time_point goal_response_deadline_{};
  std::chrono::steady_clock::time_point action_deadline_{};
  std::chrono::steady_clock::time_point cancel_deadline_{};
  std::chrono::steady_clock::time_point qr_plan_started_{};
  std::chrono::steady_clock::time_point qr_plan_goal_response_deadline_{};
  std::chrono::steady_clock::time_point qr_plan_deadline_{};
  bool goal_response_pending_{false};
  bool compute_warning_emitted_{false};
  double goal_response_hard_sec_{1.0};
  double compute_warning_sec_{1.0};
  double compute_hard_sec_{3.0};
  double cancel_terminal_hard_sec_{1.5};
  double forward_speed_limit_mps_{0.50};
  double reverse_speed_limit_mps_{0.35};

  // A replacement request has its own goal-response callbacks. Until the
  // new goal is accepted, the old motion fields above remain untouched.
  bool replacement_pending_{false};
  std::uint64_t replacement_generation_{0};
  std::uint64_t replacement_old_generation_{0};
  FollowHandle::SharedPtr replacement_old_handle_;
  MotionResult replacement_callback_;
  std::string replacement_tag_;
};

}  // namespace hobot_nav

#endif  // HOBOT_NAV__MOTION_ACTION_ARBITER_HPP_
