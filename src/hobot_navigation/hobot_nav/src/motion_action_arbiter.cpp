#include "hobot_nav/motion_action_arbiter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <utility>

namespace hobot_nav
{
namespace
{
const char * resultCodeName(rclcpp_action::ResultCode code)
{
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED: return "SUCCEEDED";
    case rclcpp_action::ResultCode::ABORTED: return "ABORTED";
    case rclcpp_action::ResultCode::CANCELED: return "CANCELED";
    default: return "UNKNOWN";
  }
}
}  // namespace

MotionActionArbiter::MotionActionArbiter(rclcpp::Node * node)
: node_(node)
{
  (void)node_->get_parameter("competition_quiet_mode", quiet_mode_);
  follow_client_ = rclcpp_action::create_client<FollowPath>(
    node_->get_node_base_interface(), node_->get_node_graph_interface(),
    node_->get_node_logging_interface(), node_->get_node_waitables_interface(),
    "follow_path");
  plan_client_ = rclcpp_action::create_client<ComputePath>(
    node_->get_node_base_interface(), node_->get_node_graph_interface(),
    node_->get_node_logging_interface(), node_->get_node_waitables_interface(),
    "compute_path_to_pose");
  action_audit_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/race/action_audit",
    rclcpp::QoS(rclcpp::KeepLast(50)).reliable().transient_local());
  goal_response_hard_sec_ = node_->declare_parameter<double>(
    "action_deadlines.goal_response_hard_sec", 1.0);
  compute_warning_sec_ = node_->declare_parameter<double>(
    "action_deadlines.compute_path_warning_sec", 1.0);
  compute_hard_sec_ = node_->declare_parameter<double>(
    "action_deadlines.compute_path_hard_sec", 3.0);
  cancel_terminal_hard_sec_ = node_->declare_parameter<double>(
    "action_deadlines.cancel_terminal_hard_sec", 1.5);
  forward_speed_limit_mps_ = node_->declare_parameter<double>(
    "action_deadlines.forward_speed_limit_mps", 0.50);
  (void)node_->get_parameter("reverse_speed_limit", reverse_speed_limit_mps_);
  deadline_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MotionActionArbiter::onDeadlineTick, this));
}

void MotionActionArbiter::publishAudit(
  const std::string & event, const std::string & detail,
  std::uint64_t generation) const
{
  if (quiet_mode_) {return;}
  std_msgs::msg::String message;
  std::ostringstream text;
  text << "stamp=" << node_->now().seconds() << ",event=" << event
       << ",generation=" << generation << ",detail=" << detail;
  message.data = text.str();
  action_audit_pub_->publish(message);
}

bool MotionActionArbiter::serversReady() const
{
  return follow_client_->action_server_is_ready() &&
         plan_client_->action_server_is_ready();
}

bool MotionActionArbiter::motionActive() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return motion_active_;
}

bool MotionActionArbiter::planActive() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return plan_active_ || qr_plan_active_;
}

bool MotionActionArbiter::controlsReleased() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return terminal_release_confirmed_ && !motion_active_ && !plan_active_ &&
         !qr_plan_active_ && !replacement_pending_;
}

std::uint64_t MotionActionArbiter::generation() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return generation_;
}

double MotionActionArbiter::pathLength(const nav_msgs::msg::Path & path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    length += std::hypot(
      path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
      path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
  }
  return length;
}

bool MotionActionArbiter::sendFollowPath(
  const nav_msgs::msg::Path & path, const std::string & controller_id,
  const std::string & goal_checker_id, const std::string & tag,
  MotionResult callback)
{
  std::uint64_t generation;
  const auto sent_at = std::chrono::steady_clock::now();
  const bool channel = tag.find("channel") != std::string::npos;
  const bool reverse = tag.find("reverse") != std::string::npos;
  const double speed_limit = reverse ? reverse_speed_limit_mps_ :
    forward_speed_limit_mps_;
  const double expected_sec = pathLength(path) /
    std::max(0.12, 0.35 * std::max(0.01, speed_limit));
  const double hard_sec = std::clamp(
    expected_sec + 5.0, channel ? 8.0 : 5.0, channel ? 60.0 : 40.0);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (motion_active_ || plan_active_ || path.poses.size() < 2U) {
      return false;
    }
    generation = ++generation_;
    motion_active_ = true;
    terminal_release_confirmed_ = false;
    motion_cancel_pending_ = false;
    motion_tag_ = tag;
    motion_callback_ = std::move(callback);
    follow_handle_.reset();
    active_started_ = sent_at;
    goal_response_pending_ = true;
    compute_warning_emitted_ = false;
    goal_response_deadline_ = sent_at + std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(goal_response_hard_sec_));
    action_deadline_ = sent_at + std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(std::chrono::duration<double>(hard_sec));
    cancel_deadline_ = {};
  }

  FollowPath::Goal goal;
  goal.path = path;
  goal.controller_id = controller_id;
  goal.goal_checker_id = goal_checker_id;

  rclcpp_action::Client<FollowPath>::SendGoalOptions options;
  options.goal_response_callback =
    [this, generation](FollowHandle::SharedPtr handle) {
      bool cancel = false;
      MotionResult rejected_callback;
      std::string tag;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!motion_active_ || generation != generation_) {
          if (handle) {follow_client_->async_cancel_goal(handle);}
          return;
        }
        goal_response_pending_ = false;
        if (!handle) {
          motion_active_ = false;
          terminal_release_confirmed_ = true;
          rejected_callback = std::move(motion_callback_);
          tag = motion_tag_;
          motion_callback_ = nullptr;
        } else {
          follow_handle_ = handle;
          cancel = motion_cancel_pending_;
        }
      }
      if (cancel && handle) {follow_client_->async_cancel_goal(handle);}
      if (rejected_callback) {
        publishAudit("follow_goal_rejected", tag, generation);
        rejected_callback(generation, rclcpp_action::ResultCode::ABORTED,
          tag + ":goal_rejected");
      }
    };
  options.result_callback =
    [this, generation](const FollowHandle::WrappedResult & result) {
      MotionResult callback;
      MotionResult replacement_callback;
      std::string tag;
      bool replacement_aborted = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (replacement_pending_ && generation == replacement_old_generation_) {
          // The old goal completed before the replacement was accepted. Do
          // not let the later replacement response resurrect this motion.
          motion_active_ = false;
          terminal_release_confirmed_ = true;
          motion_cancel_pending_ = false;
          goal_response_pending_ = false;
          follow_handle_.reset();
          callback = std::move(motion_callback_);
          tag = motion_tag_;
          motion_callback_ = nullptr;
          replacement_callback = std::move(replacement_callback_);
          replacement_pending_ = false;
          replacement_old_handle_.reset();
          replacement_tag_.clear();
          ++generation_;
          replacement_aborted = true;
        } else if (!motion_active_ || generation != generation_) {
          return;
        }
        if (replacement_aborted) {
          // The common terminal bookkeeping below would be a second state
          // transition; the old result is already finalized above.
        } else {
          motion_active_ = false;
          terminal_release_confirmed_ = true;
          motion_cancel_pending_ = false;
          goal_response_pending_ = false;
          follow_handle_.reset();
          callback = std::move(motion_callback_);
          tag = motion_tag_;
          motion_callback_ = nullptr;
        }
      }
      std::ostringstream detail;
      detail << tag << ":result_code=" << resultCodeName(result.code)
             << ":nav2_error_code=unavailable_in_humble_follow_path_result"
             << ":result_payload=" << (result.result ? "present" : "missing");
      publishAudit("follow_result", detail.str(), generation);
      if (callback) {callback(generation, result.code, detail.str());}
      if (replacement_aborted && replacement_callback) {
        replacement_callback(
          generation, rclcpp_action::ResultCode::ABORTED,
          "follow_replacement:old_goal_completed_before_accept");
      }
    };
  follow_client_->async_send_goal(goal, options);
  publishAudit("follow_goal_sent", tag, generation);
  return true;
}

bool MotionActionArbiter::computePath(
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const std::string & planner_id, const std::string & tag,
  PlanResult callback)
{
  return computePathImpl(nullptr, goal_pose, planner_id, tag, std::move(callback));
}

bool MotionActionArbiter::computePath(
  const geometry_msgs::msg::PoseStamped & start_pose,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const std::string & planner_id, const std::string & tag,
  PlanResult callback)
{
  return computePathImpl(&start_pose, goal_pose, planner_id, tag, std::move(callback));
}

bool MotionActionArbiter::computePathWhileFollowing(
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const std::string & planner_id, const std::string & tag,
  PlanResult callback)
{
  return computeQrPathImpl(goal_pose, planner_id, tag, std::move(callback));
}

bool MotionActionArbiter::computeQrPathImpl(
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const std::string & planner_id, const std::string & tag,
  PlanResult callback)
{
  std::uint64_t generation;
  const auto sent_at = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!motion_active_ || motion_cancel_pending_ || plan_active_ ||
      qr_plan_active_)
    {
      return false;
    }
    generation = ++qr_plan_generation_;
    qr_plan_active_ = true;
    qr_plan_cancel_pending_ = false;
    qr_plan_goal_response_pending_ = true;
    qr_plan_warning_emitted_ = false;
    qr_plan_tag_ = tag;
    qr_plan_callback_ = std::move(callback);
    qr_plan_handle_.reset();
    qr_plan_started_ = sent_at;
    qr_plan_goal_response_deadline_ = sent_at +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(goal_response_hard_sec_));
    qr_plan_deadline_ = sent_at +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(compute_hard_sec_));
  }

  ComputePath::Goal goal;
  goal.goal = goal_pose;
  goal.use_start = false;
  goal.planner_id = planner_id;
  rclcpp_action::Client<ComputePath>::SendGoalOptions options;
  options.goal_response_callback =
    [this, generation](PlanHandle::SharedPtr handle) {
      bool cancel = false;
      PlanResult rejected_callback;
      std::string tag;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!qr_plan_active_ || generation != qr_plan_generation_) {
          if (handle) {plan_client_->async_cancel_goal(handle);}
          return;
        }
        qr_plan_goal_response_pending_ = false;
        if (!handle) {
          qr_plan_active_ = false;
          rejected_callback = std::move(qr_plan_callback_);
          tag = qr_plan_tag_;
          qr_plan_callback_ = nullptr;
        } else {
          qr_plan_handle_ = handle;
          cancel = qr_plan_cancel_pending_;
        }
      }
      if (cancel && handle) {plan_client_->async_cancel_goal(handle);}
      if (rejected_callback) {
        publishAudit("qr_compute_goal_rejected", tag, generation);
        rejected_callback(generation, false, nav_msgs::msg::Path{},
          tag + ":goal_rejected");
      }
    };
  options.result_callback =
    [this, generation](const PlanHandle::WrappedResult & result) {
      PlanResult callback;
      std::string tag;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!qr_plan_active_ || generation != qr_plan_generation_) {return;}
        qr_plan_active_ = false;
        qr_plan_cancel_pending_ = false;
        qr_plan_goal_response_pending_ = false;
        qr_plan_handle_.reset();
        callback = std::move(qr_plan_callback_);
        tag = qr_plan_tag_;
        qr_plan_callback_ = nullptr;
      }
      const bool ok = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        result.result && result.result->path.poses.size() >= 2U;
      std::ostringstream detail;
      detail << tag << ":result_code=" << resultCodeName(result.code)
             << ":nav2_error_code=unavailable_in_humble_compute_path_result"
             << ":result_payload=" << (result.result ? "present" : "missing");
      publishAudit("qr_compute_result", detail.str(), generation);
      if (callback) {
        callback(generation, ok,
          ok ? result.result->path : nav_msgs::msg::Path{}, detail.str());
      }
    };
  plan_client_->async_send_goal(goal, options);
  publishAudit(
    "qr_compute_goal_sent",
    "planner=" + planner_id + ",seed=" + tag, generation);
  return true;
}

bool MotionActionArbiter::replaceFollowPath(
  const nav_msgs::msg::Path & path, const std::string & controller_id,
  const std::string & goal_checker_id, const std::string & tag,
  MotionResult callback)
{
  const auto sent_at = std::chrono::steady_clock::now();
  const bool reverse = tag.find("reverse") != std::string::npos;
  const double speed_limit = reverse ? reverse_speed_limit_mps_ :
    forward_speed_limit_mps_;
  const double expected_sec = pathLength(path) /
    std::max(0.12, 0.35 * std::max(0.01, speed_limit));
  const double hard_sec = std::clamp(expected_sec + 5.0, 5.0, 40.0);
  std::uint64_t generation;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!motion_active_ || motion_cancel_pending_ || plan_active_ ||
      qr_plan_active_ || replacement_pending_ || path.poses.size() < 2U)
    {
      return false;
    }
    // Do not mutate the current motion fields until Nav2 accepts the new
    // goal.  This keeps the old FollowPath and its callback authoritative
    // if the replacement is rejected.
    replacement_pending_ = true;
    replacement_old_generation_ = generation_;
    replacement_generation_ = generation_ + 1U;
    generation = replacement_generation_;
    replacement_old_handle_ = follow_handle_;
    replacement_callback_ = std::move(callback);
    replacement_tag_ = tag;
  }

  FollowPath::Goal goal;
  goal.path = path;
  goal.controller_id = controller_id;
  goal.goal_checker_id = goal_checker_id;
  rclcpp_action::Client<FollowPath>::SendGoalOptions options;
  options.goal_response_callback =
    [this, generation, sent_at, hard_sec](FollowHandle::SharedPtr handle) {
      bool rejected = false;
      MotionResult rejected_callback;
      std::string tag;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!replacement_pending_ || generation != replacement_generation_) {
          if (handle) {follow_client_->async_cancel_goal(handle);}
          return;
        }
        tag = replacement_tag_;
        if (!handle) {
          rejected = true;
          rejected_callback = std::move(replacement_callback_);
          replacement_pending_ = false;
          replacement_old_handle_.reset();
          replacement_tag_.clear();
        } else {
          generation_ = generation;
          motion_tag_ = replacement_tag_;
          motion_callback_ = std::move(replacement_callback_);
          follow_handle_ = handle;
          motion_cancel_pending_ = false;
          goal_response_pending_ = false;
          active_started_ = sent_at;
          action_deadline_ = sent_at +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(hard_sec));
          cancel_deadline_ = {};
          replacement_pending_ = false;
          replacement_old_handle_.reset();
          replacement_tag_.clear();
        }
      }
      if (rejected && rejected_callback) {
        publishAudit("follow_replacement_rejected", tag, generation);
        rejected_callback(generation, rclcpp_action::ResultCode::ABORTED,
          tag + ":replacement_goal_rejected");
      } else if (!rejected) {
        publishAudit("follow_replacement_accepted", tag, generation);
      }
    };
  options.result_callback =
    [this, generation](const FollowHandle::WrappedResult & result) {
      MotionResult callback;
      std::string tag;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!motion_active_ || generation != generation_) {return;}
        motion_active_ = false;
        terminal_release_confirmed_ = true;
        motion_cancel_pending_ = false;
        goal_response_pending_ = false;
        follow_handle_.reset();
        callback = std::move(motion_callback_);
        tag = motion_tag_;
        motion_callback_ = nullptr;
      }
      std::ostringstream detail;
      detail << tag << ":result_code=" << resultCodeName(result.code)
             << ":nav2_error_code=unavailable_in_humble_follow_path_result"
             << ":result_payload=" << (result.result ? "present" : "missing");
      publishAudit("follow_result", detail.str(), generation);
      if (callback) {callback(generation, result.code, detail.str());}
    };
  follow_client_->async_send_goal(goal, options);
  publishAudit("follow_replacement_sent", tag, generation);
  return true;
}

void MotionActionArbiter::cancelQrFollowPathAsync(const std::string & reason)
{
  FollowHandle::SharedPtr motion_handle;
  PlanHandle::SharedPtr qr_plan_handle;
  std::uint64_t retired_generation = 0U;
  std::uint64_t retired_qr_generation = 0U;
  std::uint64_t invalidated_generation = 0U;
  bool retire_motion = false;
  bool retire_qr_plan = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    retired_generation = generation_;
    retired_qr_generation = qr_plan_generation_;
    retire_motion = motion_active_ && motion_tag_.rfind("qr:", 0) == 0;
    retire_qr_plan = qr_plan_active_;
    if (!retire_motion && !retire_qr_plan && !replacement_pending_) {
      publishAudit("qr_motion_cancel_ignored", reason, generation_);
      return;
    }

    // Invalidate every callback before releasing the local ownership. A late
    // Nav2 result therefore cannot call a QR completion closure or resurrect
    // the old path while Direct/Reverse preparation is running.
    ++generation_;
    ++qr_plan_generation_;
    invalidated_generation = generation_;
    if (retire_motion) {
      motion_handle = follow_handle_;
      motion_active_ = false;
      terminal_release_confirmed_ = true;
      motion_cancel_pending_ = false;
      goal_response_pending_ = false;
      follow_handle_.reset();
      motion_callback_ = nullptr;
      motion_tag_.clear();
    }
    if (retire_qr_plan) {
      qr_plan_handle = qr_plan_handle_;
      qr_plan_active_ = false;
      qr_plan_cancel_pending_ = false;
      qr_plan_goal_response_pending_ = false;
      qr_plan_handle_.reset();
      qr_plan_callback_ = nullptr;
      qr_plan_tag_.clear();
    }
    // A replacement goal that has not received its response is canceled by
    // its generation-aware response callback. If it was already accepted,
    // follow_handle_ above is the accepted handle and is canceled directly.
    replacement_pending_ = false;
    replacement_old_handle_.reset();
    replacement_callback_ = nullptr;
    replacement_tag_.clear();
  }

  if (motion_handle) {
    follow_client_->async_cancel_goal(motion_handle);
  } else if (retire_motion) {
    follow_client_->async_cancel_all_goals();
  }
  if (qr_plan_handle) {
    plan_client_->async_cancel_goal(qr_plan_handle);
  }
  publishAudit("qr_motion_cancel_requested", reason, retired_generation);
  publishAudit(
    "qr_generation_invalidated",
    "motion_generation=" + std::to_string(retired_generation) +
    ",qr_plan_generation=" + std::to_string(retired_qr_generation),
    invalidated_generation);
}

bool MotionActionArbiter::computePathImpl(
  const geometry_msgs::msg::PoseStamped * start_pose,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const std::string & planner_id, const std::string & tag,
  PlanResult callback)
{
  std::uint64_t generation;
  const auto sent_at = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (plan_active_ || motion_active_) {return false;}
    generation = ++generation_;
    plan_active_ = true;
    terminal_release_confirmed_ = false;
    plan_cancel_pending_ = false;
    plan_tag_ = tag;
    plan_callback_ = std::move(callback);
    plan_handle_.reset();
    active_started_ = sent_at;
    goal_response_pending_ = true;
    compute_warning_emitted_ = false;
    goal_response_deadline_ = sent_at + std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(goal_response_hard_sec_));
    action_deadline_ = sent_at + std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(compute_hard_sec_));
    cancel_deadline_ = {};
  }

  ComputePath::Goal goal;
  goal.goal = goal_pose;
  goal.use_start = start_pose != nullptr;
  if (start_pose) {goal.start = *start_pose;}
  goal.planner_id = planner_id;
  rclcpp_action::Client<ComputePath>::SendGoalOptions options;
  options.goal_response_callback =
    [this, generation](PlanHandle::SharedPtr handle) {
      bool cancel = false;
      PlanResult rejected_callback;
      std::string tag;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!plan_active_ || generation != generation_) {
          if (handle) {plan_client_->async_cancel_goal(handle);}
          return;
        }
        goal_response_pending_ = false;
        if (!handle) {
          plan_active_ = false;
          terminal_release_confirmed_ = true;
          rejected_callback = std::move(plan_callback_);
          tag = plan_tag_;
          plan_callback_ = nullptr;
        } else {
          plan_handle_ = handle;
          cancel = plan_cancel_pending_;
        }
      }
      if (cancel && handle) {plan_client_->async_cancel_goal(handle);}
      if (rejected_callback) {
        publishAudit("compute_goal_rejected", tag, generation);
        rejected_callback(generation, false, nav_msgs::msg::Path{},
          tag + ":goal_rejected");
      }
    };
  options.result_callback =
    [this, generation](const PlanHandle::WrappedResult & result) {
      PlanResult callback;
      std::string tag;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!plan_active_ || generation != generation_) {return;}
        plan_active_ = false;
        terminal_release_confirmed_ = true;
        plan_cancel_pending_ = false;
        goal_response_pending_ = false;
        plan_handle_.reset();
        callback = std::move(plan_callback_);
        tag = plan_tag_;
        plan_callback_ = nullptr;
      }
      const bool ok = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        result.result && result.result->path.poses.size() >= 2U;
      std::ostringstream detail;
      detail << tag << ":result_code=" << resultCodeName(result.code)
             << ":nav2_error_code=unavailable_in_humble_compute_path_result"
             << ":result_payload=" << (result.result ? "present" : "missing");
      publishAudit("compute_result", detail.str(), generation);
      if (callback) {
        callback(generation, ok,
          ok ? result.result->path : nav_msgs::msg::Path{}, detail.str());
      }
    };
  plan_client_->async_send_goal(goal, options);
  publishAudit(
    "compute_goal_sent",
    "planner=" + planner_id + ",seed=" + tag, generation);
  return true;
}

void MotionActionArbiter::cancelAllAsync(const std::string & reason)
{
  FollowHandle::SharedPtr motion_handle;
  PlanHandle::SharedPtr plan_handle;
  PlanHandle::SharedPtr qr_plan_handle;
  const auto requested_at = std::chrono::steady_clock::now();
  std::uint64_t generation = 0U;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    generation = generation_;
    if (motion_active_) {
      motion_cancel_pending_ = true;
      motion_handle = follow_handle_;
      cancel_deadline_ = requested_at + std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(cancel_terminal_hard_sec_));
    }
    if (plan_active_) {
      plan_cancel_pending_ = true;
      plan_handle = plan_handle_;
      cancel_deadline_ = requested_at + std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(cancel_terminal_hard_sec_));
    }
    if (qr_plan_active_) {
      qr_plan_cancel_pending_ = true;
      qr_plan_handle = qr_plan_handle_;
    }
    // An unaccepted replacement must never be allowed to preempt a motion
    // after a mission-wide cancel. The old handle is still canceled below;
    // a late replacement response will see the cleared pending flag and
    // immediately cancel its newly accepted goal.
    if (replacement_pending_) {
      replacement_pending_ = false;
      replacement_old_handle_.reset();
      replacement_callback_ = nullptr;
      replacement_tag_.clear();
    }
  }
  if (motion_handle) {follow_client_->async_cancel_goal(motion_handle);}
  if (plan_handle) {plan_client_->async_cancel_goal(plan_handle);}
  if (qr_plan_handle) {plan_client_->async_cancel_goal(qr_plan_handle);}
  publishAudit("cancel_requested", reason, generation);
  RCLCPP_WARN(node_->get_logger(), "async cancel requested: %s", reason.c_str());
}

void MotionActionArbiter::onDeadlineTick()
{
  const auto stamp = std::chrono::steady_clock::now();
  FollowHandle::SharedPtr motion_handle;
  PlanHandle::SharedPtr plan_handle;
  MotionResult motion_callback;
  PlanResult plan_callback;
  PlanHandle::SharedPtr qr_plan_handle;
  PlanResult qr_plan_callback;
  std::uint64_t qr_retired_generation = 0U;
  std::uint64_t retired_generation = 0U;
  std::string detail;
  std::string qr_detail;
  bool cancel_all_motion = false;
  bool cancel_all_plan = false;
  bool qr_only_timeout = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (plan_active_ && !compute_warning_emitted_ &&
      stamp - active_started_ >= std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(compute_warning_sec_)))
    {
      compute_warning_emitted_ = true;
      RCLCPP_WARN(
        node_->get_logger(), "ComputePath warning deadline reached: %s",
        plan_tag_.c_str());
    }
    if (qr_plan_active_ && !qr_plan_warning_emitted_ &&
      stamp - qr_plan_started_ >= std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(compute_warning_sec_)))
    {
      qr_plan_warning_emitted_ = true;
      RCLCPP_WARN(
        node_->get_logger(), "QR background ComputePath warning deadline reached: %s",
        qr_plan_tag_.c_str());
    }
    const bool goal_response_timeout =
      goal_response_pending_ && stamp >= goal_response_deadline_;
    const bool action_timeout =
      (motion_active_ || plan_active_) && stamp >= action_deadline_;
    const bool cancel_timeout =
      (motion_cancel_pending_ || plan_cancel_pending_) &&
      cancel_deadline_ != std::chrono::steady_clock::time_point{} &&
      stamp >= cancel_deadline_;
    const bool qr_plan_timeout = qr_plan_active_ &&
      ((qr_plan_goal_response_pending_ && stamp >= qr_plan_goal_response_deadline_) ||
      stamp >= qr_plan_deadline_);
    if (!goal_response_timeout && !action_timeout && !cancel_timeout &&
      !qr_plan_timeout) {return;}

    if (qr_plan_timeout) {
      qr_retired_generation = qr_plan_generation_;
      qr_plan_handle = qr_plan_handle_;
      qr_plan_callback = std::move(qr_plan_callback_);
      qr_detail = qr_plan_tag_ + ":action_hard_timeout";
      qr_plan_active_ = false;
      qr_plan_cancel_pending_ = false;
      qr_plan_goal_response_pending_ = false;
      qr_plan_handle_.reset();
      qr_plan_callback_ = nullptr;
    }
    if (!goal_response_timeout && !action_timeout && !cancel_timeout) {
      // The QR replanning timeout is intentionally independent of the
      // active FollowPath deadline; never retire the vehicle motion here.
      qr_only_timeout = true;
    }

    if (qr_only_timeout) {
      // Leave the active FollowPath and all normal arbiter state untouched.
    } else {
      retired_generation = generation_;
      ++generation_;
      terminal_release_confirmed_ = false;
      const char * timeout_name = cancel_timeout ? "cancel_terminal_timeout" :
        (goal_response_timeout ? "goal_response_timeout" : "action_hard_timeout");
      if (motion_active_) {
        detail = motion_tag_ + ":" + timeout_name;
        motion_handle = follow_handle_;
        cancel_all_motion = !motion_handle;
        motion_callback = std::move(motion_callback_);
        motion_callback_ = nullptr;
        motion_active_ = false;
        motion_cancel_pending_ = false;
        follow_handle_.reset();
      } else if (plan_active_) {
        detail = plan_tag_ + ":" + timeout_name;
        plan_handle = plan_handle_;
        cancel_all_plan = !plan_handle;
        plan_callback = std::move(plan_callback_);
        plan_callback_ = nullptr;
        plan_active_ = false;
        plan_cancel_pending_ = false;
        plan_handle_.reset();
      }
      goal_response_pending_ = false;
      cancel_deadline_ = {};
    }
  }

  if (motion_handle) {follow_client_->async_cancel_goal(motion_handle);}
  else if (cancel_all_motion) {follow_client_->async_cancel_all_goals();}
  if (plan_handle) {plan_client_->async_cancel_goal(plan_handle);}
  else if (cancel_all_plan) {plan_client_->async_cancel_all_goals();}
  if (qr_plan_handle) {plan_client_->async_cancel_goal(qr_plan_handle);}
  if (qr_only_timeout) {
    if (qr_plan_callback) {
      publishAudit("qr_compute_deadline_retired", qr_detail, qr_retired_generation);
      qr_plan_callback(
        qr_retired_generation, false, nav_msgs::msg::Path{}, qr_detail);
    }
    return;
  }
  RCLCPP_ERROR(node_->get_logger(), "Action retired by deadline: %s", detail.c_str());
  publishAudit("deadline_retired", detail, retired_generation);
  if (motion_callback) {
    motion_callback(
      retired_generation, rclcpp_action::ResultCode::ABORTED, detail);
  }
  if (plan_callback) {
      plan_callback(
      retired_generation, false, nav_msgs::msg::Path{}, detail);
  }
  if (qr_plan_callback) {
    publishAudit("qr_compute_deadline_retired", qr_detail, qr_retired_generation);
    qr_plan_callback(
      qr_retired_generation, false, nav_msgs::msg::Path{}, qr_detail);
  }
}

}  // namespace hobot_nav
