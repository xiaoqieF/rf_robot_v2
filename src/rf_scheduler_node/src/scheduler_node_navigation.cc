#include "scheduler_node_internal.hpp"

#include <chrono>
#include <cstdint>
#include <string>

namespace rf_scheduler
{

namespace
{

constexpr uint32_t kMaxEventDrivenReplans = 2;

} // namespace

SchedulerNode::NavigateStatus SchedulerNode::navigateToPose(
    const PoseStampedT& goal_pose,
    const std::function<bool()>& should_interrupt,
    std::string* failure_reason)
{
    using namespace std::chrono_literals;
    using detail::FutureWaitStatus;

    std::lock_guard<std::mutex> navigation_lock(navigation_mutex_);

    auto set_failure_reason = [failure_reason](const std::string& reason) {
        if (failure_reason != nullptr) {
            *failure_reason = reason;
        }
    };

    for (uint32_t replan_attempt = 0;;) {
        if (!action_to_pose_client_->wait_for_action_server(500ms)) {
            set_failure_reason("global planner action server is not available");
            return NavigateStatus::SERVER_UNAVAILABLE;
        }

        ActionToPose::Goal plan_goal;
        plan_goal.goal = goal_pose;

        auto future_goal_handle = action_to_pose_client_->async_send_goal(plan_goal);
        switch (detail::waitForFuture(future_goal_handle, should_interrupt)) {
            case FutureWaitStatus::READY:
                break;
            case FutureWaitStatus::INTERRUPTED:
                detail::cancelFutureGoalIfAccepted(future_goal_handle, [this](const auto& goal_handle) {
                    action_to_pose_client_->async_cancel_goal(goal_handle);
                });
                cancelActiveNavigationGoals();
                set_failure_reason("navigation interrupted while waiting for planner goal acceptance");
                return NavigateStatus::INTERRUPTED;
            case FutureWaitStatus::SHUTDOWN:
                set_failure_reason("ROS shutdown while waiting for planner goal acceptance");
                return NavigateStatus::INTERRUPTED;
        }

        auto plan_goal_handle = future_goal_handle.get();
        if (!plan_goal_handle) {
            set_failure_reason("global planner rejected goal");
            return NavigateStatus::REJECTED;
        }

        {
            std::lock_guard<std::mutex> lock(goal_pose_mutex_);
            current_plan_goal_handle_ = plan_goal_handle;
        }

        if (should_interrupt()) {
            cancelActiveNavigationGoals();
            set_failure_reason("navigation interrupted right after planner goal acceptance");
            return NavigateStatus::INTERRUPTED;
        }

        auto future_plan_result = action_to_pose_client_->async_get_result(plan_goal_handle);
        switch (detail::waitForFuture(future_plan_result, should_interrupt)) {
            case FutureWaitStatus::READY:
                break;
            case FutureWaitStatus::INTERRUPTED:
                cancelActiveNavigationGoals();
                set_failure_reason("navigation interrupted while waiting for planner result");
                return NavigateStatus::INTERRUPTED;
            case FutureWaitStatus::SHUTDOWN:
                {
                    std::lock_guard<std::mutex> lock(goal_pose_mutex_);
                    current_plan_goal_handle_.reset();
                }
                set_failure_reason("ROS shutdown while waiting for planner result");
                return NavigateStatus::INTERRUPTED;
        }

        {
            std::lock_guard<std::mutex> lock(goal_pose_mutex_);
            current_plan_goal_handle_.reset();
        }

        auto plan_result = future_plan_result.get();
        if (plan_result.code != rclcpp_action::ResultCode::SUCCEEDED || !plan_result.result) {
            set_failure_reason("global planner failed to produce a valid result");
            return NavigateStatus::PLANNER_FAILED;
        }

        if (plan_result.result->path.poses.empty()) {
            set_failure_reason("global planner returned an empty path");
            return NavigateStatus::PLANNER_FAILED;
        }

        if (should_interrupt()) {
            set_failure_reason("navigation interrupted before local tracking started");
            return NavigateStatus::INTERRUPTED;
        }

        if (!follow_path_client_->wait_for_action_server(500ms)) {
            set_failure_reason("local planner action server is not available");
            return NavigateStatus::SERVER_UNAVAILABLE;
        }

        ActionFollowPath::Goal follow_goal;
        follow_goal.path = plan_result.result->path;

        auto future_follow_handle = follow_path_client_->async_send_goal(follow_goal);
        switch (detail::waitForFuture(future_follow_handle, should_interrupt)) {
            case FutureWaitStatus::READY:
                break;
            case FutureWaitStatus::INTERRUPTED:
                detail::cancelFutureGoalIfAccepted(future_follow_handle, [this](const auto& goal_handle) {
                    follow_path_client_->async_cancel_goal(goal_handle);
                });
                cancelActiveNavigationGoals();
                set_failure_reason("navigation interrupted while waiting for local planner goal acceptance");
                return NavigateStatus::INTERRUPTED;
            case FutureWaitStatus::SHUTDOWN:
                set_failure_reason("ROS shutdown while waiting for local planner goal acceptance");
                return NavigateStatus::INTERRUPTED;
        }

        auto follow_goal_handle = future_follow_handle.get();
        if (!follow_goal_handle) {
            set_failure_reason("local planner rejected the tracking goal");
            return NavigateStatus::REJECTED;
        }

        {
            std::lock_guard<std::mutex> lock(goal_pose_mutex_);
            current_follow_goal_handle_ = follow_goal_handle;
        }

        if (should_interrupt()) {
            cancelActiveNavigationGoals();
            set_failure_reason("navigation interrupted right after local planner goal acceptance");
            return NavigateStatus::INTERRUPTED;
        }

        auto future_follow_result = follow_path_client_->async_get_result(follow_goal_handle);
        switch (detail::waitForFuture(future_follow_result, should_interrupt)) {
            case FutureWaitStatus::READY:
                break;
            case FutureWaitStatus::INTERRUPTED:
                cancelActiveNavigationGoals();
                set_failure_reason("navigation interrupted while waiting for local planner result");
                return NavigateStatus::INTERRUPTED;
            case FutureWaitStatus::SHUTDOWN:
                {
                    std::lock_guard<std::mutex> lock(goal_pose_mutex_);
                    current_follow_goal_handle_.reset();
                }
                set_failure_reason("ROS shutdown while waiting for local planner result");
                return NavigateStatus::INTERRUPTED;
        }

        {
            std::lock_guard<std::mutex> lock(goal_pose_mutex_);
            current_follow_goal_handle_.reset();
        }

        auto follow_result = future_follow_result.get();
        switch (follow_result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                return NavigateStatus::SUCCEEDED;
            case rclcpp_action::ResultCode::CANCELED:
                set_failure_reason("path tracking was canceled");
                return NavigateStatus::INTERRUPTED;
            case rclcpp_action::ResultCode::ABORTED:
                if (follow_result.result &&
                    follow_result.result->error_code == ActionFollowPath::Result::REPLAN_REQUIRED &&
                    replan_attempt < kMaxEventDrivenReplans) {
                    ++replan_attempt;
                    SCHED_WARN(
                        "Local planner requested replan {}/{} for goal ({:.2f}, {:.2f}): {}",
                        replan_attempt,
                        kMaxEventDrivenReplans,
                        goal_pose.pose.position.x,
                        goal_pose.pose.position.y,
                        follow_result.result->error_msg);
                    continue;
                }

                if (follow_result.result) {
                    if (follow_result.result->error_code == ActionFollowPath::Result::REPLAN_REQUIRED) {
                        set_failure_reason(
                            "path tracking exhausted event-driven replans: " + follow_result.result->error_msg);
                    } else {
                        set_failure_reason("path tracking aborted: " + follow_result.result->error_msg);
                    }
                } else {
                    set_failure_reason("path tracking aborted without a result payload");
                }
                return NavigateStatus::TRACKING_FAILED;
            default:
                set_failure_reason("path tracking finished with an unexpected result code");
                return NavigateStatus::TRACKING_FAILED;
        }
    }
}

} // namespace rf_scheduler
