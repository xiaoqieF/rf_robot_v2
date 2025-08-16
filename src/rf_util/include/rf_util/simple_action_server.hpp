#pragma once

#include <bits/types/struct_sched_param.h>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <rclcpp_action/types.hpp>
#include <sched.h>
#include <string>

#include "elog/elog.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace rf_util
{

template <typename ActionT>
class SimpleActionServer
{
public:
    using ExecuteCallback =  std::function<void()>;
    using CompletionCallback = std::function<void()>;
    using UniquePtr = std::unique_ptr<SimpleActionServer<ActionT>>;
    using SharedPtr = std::shared_ptr<SimpleActionServer<ActionT>>;

    SimpleActionServer(rclcpp::Node::SharedPtr node,
                       const std::string& action_name,
                       ExecuteCallback execute_callback,
                       CompletionCallback completion_callback = nullptr,
                       bool realtime = false)
        : node_(node),
          action_name_(action_name),
          execute_callback_(execute_callback),
          completion_callback_(completion_callback),
          realtime_(realtime)
    {
        action_server_ = rclcpp_action::create_server<ActionT>(
            node_,
            action_name_,
            [this](const rclcpp_action::GoalUUID& uuid,
                   std::shared_ptr<const typename ActionT::Goal> goal_handle) {
                return this->handleGoal(uuid, goal_handle);
            },
            [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle) {
                return this->handleCancel(goal_handle);
            },
            [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle) {
                return this->handleAccepted(goal_handle);
            });
    }

    bool isActive(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle) const
    {
        return goal_handle && goal_handle->is_active();
    }

    bool isRunning()
    {
        return execution_future_.valid() &&
            execution_future_.wait_for(std::chrono::seconds(0)) == std::future_status::timeout;
    }

    bool isPreemptRequested() const
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        return preempt_requested_;
    }

    const std::shared_ptr<const typename ActionT::Goal> getCurrentGoal() const
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (!isActive(current_handle_)) {
            errorLog("[ActionServer] No active goal handle.");
            return {};
        }

        return current_handle_->get_goal();
    }

    bool isCancelRequested() const
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (current_handle_ == nullptr) {
            return false;
        }

        if (pending_handle_) {
            return pending_handle_->is_canceling();
        }

        return current_handle_->is_canceling();
    }

    void publishFeedback(
        const std::shared_ptr<typename ActionT::Feedback> feedback_msg)
    {
        if (!isActive(current_handle_)) {
            errorLog("Cannot publish feedback, no active goal handle.");
            return;
        }

        current_handle_->publish_feedback(feedback_msg);
    }

    void terminate(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>& goal_handle,
        std::shared_ptr<typename ActionT::Result> result = std::make_shared<typename ActionT::Result>())
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (!isActive(goal_handle)) {
            return;
        }

        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
        } else {
            warnLog("Aborting goal handle.");
            goal_handle->abort(result);
        }

        goal_handle.reset();
    }

    void terminateAll(
        std::shared_ptr<typename ActionT::Result> result = std::make_shared<typename ActionT::Result>())
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        terminate(current_handle_, result);
        terminate(pending_handle_, result);

        preempt_requested_ = false;
    }

    void terminateCurrent(
        std::shared_ptr<typename ActionT::Result> result = std::make_shared<typename ActionT::Result>())
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        terminate(current_handle_, result);
    }

    void succeededCurrent(
        std::shared_ptr<typename ActionT::Result> result = std::make_shared<typename ActionT::Result>())
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (isActive(current_handle_)) {
            debugLog("Succeeding current goal handle for action.");
            current_handle_->succeed(result);
            current_handle_.reset();
        }
    }

    const std::shared_ptr<const typename ActionT::Goal> acceptPendingGoal()
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (!isActive(pending_handle_)) {
            errorLog("No pending goal to accept for action.");
            return {};
        }

        if (isActive(current_handle_) && current_handle_ != pending_handle_) {
            current_handle_->abort(std::make_shared<typename ActionT::Result>());
        }

        current_handle_ = pending_handle_;
        pending_handle_.reset();
        preempt_requested_ = false;

        debugLog("Accept preempted goal");

        return current_handle_->get_goal();
    }

private:
    // Always accepted, no need to check goal
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const typename ActionT::Goal>)
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        debugLog("Received goal request for action.");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        debugLog("Received cancel request for action.");

        if (!goal_handle->is_active()) {
            warnLog("Cancel request received for an inactive goal."
                "reject it.");
            return rclcpp_action::CancelResponse::REJECT;
        }

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        debugLog("Accepted goal for action.");

        if (isActive(current_handle_) || isRunning()) {
            debugLog("Another goal is already running, pending this one.");

            if (isActive(pending_handle_)) {
                debugLog("Another pending goal exists, terminat and replace it.");
                terminate(pending_handle_);
            }
            pending_handle_ = goal_handle;
            preempt_requested_ = true;
        } else {
            if (isActive(pending_handle_)) {
                errorLog("Pending goal exists, but no running goal. This should not happen.");
                terminate(pending_handle_);
                preempt_requested_ = false;
            }

            current_handle_ = goal_handle;

            debugLog("Starting execution for action.");
            execution_future_ = std::async(std::launch::async, [this]() {
                this->setThreadPriority();
                this->work();
            });
        }
    }

    void setThreadPriority()
    {
        if (realtime_) {
            sched_param param;
            param.sched_priority = 49;
            if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
                errorLog("Failed to set realtime priority for action server thread: " + std::string(strerror(errno)));
                throw std::runtime_error("Failed to set realtime priority for action server thread");
            }

            debugLog("Set realtime priority for action server thread.");
        }
    }

    void work()
    {
        while (rclcpp::ok() && isActive(current_handle_)) {
            debugLog("Executing action.");
            try {
                execute_callback_();
            } catch (const std::exception& e) {
                errorLog(std::string("Exception during action execution: ") + e.what());
                terminateAll();

                if (completion_callback_) {
                    completion_callback_();
                }
                return;
            }

            debugLog("processing new goal handles.");
            std::lock_guard<std::recursive_mutex> lock(update_mutex_);
            if (isActive(current_handle_)) {
                warnLog("Action execution completed, but goal handle is still active.");
                terminate(current_handle_);

                if (completion_callback_) {
                    completion_callback_();
                }
            }

            if (isActive(pending_handle_)) {
                debugLog("Pending goal exists, switching to it.");
                acceptPendingGoal();
            } else {
                debugLog("No pending goal, stopping execution.");
                break;
            }
        }
        debugLog("Action execution thread done.");
    }

private:
    void debugLog(const std::string& message) const
    {
        elog::debug("[{}] [ActionServer] {}", action_name_, message);
    }

    void warnLog(const std::string& message) const
    {
        elog::warn("[{}] [ActionServer] {}", action_name_, message);
    }

    void errorLog(const std::string& message) const
    {
        elog::error("[{}] [ActionServer] {}", action_name_, message);
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::string action_name_;
    ExecuteCallback execute_callback_;
    CompletionCallback completion_callback_;
    bool realtime_{false};
    std::future<void> execution_future_;
    typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;

    mutable std::recursive_mutex update_mutex_;
    bool preempt_requested_{false};

    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> current_handle_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> pending_handle_;

};

} // namespace rf_util