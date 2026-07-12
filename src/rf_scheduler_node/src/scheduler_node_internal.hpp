#pragma once

#include "rf_scheduler_node/scheduler_node.hpp"

#include "elog/elog.h"

#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace rf_scheduler::detail
{

class MappingJob
{
public:
    explicit MappingJob(SchedulerNode& node);

    void run();

private:
    struct ExplorationContext
    {
        std::vector<SchedulerNode::FrontierCandidate> frontier_candidates;
        bool map_seeded = false;
        std::size_t frontier_cluster_count = 0;
        std::size_t suppressed_cluster_count = 0;
        std::size_t rejected_cluster_count = 0;
    };

    enum class FrontierNavigationResult
    {
        REACHED_FRONTIER,
        RETRY_LATER,
        CANCELED,
        FAILURE_LIMIT_REACHED,
    };

    void finish(
        bool succeeded,
        uint16_t error_code,
        std::string message,
        bool save_map_before_cleanup);
    void resetSuppressedFrontiers();
    void publishFeedback(const std::string& state, std::size_t frontier_candidates) const;
    bool shouldInterrupt() const;
    bool prepare(std::string* reason);
    void cleanup(bool stop_build_map);
    bool finishAndSave(std::string* reason);
    ExplorationContext collectContext() const;
    FrontierNavigationResult navigateFrontierCandidates(
        const std::vector<SchedulerNode::FrontierCandidate>& frontier_candidates,
        uint32_t& navigation_failures,
        std::string* failure_reason);

private:
    SchedulerNode& node_;
    std::shared_ptr<ActionExploreMap::Result> result_;
    rclcpp::Time start_time_;
};

constexpr int8_t kFreeThreshold = 65;
constexpr std::chrono::milliseconds kDefaultServiceWait{1000};
constexpr std::chrono::milliseconds kDefaultResponseWait{5000};
constexpr std::chrono::milliseconds kMapWarmupWait{4000};
constexpr std::chrono::milliseconds kFinalMapWait{2000};
constexpr std::chrono::milliseconds kFrontierRetrySleep{1000};
constexpr std::chrono::seconds kInitialMappingTimeout{20};
constexpr std::chrono::seconds kNoFrontierStableDuration{8};
constexpr uint32_t kDefaultMaxFrontierFailures = 10;
constexpr uint32_t kDefaultMinFrontierClusterSize = 8;
constexpr double kReachedGoalSuppressRadius = 0.2;
constexpr double kFailedGoalSuppressRadius = 0.35;
constexpr std::size_t kMinimumSeedFreeCells = 50;
constexpr int8_t kGoalOccupiedThreshold = 80;
constexpr double kGoalMinStandoffMeters = 0.20;
constexpr double kGoalPreferredStandoffMeters = 0.40;
constexpr double kGoalSearchRadiusMeters = 1.20;
constexpr double kGoalClearanceRadiusMeters = 0.20;
constexpr double kGoalKnownRadiusMeters = 0.10;

enum class FutureWaitStatus
{
    READY,
    INTERRUPTED,
    SHUTDOWN,
};

inline geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw)
{
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.z = std::sin(yaw * 0.5);
    quaternion.w = std::cos(yaw * 0.5);
    return quaternion;
}

inline bool isUnknownCell(const int8_t value)
{
    return value < 0;
}

inline bool isFreeCell(const int8_t value)
{
    return value >= 0 && value <= kFreeThreshold;
}

inline std::size_t countFreeCells(const OccupancyGridMsgT& map)
{
    std::size_t free_cells = 0;
    for (const auto value : map.data) {
        if (isFreeCell(value)) {
            ++free_cells;
        }
    }
    return free_cells;
}

inline double squaredDistance(
    const double ax,
    const double ay,
    const double bx,
    const double by)
{
    const double dx = ax - bx;
    const double dy = ay - by;
    return dx * dx + dy * dy;
}

template <typename FutureT, typename InterruptFn>
FutureWaitStatus waitForFuture(
    FutureT& future,
    const InterruptFn& should_interrupt,
    const std::chrono::milliseconds poll_interval = std::chrono::milliseconds(100))
{
    while (rclcpp::ok() && future.wait_for(poll_interval) != std::future_status::ready) {
        if (should_interrupt()) {
            return FutureWaitStatus::INTERRUPTED;
        }
    }

    return rclcpp::ok() ? FutureWaitStatus::READY : FutureWaitStatus::SHUTDOWN;
}

template <typename FutureT, typename CancelFn>
void cancelFutureGoalIfAccepted(FutureT& future, const CancelFn& cancel_goal)
{
    using namespace std::chrono_literals;

    if (future.wait_for(500ms) != std::future_status::ready) {
        return;
    }

    auto goal_handle = future.get();
    if (goal_handle) {
        cancel_goal(goal_handle);
    }
}

} // namespace rf_scheduler::detail

#define SCHED_INFO(fmt_str, ...) \
    do { \
        elog::info("[rf_scheduler_node] " fmt_str, ##__VA_ARGS__); \
    } while(0)

#define SCHED_WARN(fmt_str, ...) \
    do { \
        elog::warn("[rf_scheduler_node] " fmt_str, ##__VA_ARGS__); \
    } while(0)
