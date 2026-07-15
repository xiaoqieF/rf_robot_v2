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

// 栅格值不大于此阈值才视为可通行。调大可扩大前沿和候选目标的可用区域，但会更接近膨胀区或障碍物。
constexpr int8_t kFreeThreshold = 65;
// 等待建图相关服务上线的最长时间；过短会在节点启动较慢时误判服务不可用。
constexpr std::chrono::milliseconds kDefaultServiceWait{1000};
// 建图相关服务请求的最长响应时间；过短会在系统繁忙时中断探索任务。
constexpr std::chrono::milliseconds kDefaultResponseWait{5000};
// 启动 /build_map 后等待首帧 /slam_map 的时间；调大可适应启动较慢的建图链路。
constexpr std::chrono::milliseconds kMapWarmupWait{4000};
// 停止 /build_map 后等待最终地图刷新的时间；调大可降低保存到旧地图的概率。
constexpr std::chrono::milliseconds kFinalMapWait{2000};
// 未找到可导航目标或一轮导航均失败后的重试间隔；调小响应更快，但会增加地图处理和日志频率。
constexpr std::chrono::milliseconds kFrontierRetrySleep{1000};
// 起图阶段必须积累足够空闲栅格的等待上限；调大可容忍建图起步慢，调小可更快暴露建图异常。
constexpr std::chrono::seconds kInitialMappingTimeout{20};
// 地图已起图后持续找不到可用前沿时的完成判定时长；调大可减少过早结束，代价是完成更慢。
constexpr std::chrono::seconds kNoFrontierStableDuration{8};
// 连续导航失败达到该次数即终止任务；调大可尝试更多备选目标，调小可更快避开异常导航状态。
constexpr uint32_t kDefaultMaxFrontierFailures = 10;
// 构成有效前沿的最小连通栅格数；调大可过滤噪声小前沿，调小可探索狭小或碎片化区域。
constexpr uint32_t kDefaultMinFrontierClusterSize = 4;
// 成功到达前沿后，抑制该前沿中心周围区域的半径（米）；调大可减少重复探索，过大可能跳过邻近未覆盖区域。
constexpr double kReachedFrontierSuppressRadius = 0.5;
// 目标导航失败后，抑制失败目标周围候选点的半径（米）；调大可避免反复失败，过大可能丢弃可行替代点。
constexpr double kFailedGoalSuppressRadius = 0.35;
// 开始前沿探索前所需的最少空闲栅格数；调大可确保地图更稳定后再探索，调小可更早开始移动。
constexpr std::size_t kMinimumSeedFreeCells = 50;
// 宽松候选目标筛选中的障碍阈值：大于此值即拒绝；调小更保守，调大更容易接受靠近高代价区的目标。
constexpr int8_t kGoalOccupiedThreshold = 80;
// 候选目标与前沿之间的最小退让距离（米）；调大可避免贴近未知边界，过大可能在窄区找不到目标。
constexpr double kGoalMinStandoffMeters = 0.10;
// 目标评分偏好的前沿退让距离（米）；调大使机器人倾向于离前沿更远处停靠，不改变最小安全距离。
constexpr double kGoalPreferredStandoffMeters = 0.20;
// 从前沿向已知自由区搜索候选目标的最大距离（米）；调大可找到更安全的目标，代价是计算量增加且离前沿更远。
constexpr double kGoalSearchRadiusMeters = 0.50;
// 候选目标周围必须通过安全检查的半径（米）；调大提高避障余量，过大可能拒绝狭窄区域内的全部目标。
constexpr double kGoalClearanceRadiusMeters = 0.25;
// 安全检查中候选目标中心必须保持已知自由的半径（米）；调大可避免目标落在未知边缘，过大将降低目标可用性。
constexpr double kGoalKnownRadiusMeters = 0.10;
// 朝向与候选点方向一致时的评分权重；调大更偏好少转向的目标，调小则更重视前沿规模、间隙和距离。
constexpr double kFrontierHeadingAlignmentWeight = 1.5;

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

inline double yawFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion)
{
    const double sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y);
    const double cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z);
    return std::atan2(sin_yaw, cos_yaw);
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
