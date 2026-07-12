#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rf_robot_msgs/action/compute_path_to_pose.hpp"
#include "rf_robot_msgs/action/explore_map.hpp"
#include "rf_robot_msgs/action/follow_path.hpp"
#include "rf_robot_msgs/srv/req_ack.hpp"
#include "rf_util/simple_action_server.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace rf_scheduler
{

namespace detail
{
class MappingJob;
}

using PoseStampedT = geometry_msgs::msg::PoseStamped;
using OccupancyGridMsgT = nav_msgs::msg::OccupancyGrid;
using ActionToPose = rf_robot_msgs::action::ComputePathToPose;
using ActionFollowPath = rf_robot_msgs::action::FollowPath;
using ActionExploreMap = rf_robot_msgs::action::ExploreMap;
using ReqAckSrvT = rf_robot_msgs::srv::ReqAck;

class SchedulerNode : public rclcpp::Node
{
public:
    SchedulerNode()
        : Node("rf_scheduler_node"){}

    void init();

private:
    struct FrontierCandidate
    {
        PoseStampedT goal_pose;
        std::size_t cell_count = 0;
        double distance = 0.0;
        double score = 0.0;
        double frontier_x = 0.0;
        double frontier_y = 0.0;
    };

    struct SuppressedFrontier
    {
        double x = 0.0;
        double y = 0.0;
        double radius = 0.0;
        bool cluster_level = false;
    };

    enum class NavigateStatus
    {
        SUCCEEDED,
        INTERRUPTED,
        PLANNER_FAILED,
        TRACKING_FAILED,
        SERVER_UNAVAILABLE,
        REJECTED,
    };

    // Node lifecycle and top-level request handling.
    void loop();
    void handleGoalPoseMsg();
    void handleExploreMap();

    // Navigation pipeline.
    NavigateStatus navigateToPose(
        const PoseStampedT& goal_pose,
        const std::function<bool()>& should_interrupt,
        std::string* failure_reason = nullptr);

    // Shared services and map state helpers.
    bool callReqAckService(
        const rclcpp::Client<ReqAckSrvT>::SharedPtr& client,
        uint8_t trigger,
        const std::string& service_name,
        std::string* reason,
        std::chrono::milliseconds wait_for_service = std::chrono::milliseconds(1000),
        std::chrono::milliseconds wait_for_response = std::chrono::milliseconds(5000));
    bool waitForMapUpdate(std::chrono::milliseconds timeout, std::size_t min_update_count = 1);

    // Map and frontier utilities.
    std::shared_ptr<OccupancyGridMsgT> getLatestSlamMap() const;
    bool getRobotPoseInMap(PoseStampedT& pose) const;
    std::vector<FrontierCandidate> computeFrontierCandidates(
        const OccupancyGridMsgT& map,
        const PoseStampedT* robot_pose,
        std::size_t* frontier_cluster_count = nullptr,
        std::size_t* suppressed_cluster_count = nullptr,
        std::size_t* rejected_cluster_count = nullptr) const;
    void suppressFrontier(const FrontierCandidate& frontier_candidate, double radius);
    void suppressGoalPose(const PoseStampedT& goal_pose, double radius);
    bool isFrontierSuppressed(double x, double y, bool cluster_only = false) const;
    void cancelActiveNavigationGoals();
    bool hasPendingPreempt() const;

private:
    friend class detail::MappingJob;

    // Goal inputs and navigation state.
    rclcpp::Subscription<PoseStampedT>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<OccupancyGridMsgT>::SharedPtr slam_map_sub_;

    mutable std::mutex goal_pose_mutex_;
    PoseStampedT::SharedPtr goal_pose_msg_{nullptr};
    bool preempt_requested_{false};

    mutable std::mutex navigation_mutex_;
    rclcpp_action::Client<ActionToPose>::SharedPtr action_to_pose_client_{nullptr};
    rclcpp_action::Client<ActionFollowPath>::SharedPtr follow_path_client_{nullptr};
    rclcpp_action::ClientGoalHandle<ActionToPose>::SharedPtr current_plan_goal_handle_{nullptr};
    rclcpp_action::ClientGoalHandle<ActionFollowPath>::SharedPtr current_follow_goal_handle_{nullptr};

    // Service and action orchestration.
    rf_util::SimpleActionServer<ActionExploreMap>::UniquePtr explore_map_server_;
    rclcpp::Client<ReqAckSrvT>::SharedPtr build_map_client_{nullptr};
    rclcpp::Client<ReqAckSrvT>::SharedPtr save_map_client_{nullptr};
    rclcpp::Client<ReqAckSrvT>::SharedPtr localization_control_client_{nullptr};
    rclcpp::Client<ReqAckSrvT>::SharedPtr local_map_control_client_{nullptr};
    rclcpp::Client<ReqAckSrvT>::SharedPtr global_map_control_client_{nullptr};

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Shared map state.
    mutable std::mutex slam_map_mutex_;
    std::condition_variable slam_map_cv_;
    std::shared_ptr<OccupancyGridMsgT> latest_slam_map_{nullptr};
    std::size_t slam_map_update_count_{0};

    // Frontier suppression state.
    mutable std::mutex suppressed_frontiers_mutex_;
    std::vector<SuppressedFrontier> suppressed_frontiers_;

    // Background execution state.
    std::atomic_bool exploration_active_{false};
    uint32_t explored_frontiers_count_{0};

    std::unique_ptr<std::thread> loop_thread_;
};

} // namespace rf_scheduler
