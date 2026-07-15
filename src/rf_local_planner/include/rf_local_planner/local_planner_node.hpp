#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rf_robot_msgs/action/follow_path.hpp"
#include "rf_robot_msgs/msg/costmap.hpp"
#include "rf_robot_msgs/srv/req_ack.hpp"
#include "rf_util/simple_action_server.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include <memory>
#include <limits>
#include <mutex>
#include <optional>
#include <vector>

namespace rf_local_planner
{

using CostmapMsgT = rf_robot_msgs::msg::Costmap;
using PathMsgT = nav_msgs::msg::Path;
using OdomMsgT = nav_msgs::msg::Odometry;
using TwistMsgT = geometry_msgs::msg::Twist;
using ReqAckSrvT = rf_robot_msgs::srv::ReqAck;
using ActionFollowPath = rf_robot_msgs::action::FollowPath;

class LocalPlannerNode : public rclcpp::Node
{
public:
    LocalPlannerNode();

    void init();

private:
    struct Options
    {
        // 控制循环频率（Hz）。调大能更快响应环境变化，但会增加轨迹采样和 CPU 开销。
        double control_frequency = 10.0;
        // 每条候选轨迹的前向预测时长（秒）。调大更早发现远处风险，但在狭窄区域可能过于保守。
        double sim_time = 1.0;
        // 轨迹仿真的积分步长（秒），实际不小于 0.02 秒。调小可提高碰撞检测精度，但会增加计算量。
        double sim_granularity = 0.05;
        // 线速度采样数。调大可找到更细致的速度解，但计算量按采样数线性增加。
        int vx_samples = 8;
        // 角速度采样数。调大可提高转向轨迹的选择精度，但计算量按采样数线性增加。
        int vtheta_samples = 12;
        // 最大前进线速度（m/s）。调大可提高巡航速度，但需要与机器人制动能力和局部地图范围匹配。
        double max_vel_x = 0.5;
        // 最小前进线速度（m/s）。设为正值可避免低速爬行，但可能降低贴近目标或窄区时的可控性。
        double min_vel_x = 0.0;
        // 最大角速度（rad/s）。调大可更快转弯，但可能造成轨迹抖动或底盘跟踪困难。
        double max_vel_theta = 2.0;
        // 最小角速度（rad/s）。设为负值允许反向旋转，其绝对值决定最大反向转速。
        double min_vel_theta = -2.0;
        // 线加速度限制（m/s²），用于从当前里程计速度生成动态窗口。调大响应更快，但须符合底盘能力。
        double acc_lim_x = 3.0;
        // 角加速度限制（rad/s²），用于从当前里程计角速度生成动态窗口。调大转向响应更快，但易产生突变。
        double acc_lim_theta = 10.0;
        // 偏离全局路径的惩罚权重。调大更严格贴合路径，调小则更愿意绕开局部障碍。
        double path_distance_bias = 0.5;
        // 轨迹终点到局部目标距离的惩罚权重。调大更积极接近局部目标，过大可能削弱避障和路径跟随。
        double goal_distance_bias = 0.5;
        // 轨迹平均代价地图代价的惩罚权重。调大使轨迹远离障碍和膨胀区，调小会允许更贴近障碍通行。
        double obstacle_cost_bias = 5.0;
        // 沿全局路径前进距离的奖励权重。调大可减少原地犹豫，过大可能偏好快速推进而忽略平滑性。
        double progress_bias = 15.0;
        // 低线速度的惩罚权重。调大更偏好高速轨迹，调小更容易选择慢速或停下的轨迹。
        double speed_bias = 0.5;
        // 轨迹末端朝向局部目标的误差惩罚权重。调大更偏好朝向正确的轨迹，可改善入弯但可能增加转向。
        double heading_bias = 0.5;
        // 局部目标在全局路径上的前视距离（米）。调大更平滑且更快，调小更贴合路径并适合急弯或狭窄区域。
        double local_goal_distance = 0.3;
        // 终点位置容差（米）。调大更容易判定到达，调小可提高最终停车位置精度。
        double goal_tolerance_xy = 0.15;
        // 终点朝向容差（rad）；仅在 enforce_goal_heading 为 true 时生效。调大可更快完成，调小可提高朝向精度。
        double goal_tolerance_yaw = 0.10;
        // 是否到达位置后继续原地旋转至终点朝向；开启后任务完成姿态更准确，但可能增加完成时间。
        bool enforce_goal_heading = false;
        // 连续找不到无碰撞控制量的最大周期数。调大可容忍瞬时地图波动，调小可更快报告不可通行。
        int max_no_control_cycles = 20;
        // 目标距离未出现足够改善时的超时（秒）。调大可容忍缓慢绕行，调小可更快检测卡死。
        double no_progress_timeout = 6.0;
        // 重置无进展计时器所需的最小靠近终点距离（米）。调大可避免把微小抖动视为进展，过大可能误判低速运动为卡死。
        double min_progress_distance = 0.10;
        // 是否发布全部采样轨迹的调试 Marker。关闭可降低可视化消息和 CPU 开销，不影响控制决策。
        bool publish_debug_trajectories = true;
    };

    struct Pose2D
    {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

    enum class CostmapCellStatus
    {
        TRAVERSABLE,
        INSCRIBED,
        LETHAL,
        UNKNOWN,
        OUT_OF_BOUNDS,
    };

    struct TrajectorySample
    {
        double linear_vel = 0.0;
        double angular_vel = 0.0;
        double total_cost = std::numeric_limits<double>::infinity();
        double path_cost = 0.0;
        double goal_cost = 0.0;
        double obstacle_cost = 0.0;
        double obstacle_mean_cost = 0.0;
        double obstacle_peak_cost = 0.0;
        double speed_cost = 0.0;
        double heading_cost = 0.0;
        double progress_reward = 0.0;
        bool collision = true;
        std::vector<Pose2D> poses;
    };

private:
    void handleLocalCostmap(const CostmapMsgT::SharedPtr msg);
    void handleOdometry(const OdomMsgT::SharedPtr msg);
    void handleFollowPath();
    void publishZeroVelocity();
    void publishTrajectory(const TrajectorySample& sample, const std::string& frame_id);
    void publishDebugTrajectories(
        const std::vector<TrajectorySample>& samples,
        const TrajectorySample* best,
        const std::string& frame_id);
    void clearDebugTrajectories();
    void publishRotateToGoal(double yaw_error);
    void publishInscribedRecovery();
    bool isOdomVelocityValid(const OdomMsgT& odom, const Options& options) const;
    Options getOptions() const;
    rcl_interfaces::msg::SetParametersResult handleParameterUpdate(
        const std::vector<rclcpp::Parameter>& parameters);

    bool getRobotPose(Pose2D& pose);
    std::vector<geometry_msgs::msg::PoseStamped> prunePlan(
        const PathMsgT& plan, const Pose2D& robot_pose) const;
    geometry_msgs::msg::PoseStamped selectLocalGoal(
        const std::vector<geometry_msgs::msg::PoseStamped>& plan_segment) const;
    TrajectorySample findBestTrajectory(
        const Pose2D& robot_pose,
        const OdomMsgT& odom,
        const CostmapMsgT& costmap,
        const std::vector<geometry_msgs::msg::PoseStamped>& plan_segment,
        const geometry_msgs::msg::PoseStamped& local_goal,
        std::vector<TrajectorySample>* debug_samples = nullptr) const;
    TrajectorySample simulateTrajectory(
        const Pose2D& start_pose,
        double linear_vel,
        double angular_vel,
        const CostmapMsgT& costmap,
        const std::vector<geometry_msgs::msg::PoseStamped>& plan_segment,
        const geometry_msgs::msg::PoseStamped& local_goal) const;

    double computePathDistance(
        const Pose2D& pose,
        const std::vector<geometry_msgs::msg::PoseStamped>& plan_segment) const;
    double computePathProgress(
        const Pose2D& pose,
        const std::vector<geometry_msgs::msg::PoseStamped>& plan_segment) const;
    CostmapCellStatus getCostmapCellStatus(
        const Pose2D& pose, const CostmapMsgT& costmap, uint8_t* cost = nullptr) const;
    std::optional<double> findReverseRecoveryDistance(
        const Pose2D& pose, const CostmapMsgT& costmap) const;
    bool isCollision(const Pose2D& pose, const CostmapMsgT& costmap, double* max_cost = nullptr) const;
    bool worldToMap(
        const CostmapMsgT& costmap, double wx, double wy, unsigned int& mx, unsigned int& my) const;
    double normalizeAngle(double angle) const;
    double computeGoalYaw(const geometry_msgs::msg::PoseStamped& pose) const;

private:
    mutable std::mutex options_mutex_;
    Options options_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    rclcpp::Subscription<CostmapMsgT>::SharedPtr local_costmap_sub_;
    rclcpp::Subscription<OdomMsgT>::SharedPtr odom_sub_;
    rclcpp::Publisher<TwistMsgT>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<PathMsgT>::SharedPtr local_traj_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_traj_pub_;
    rf_util::SimpleActionServer<ActionFollowPath>::UniquePtr follow_path_server_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    mutable std::mutex data_mutex_;
    CostmapMsgT::SharedPtr local_costmap_msg_;
    OdomMsgT::SharedPtr odom_msg_;
    bool zero_cmd_published_ = false;
    bool debug_markers_active_ = false;
};

} // namespace rf_local_planner
