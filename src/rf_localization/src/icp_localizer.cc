#include "rf_localization/icp_localizer.hpp"

#include "pcl/common/transforms.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>

namespace rf_localization
{

IcpLocalizer::IcpLocalizer(Options options)
    : options_(std::move(options))
{
    icp_.setMaxCorrespondenceDistance(options_.max_correspondence_distance);
    icp_.setMaximumIterations(options_.max_iterations);
    icp_.setTransformationEpsilon(options_.transformation_epsilon);
    icp_.setEuclideanFitnessEpsilon(options_.fitness_epsilon);
}

bool IcpLocalizer::setMap(const nav_msgs::msg::OccupancyGrid& map)
{
    // 将地图转换为点云，并应用体素滤波器以减少点的数量，从而提高ICP的效率。
    auto raw_map_cloud = buildMapPointCloud(map);
    if (raw_map_cloud->empty()) {
        return false;
    }

    map_cloud_ = voxelFilter(raw_map_cloud, options_.map_voxel_leaf_size);
    if (map_cloud_->empty()) {
        return false;
    }

    icp_.setInputTarget(map_cloud_);
    return true;
}

std::optional<LocalizationResult> IcpLocalizer::localize(const LocalizationInput& input)
{
    if (map_cloud_->empty() || input.scan_in_base == nullptr || input.scan_in_base->empty()) {
        return std::nullopt;
    }

    // 1. 对输入的激光扫描点云进行体素滤波，以减少点的数量，从而提高ICP的效率。
    auto filtered_scan = voxelFilter(input.scan_in_base, options_.scan_voxel_leaf_size);
    if (filtered_scan->size() < static_cast<size_t>(options_.min_scan_points)) {
        return std::nullopt;
    }

    icp_.setInputSource(filtered_scan);

    // 2. 使用ICP算法对滤波后的扫描点云与地图点云进行配准，得到在地图坐标系下的位姿估计。
    PointCloudT aligned_scan;
    icp_.align(aligned_scan, input.initial_guess);
    if (!icp_.hasConverged()) {
        return std::nullopt;
    }

    const double fitness_score = icp_.getFitnessScore();
    if (fitness_score > options_.fitness_score_threshold) {
        return std::nullopt;
    }

    // 3. 返回位姿估计结果，包括在地图坐标系下的位姿、配准后的点云以及ICP的拟合分数。
    LocalizationResult result;
    result.pose_in_map = normalizePlanarTransform(icp_.getFinalTransformation());
    result.fitness_score = fitness_score;
    pcl::transformPointCloud(*filtered_scan, *result.aligned_scan_in_map, result.pose_in_map);
    return result;
}

PointCloudT::Ptr IcpLocalizer::buildMapPointCloud(const nav_msgs::msg::OccupancyGrid& map) const
{
    auto cloud = std::make_shared<PointCloudT>();
    cloud->reserve(static_cast<size_t>(map.info.width) * map.info.height / 4U);

    tf2::Transform origin_transform;
    tf2::fromMsg(map.info.origin, origin_transform);

    const double resolution = map.info.resolution;
    for (uint32_t y = 0; y < map.info.height; ++y) {
        for (uint32_t x = 0; x < map.info.width; ++x) {
            const size_t index = static_cast<size_t>(y) * map.info.width + x;
            if (index >= map.data.size()) {
                continue;
            }
            if (map.data[index] < options_.map_occupied_threshold) {
                continue;
            }

            const tf2::Vector3 cell_center(
                (static_cast<double>(x) + 0.5) * resolution,
                (static_cast<double>(y) + 0.5) * resolution,
                0.0);
            const tf2::Vector3 world_point = origin_transform * cell_center;

            PointT point;
            point.x = static_cast<float>(world_point.x());
            point.y = static_cast<float>(world_point.y());
            point.z = 0.0F;
            cloud->push_back(point);
        }
    }

    return cloud;
}

PointCloudT::Ptr IcpLocalizer::voxelFilter(const PointCloudT::ConstPtr& cloud, float leaf_size) const
{
    if (cloud == nullptr) {
        return std::make_shared<PointCloudT>();
    }
    if (leaf_size <= 0.0F) {
        return std::make_shared<PointCloudT>(*cloud);
    }

    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.setInputCloud(cloud);

    auto filtered_cloud = std::make_shared<PointCloudT>();
    voxel_filter.filter(*filtered_cloud);
    return filtered_cloud;
}

// 将4x4变换矩阵归一化为平面变换矩阵，保留平移和旋转信息，忽略z轴的旋转和平移。
Eigen::Matrix4f IcpLocalizer::normalizePlanarTransform(const Eigen::Matrix4f& transform) const
{
    const float yaw = std::atan2(transform(1, 0), transform(0, 0));
    Eigen::Matrix4f planar_transform = Eigen::Matrix4f::Identity();
    planar_transform(0, 0) = std::cos(yaw);
    planar_transform(0, 1) = -std::sin(yaw);
    planar_transform(1, 0) = std::sin(yaw);
    planar_transform(1, 1) = std::cos(yaw);
    planar_transform(0, 3) = transform(0, 3);
    planar_transform(1, 3) = transform(1, 3);
    return planar_transform;
}

} // namespace rf_localization
