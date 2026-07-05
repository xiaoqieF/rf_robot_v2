#pragma once

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/time.hpp"

#include <Eigen/Core>
#include <memory>
#include <optional>
#include <string>

namespace rf_localization
{

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

struct LocalizationInput
{
    rclcpp::Time stamp;
    PointCloudT::ConstPtr scan_in_base;
    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
};

struct LocalizationResult
{
    Eigen::Matrix4f pose_in_map = Eigen::Matrix4f::Identity();
    PointCloudT::Ptr aligned_scan_in_map = std::make_shared<PointCloudT>();
    double fitness_score = 0.0;
};

class LocalizationMethod
{
public:
    virtual ~LocalizationMethod() = default;

    virtual std::string name() const = 0;
    virtual bool setMap(const nav_msgs::msg::OccupancyGrid& map) = 0;
    virtual std::optional<LocalizationResult> localize(const LocalizationInput& input) = 0;
};

} // namespace rf_localization
