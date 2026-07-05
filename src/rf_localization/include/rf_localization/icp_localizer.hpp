#pragma once

#include "rf_localization/localization_method.hpp"

#include "pcl/filters/voxel_grid.h"
#include "pcl/registration/icp.h"

#include <Eigen/Core>

namespace rf_localization
{

class IcpLocalizer : public LocalizationMethod
{
public:
    struct Options
    {
        int map_occupied_threshold = 65;
        int min_scan_points = 20;
        float map_voxel_leaf_size = 0.05F;
        float scan_voxel_leaf_size = 0.05F;
        float max_correspondence_distance = 0.5F;
        float transformation_epsilon = 1e-4F;
        float fitness_epsilon = 1e-3F;
        float fitness_score_threshold = 0.3F;
        int max_iterations = 50;
    };

    explicit IcpLocalizer(Options options);

    std::string name() const override { return "icp"; }
    bool setMap(const nav_msgs::msg::OccupancyGrid& map) override;
    std::optional<LocalizationResult> localize(const LocalizationInput& input) override;

private:
    PointCloudT::Ptr buildMapPointCloud(const nav_msgs::msg::OccupancyGrid& map) const;
    PointCloudT::Ptr voxelFilter(const PointCloudT::ConstPtr& cloud, float leaf_size) const;
    Eigen::Matrix4f normalizePlanarTransform(const Eigen::Matrix4f& transform) const;

private:
    Options options_;
    PointCloudT::Ptr map_cloud_ = std::make_shared<PointCloudT>();
    pcl::IterativeClosestPoint<PointT, PointT> icp_;
};

} // namespace rf_localization
