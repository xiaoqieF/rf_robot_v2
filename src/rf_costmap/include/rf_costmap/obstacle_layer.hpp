#pragma once

#include "rf_costmap/costmap_layer.hpp"
#include "rf_costmap/cost_values.hpp"
#include "rf_costmap/observation_buffer.hpp"

#include "laser_geometry/laser_geometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "message_filters/subscriber.h"
#include <tf2_ros/message_filter.h>

#include <memory>

namespace rf_costmap
{
class ObstacleLayer : public CostmapLayer
{
public:
    ObstacleLayer() : CostmapLayer() {}
    virtual ~ObstacleLayer() = default;

    void reset() override;

    void updateBounds(double robot_x, double robot_y, double robot_yaw,
                      double* min_x, double* min_y, double* max_x, double* max_y) override;
    void updateCosts(Costmap2D& master_grid,
                     unsigned int min_i, unsigned int min_j,
                     unsigned int max_i, unsigned int max_j) override;

protected:
    void onInitialize() override;
    void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

private:
    void raytraceFreespace(const Observation& obs, double* min_x, double* min_y, double* max_x, double* max_y);

private:
    laser_geometry::LaserProjection projector_;
    std::unique_ptr<ObservationBuffer> observation_buffer_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> laser_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> tf_filter_;
};
} // namespace rf_costmap