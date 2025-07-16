#include "rf_costmap/master_costmap.hpp"
#include "rf_costmap/layer.hpp"
#include "elog/elog.h"

namespace rf_costmap
{

MasterCostmap::MasterCostmap(bool rolling_window)
    : rolling_window_(rolling_window)
{
    combined_costmap_ = std::make_unique<Costmap2D>();
}

MasterCostmap::~MasterCostmap()
{
    layers_.clear();
}

void MasterCostmap::addLayer(std::shared_ptr<Layer> layer)
{
    layers_.push_back(layer);
}
void MasterCostmap::resizeMap(unsigned int size_x, unsigned int size_y,
    double resolution, double origin_x, double origin_y)
{
    combined_costmap_->resizeMap(size_x, size_y, resolution, origin_x, origin_y);
    for (const auto& layer : layers_) {
        layer->matchSize();
    }
}

void MasterCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)
{
    if (rolling_window_) {
        // Update the origin of the costmap based on the robot's position
        double new_origin_x = robot_x - combined_costmap_->getSizeX() * combined_costmap_->getResolution() / 2.0;
        double new_origin_y = robot_y - combined_costmap_->getSizeY() * combined_costmap_->getResolution() / 2.0;
        combined_costmap_->updateOrigin(new_origin_x, new_origin_y);
    }

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    for (auto& layer : layers_) {
        double prev_min_x = min_x;
        double prev_min_y = min_y;
        double prev_max_x = max_x;
        double prev_max_y = max_y;
        layer->updateBounds(robot_x, robot_y, robot_yaw, &min_x, &min_y, &max_x, &max_y);
        if (min_x > prev_min_x || min_y > prev_min_y || max_x < prev_max_x || max_y < prev_max_y) {
            elog::warn("Layer {} bounds are not consistent with previous bounds.", layer->getName());
        }
    }

    unsigned int x0, xn, y0, yn;
    combined_costmap_->worldToMapEnforceBounds(min_x, min_y, x0, y0);
    combined_costmap_->worldToMapEnforceBounds(max_x, max_y, xn, yn);

    combined_costmap_->resetMap(x0, y0, xn, yn);
    for (auto& layer : layers_) {
        layer->updateCosts(*combined_costmap_, x0, y0, xn, yn);
    }
}

} // namespace rf_costmap