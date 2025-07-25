#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rf_costmap/static_layer.hpp"
#include "rf_costmap/cost_values.hpp"
#include "rf_costmap/master_costmap.hpp"
#include <tf2/LinearMath/Transform.hpp>

namespace rf_costmap
{

void StaticLayer::onInitialize()
{
    map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", rclcpp::QoS(1),
        std::bind(&StaticLayer::mapCallback, this, std::placeholders::_1));
}

void StaticLayer::reset()
{
    map_received_ = false;
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_buffer_.reset();
}

void StaticLayer::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_received_ = true;
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_buffer_ = msg;
}

void StaticLayer::matchSize()
{
    // We do not resize the static layer when rolling window is enabled
    // because the static map size is fixed.
    if (!master_costmap_->isRollingWindow()) {
        Costmap2D* costmap = master_costmap_->getCostmap();
        costmap_->resizeMap(costmap->getSizeX(), costmap->getSizeY(),
            costmap->getResolution(), costmap->getOriginX(), costmap->getOriginY());
    }
}

uint8_t StaticLayer::interpretCostValue(uint8_t cost) const
{
    // Use trinary logic to interpret the cost value
    if (cost == UNKNOWN_COST_VALUE) {
        return NO_INFORMATION;
    } else if (cost >= LETHAL_THRESHOLD) {
        return LETHAL_OBSTACLE;
    } else {
        return FREE_SPACE;
    }
}

void StaticLayer::processMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg)
{
    auto size_x = msg->info.width;
    auto size_y = msg->info.height;
    COSTMAP_INFO("Processing static map with size: {}x{}", size_x, size_y);

    if (!master_costmap_->isRollingWindow()) {
        auto master = master_costmap_->getCostmap();
        if (master->getSizeX() != size_x || master->getSizeY() != size_y
            || master->getResolution() != msg->info.resolution
            || master->getOriginX() != msg->info.origin.position.x
            || master->getOriginY() != msg->info.origin.position.y) {
            COSTMAP_WARN("Static map size or resolution has changed, resizing master costmap.");
            // Resize master costmap will call all layers' matchSize
            master_costmap_->resizeMap(size_x, size_y, msg->info.resolution,
                msg->info.origin.position.x, msg->info.origin.position.y);
        }
    } else {
        // If rolling window is enabled, we only resize the static layer
        if (costmap_->getSizeX() != size_x || costmap_->getSizeY() != size_y
            || costmap_->getResolution() != msg->info.resolution
            || costmap_->getOriginX() != msg->info.origin.position.x
            || costmap_->getOriginY() != msg->info.origin.position.y) {
            COSTMAP_WARN("Static map size or resolution has changed, resizing static layer.");
            costmap_->resizeMap(size_x, size_y, msg->info.resolution,
                msg->info.origin.position.x, msg->info.origin.position.y);
        }
    }

    unsigned int index = 0;
    auto costmap = costmap_->getCostmapData();
    for (unsigned int j = 0; j < size_y; ++j) {
        for (unsigned int i = 0; i < size_x; ++i) {
            uint8_t cost = msg->data[index];
            costmap[index] = interpretCostValue(cost);
            ++ index;
        }
    }
}

void StaticLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y)
{
    (void) robot_x; // Unused parameters
    (void) robot_y;
    (void) robot_yaw;

    if (!map_received_) {
        return; // No map received yet
    }
    bool has_new_map = false;
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (map_buffer_) {
        processMap(map_buffer_);
        map_buffer_ = nullptr;
        has_new_map = true;
    }
    if (!master_costmap_->isRollingWindow() && !has_new_map) {
        // If rolling window is disabled, we do not update bounds
        // because the static map size is fixed.
        return;
    }

    // Set bounds based on the costmap size
    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    *min_x = std::min(*min_x, wx);
    *min_y = std::min(*min_y, wy);

    costmap_->mapToWorld(costmap_->getSizeX() - 1, costmap_->getSizeY() - 1, wx, wy);
    *max_x = std::max(*max_x, wx);
    *max_y = std::max(*max_y, wy);

    has_new_map = false;
}

void StaticLayer::updateCosts(Costmap2D& master_grid,
                    unsigned int min_i, unsigned int min_j,
                    unsigned int max_i, unsigned int max_j)
{
    if (!map_received_) {
        return; // No map received yet
    }
    if (!master_costmap_->isRollingWindow()) {
        updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
    } else {
        // If rolling window, master_grid is unlikely to have the same size as costmap_,
        for (auto i = min_i; i <= max_i; ++i) {
            for (auto j = min_j; j <= max_j; ++j) {
                double wx, wy;
                unsigned int mx, my;
                master_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
                if (costmap_->worldToMap(wx, wy, mx, my)) {
                    master_grid.setCost(mx, my, costmap_->getCost(i, j));
                }
            }
        }
    }
}

} // namespace rf_costmap