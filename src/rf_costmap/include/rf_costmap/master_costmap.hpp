#pragma once

#include "rf_costmap/costmap_2d.hpp"
#include "rf_costmap/layer.hpp"
namespace rf_costmap
{

class MasterCostmap
{
public:
    MasterCostmap(bool rolling_window);
    ~MasterCostmap();

    Costmap2D* getCostmap() { return combined_costmap_.get(); }
    bool isRollingWindow() const { return rolling_window_; }

    void addLayer(std::shared_ptr<Layer> layer);
    void updateMap(double robot_x, double robot_y, double robot_yaw);
    void resizeMap(unsigned int size_x, unsigned int size_y,
        double resolution, double origin_x, double origin_y);

private:
    std::unique_ptr<Costmap2D> combined_costmap_;
    std::vector<std::shared_ptr<Layer>> layers_;
    bool rolling_window_;
};

} // namespace rf_costmap