#include "rf_costmap/costmap_layer.hpp"
#include "rf_costmap/cost_values.hpp"
#include "rf_costmap/master_costmap.hpp"
#include <cstdint>

namespace rf_costmap
{

void CostmapLayer::matchSize()
{
    Costmap2D* costmap = master_costmap_->getCostmap();
    costmap_->resizeMap(costmap->getSizeX(), costmap->getSizeY(),
    costmap->getResolution(), costmap->getOriginX(), costmap->getOriginY());
}

void CostmapLayer::touch(double x, double y, double* min_x, double* min_y,
            double* max_x, double* max_y)
{
    *min_x = std::min(*min_x, x);
    *min_y = std::min(*min_y, y);
    *max_x = std::max(*max_x, x);
    *max_y = std::max(*max_y, y);
}

void CostmapLayer::updateWithTrueOverwrite(Costmap2D& master_grid,
        unsigned int min_i, unsigned int min_j,
        unsigned int max_i, unsigned int max_j)
{
    if (!costmap_) {
        return; // No costmap to update
    }

    for (auto j = min_j; j <= max_j; ++j) {
        for (auto i = min_i; i <= max_i; ++i) {
            if (i >= costmap_->getSizeX() || j >= costmap_->getSizeY()) {
                continue; // Out of bounds
            }
            uint8_t cost = costmap_->getCost(i, j);
            master_grid.setCost(i, j, cost);
        }
    }
}

void CostmapLayer::updateWithOverwrite(Costmap2D& master_grid,
        unsigned int min_i, unsigned int min_j,
        unsigned int max_i, unsigned int max_j)
{
    if (!costmap_) {
        return; // No costmap to update
    }
    for (auto j = min_j; j <= max_j; ++j) {
        for (auto i = min_i; i <= max_i; ++i) {
            if (i >= costmap_->getSizeX() || j >= costmap_->getSizeY()) {
                continue; // Out of bounds
            }
            uint8_t cost = costmap_->getCost(i, j);
            if (cost == NO_INFORMATION) {
                continue; // Skip if no information
            }
            master_grid.setCost(i, j, cost);
        }
    }
}

void CostmapLayer::updateWithMax(Costmap2D& master_grid,
        unsigned int min_i, unsigned int min_j,
        unsigned int max_i, unsigned int max_j)
{
    if (!costmap_) {
        return; // No costmap to update
    }

    for (auto j = min_j; j <= max_j; ++j) {
        for (auto i = min_i; i <= max_i; ++i) {
            if (i >= costmap_->getSizeX() || j >= costmap_->getSizeY()) {
                continue; // Out of bounds
            }
            uint8_t cost = costmap_->getCost(i, j);
            if (cost == NO_INFORMATION) {
                continue; // Skip if no information
            }
            uint8_t current_cost = master_grid.getCost(i, j);
            if (current_cost == NO_INFORMATION || cost > current_cost) {
                master_grid.setCost(i, j, cost);
            }
        }
    }
}

} // namespace rf_costmap