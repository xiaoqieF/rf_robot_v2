#pragma once

#include "rf_costmap/layer.hpp"
#include "rf_costmap/costmap_2d.hpp"
#include <memory>

namespace rf_costmap
{

class CostmapLayer : public Layer
{
public:
    CostmapLayer() : Layer(),
      costmap_(std::make_unique<Costmap2D>()) {}
    virtual ~CostmapLayer() override = default;

    virtual void matchSize() override;

protected:
    void touch(double x, double y, double* min_x, double* min_y,
               double* max_x, double* max_y);
    void updateWithTrueOverwrite(Costmap2D& master_grid,
            unsigned int min_i, unsigned int min_j,
            unsigned int max_i, unsigned int max_j);
    void updateWithOverwrite(Costmap2D& master_grid,
            unsigned int min_i, unsigned int min_j,
            unsigned int max_i, unsigned int max_j);
    void updateWithMax(Costmap2D& master_grid,
            unsigned int min_i, unsigned int min_j,
            unsigned int max_i, unsigned int max_j);

protected:
    std::unique_ptr<Costmap2D> costmap_;
};

} // namespace rf_costmap
