#pragma once

#include "rf_costmap/layer.hpp"
#include <cstdint>
#include <vector>
namespace rf_costmap
{

class CellData
{
public:
    CellData(unsigned int x, unsigned int y, unsigned int src_x, unsigned int src_y)
        : x(x), y(y), src_x(src_x), src_y(src_y) {}

    unsigned int x, y;
    unsigned int src_x, src_y;
};

class InflationLayer : public Layer
{
public:
    InflationLayer();
    ~InflationLayer() = default;

    void reset() override;
    void matchSize() override;
    void updateBounds(double robot_x, double robot_y, double robot_yaw,
                      double* min_x, double* min_y, double* max_x, double* max_y) override;
    void updateCosts(Costmap2D& master_grid,
                     unsigned int min_i, unsigned int min_j,
                     unsigned int max_i, unsigned int max_j) override;

    uint8_t computeCost(double distance) const;

protected:
    void onInitialize() override;

private:
    void computeCaches();
    int computeLevelTable();
    uint8_t costLookup(unsigned int mx, unsigned int my,
        unsigned int src_x, unsigned int src_y) const;
    double distanceLookup(unsigned int mx, unsigned int my,
        unsigned int src_x, unsigned int src_y) const;
    void enqueue(unsigned int index, unsigned int mx, unsigned int my,
                 unsigned int src_x, unsigned int src_y);

private:
    const double inflation_radius{0.35};
    const double inscribed_radius{0.12};
    const double cost_scaling_factor{3.0};

    double last_min_x_{std::numeric_limits<double>::lowest()};
    double last_min_y_{std::numeric_limits<double>::lowest()};
    double last_max_x_{std::numeric_limits<double>::max()};
    double last_max_y_{std::numeric_limits<double>::max()};

    unsigned int cell_inflation_radius_;
    unsigned int cache_length_;
    std::vector<std::vector<CellData>> inflation_cells_;   // we use array instead of priority queue
    std::vector<bool> seen_;
    std::vector<unsigned char> cached_costs_;
    std::vector<double> cached_distances_;
    std::vector<std::vector<int>> level_table_;
};

} // namespace rf_costmap