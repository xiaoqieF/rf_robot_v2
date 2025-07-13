#pragma once

#include <sys/types.h>
#include <vector>
#include <string>
#include <cstdint>
#include <memory>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace rf_costmap
{

class Costmap2D
{
public:
    Costmap2D();
    explicit Costmap2D(const nav_msgs::msg::OccupancyGrid &grid);
    Costmap2D(unsigned int size_x, unsigned int size_y, double resolution,
        double origin_x, double origin_y, uint8_t default_value = 0);
    Costmap2D(const Costmap2D &other);
    Costmap2D& operator=(const Costmap2D &other);

    // Default move is ok
    Costmap2D(Costmap2D &&other) noexcept = default;
    Costmap2D& operator=(Costmap2D &&other) noexcept = default;

    ~Costmap2D() = default;

private:
    unsigned int size_x_;
    unsigned int size_y_;
    double resolution_;
    double origin_x_;
    double origin_y_;
    uint8_t default_value_;
    std::unique_ptr<uint8_t[]> costmap_data_;
};

} // namespace rf_costmap