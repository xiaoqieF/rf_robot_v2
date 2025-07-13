#include <cstring>
#include <cmath>

#include "rf_costmap/costmap_2d.hpp"
#include "rf_costmap/cost_values.hpp"

namespace rf_costmap
{

Costmap2D::Costmap2D()
    : size_x_(0), size_y_(0), resolution_(0.0),
      origin_x_(0.0), origin_y_(0.0), default_value_(FREE_SPACE), costmap_data_(nullptr)
{
}

Costmap2D::Costmap2D(const nav_msgs::msg::OccupancyGrid &grid)
    : default_value_(FREE_SPACE)
{
    size_x_ = grid.info.width;
    size_y_ = grid.info.height;
    resolution_ = grid.info.resolution;
    origin_x_ = grid.info.origin.position.x;
    origin_y_ = grid.info.origin.position.y;

    costmap_data_ = std::make_unique<uint8_t[]>(size_x_ * size_y_);

    for (unsigned int i = 0; i < size_x_ * size_y_; ++i) {
        if (grid.data[i] == OCC_GRID_UNKNOWN) {
            costmap_data_[i] = NO_INFORMATION;
        } else {
            costmap_data_[i] = std::round(
                static_cast<double>(grid.data[i]) * (LETHAL_OBSTACLE - FREE_SPACE) / (OCC_GRID_OCCUPIED - OCC_GRID_FREE));
        }
    }
}

Costmap2D::Costmap2D(unsigned int size_x, unsigned int size_y, double resolution,
    double origin_x, double origin_y, uint8_t default_value)
    : size_x_(size_x), size_y_(size_y), resolution_(resolution),
      origin_x_(origin_x), origin_y_(origin_y), default_value_(default_value)
{
    costmap_data_ = std::make_unique<uint8_t[]>(size_x_ * size_y_);
    memset(costmap_data_.get(), default_value_, size_x_ * size_y_);
}

Costmap2D::Costmap2D(const Costmap2D &other)
{
    *this = other;
}

Costmap2D& Costmap2D::operator=(const Costmap2D &other)
{
    if (this != &other) {
        size_x_ = other.size_x_;
        size_y_ = other.size_y_;
        resolution_ = other.resolution_;
        origin_x_ = other.origin_x_;
        origin_y_ = other.origin_y_;
        default_value_ = other.default_value_;
        costmap_data_ = std::make_unique<uint8_t[]>(size_x_ * size_y_);
        std::copy(other.costmap_data_.get(), other.costmap_data_.get() + (size_x_ * size_y_), costmap_data_.get());
    }
    return *this;
}

}