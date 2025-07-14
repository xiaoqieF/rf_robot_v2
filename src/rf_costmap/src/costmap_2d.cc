#include <cstdint>
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

void Costmap2D::copyMapRegion(uint8_t* src, unsigned int src_lower_left_x, unsigned int src_lower_left_y,
    unsigned int src_size_x, uint8_t* dest, unsigned int dest_lower_left_x, unsigned int dest_lower_left_y,
    unsigned int dest_size_x, unsigned int region_size_x, unsigned int region_size_y)
{
    uint8_t* src_ptr = src + src_lower_left_y * src_size_x + src_lower_left_x;
    uint8_t* dest_ptr = dest + dest_lower_left_y * dest_size_x + dest_lower_left_x;

    for (unsigned int y = 0; y < region_size_y; ++y) {
        std::memcpy(dest_ptr, src_ptr, region_size_x);
        src_ptr += src_size_x;
        dest_ptr += dest_size_x;
    }
}

uint8_t Costmap2D::getCost(unsigned int mx, unsigned int my) const
{
    return costmap_data_[getIndex(mx, my)];
}

void Costmap2D::setCost(unsigned int mx, unsigned int my, uint8_t cost)
{
    costmap_data_[getIndex(mx, my)] = cost;
}

void Costmap2D::mapToWorld(unsigned int mx, unsigned int my, double &wx, double &wy) const
{
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
}

bool Costmap2D::worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const
{
    if (wx < origin_x_ || wy < origin_y_) {
        return false; // Point is out of bounds
    }

    mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
    my = static_cast<unsigned int>((wy - origin_y_) / resolution_);
    if (mx >= size_x_ || my >= size_y_) {
        return false; // Point is out of bounds
    }
    return true;
}

void Costmap2D::updateOrigin(double new_origin_x, double new_origin_y)
{
    int cell_ox = static_cast<int>((new_origin_x - origin_x_) / resolution_);
    int cell_oy = static_cast<int>((new_origin_y - origin_y_) / resolution_);

    // Calculate the overlap region
    int size_x = static_cast<int>(size_x_);
    int size_y = static_cast<int>(size_y_);

    int lower_left_x = std::min(std::max(0, cell_ox), size_x);
    int lower_left_y = std::min(std::max(0, cell_oy), size_y);
    int upper_right_x = std::min(std::max(0, cell_ox + size_x), size_x);
    int upper_right_y = std::min(std::max(0, cell_oy + size_y), size_y);

    unsigned int new_size_x = upper_right_x - lower_left_x;
    unsigned int new_size_y = upper_right_y - lower_left_y;

    // Keep the overlap region in a temporary buffer
    std::unique_ptr<uint8_t[]> overlap = std::make_unique<uint8_t[]>(new_size_x * new_size_y);
    copyMapRegion(costmap_data_.get(), lower_left_x, lower_left_y, size_x_,
                  overlap.get(), 0, 0, new_size_x, new_size_x, new_size_y);

    memset(costmap_data_.get(), default_value_, size_x_ * size_y_);

    // Cell aligned
    origin_x_ = origin_x_ + cell_ox * resolution_;
    origin_y_ = origin_y_ + cell_oy * resolution_;

    int start_x = lower_left_x - cell_ox;
    int start_y = lower_left_y - cell_oy;

    // Copy the overlap region back to the costmap
    copyMapRegion(overlap.get(), 0, 0, new_size_x, costmap_data_.get(),
    start_x, start_y, size_x_, new_size_x, new_size_y);
}

}