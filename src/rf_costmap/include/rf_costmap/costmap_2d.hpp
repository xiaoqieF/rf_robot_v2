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

    void resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn);
    uint8_t getCost(unsigned int mx, unsigned int my) const;
    void setCost(unsigned int mx, unsigned int my, uint8_t cost);
    void mapToWorld(unsigned int mx, unsigned int my, double &wx, double &wy) const;
    bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const;
    void worldToMapEnforceBounds(double wx, double wy, unsigned int &mx, unsigned int &my) const;
    void updateOrigin(double origin_x, double origin_y);
    void resizeMap(unsigned int size_x, unsigned int size_y,
        double resolution, double origin_x, double origin_y);
    void raytraceLine(unsigned int mx0, unsigned int my0, unsigned int mx1, unsigned int my1, uint8_t value);

    inline unsigned int getIndex(unsigned int mx, unsigned int my) const
    {
        return my * size_x_ + mx;
    }

    uint8_t* getCostmapData() const
    {
        return costmap_data_.get();
    }

    unsigned int getSizeX() const { return size_x_; }
    unsigned int getSizeY() const { return size_y_; }
    double getResolution() const { return resolution_; }
    double getOriginX() const { return origin_x_; }
    double getOriginY() const { return origin_y_; }
    void setDefaultValue(uint8_t value) { default_value_ = value; }
    uint8_t getDefaultValue() const { return default_value_; }

private:
    void copyMapRegion(uint8_t* src, unsigned int src_lower_left_x, unsigned int src_lower_left_y,
        unsigned int src_size_x, uint8_t* dest, unsigned int dest_lower_left_x, unsigned int dest_lower_left_y,
        unsigned int dest_size_x, unsigned int region_size_x, unsigned int region_size_y);

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