#pragma once

#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
namespace rf_costmap
{

class Observation
{
public:
    Observation()
        : origin(), cloud(nullptr),
          obstacle_max_range(0.0),
          obstacle_min_range(0.0),
          raytrace_max_range(0.0),
          raytrace_min_range(0.0) {}

    ~Observation() = default;

    Observation(geometry_msgs::msg::Point& origin, const sensor_msgs::msg::PointCloud2& cloud,
        double obstacle_max_range, double obstacle_min_range, double raytrace_max_range, double raytrace_min_range)
        : origin(origin), cloud(std::make_unique<sensor_msgs::msg::PointCloud2>(cloud)),
          obstacle_max_range(obstacle_max_range),
          obstacle_min_range(obstacle_min_range),
          raytrace_max_range(raytrace_max_range),
          raytrace_min_range(raytrace_min_range) {}

    Observation(const Observation& other)
    {
        *this = other;
    }

    Observation& operator=(const Observation& other)
    {
        if (this != &other) {
            origin = other.origin;
            cloud = std::make_unique<sensor_msgs::msg::PointCloud2>(*other.cloud);
            obstacle_max_range = other.obstacle_max_range;
            obstacle_min_range = other.obstacle_min_range;
            raytrace_max_range = other.raytrace_max_range;
            raytrace_min_range = other.raytrace_min_range;
        }
        return *this;
    }

    Observation& operator=(Observation&& other)
    {
        if (this != &other) {
            origin = std::move(other.origin);
            cloud = std::move(other.cloud);
            obstacle_max_range = other.obstacle_max_range;
            obstacle_min_range = other.obstacle_min_range;
            raytrace_max_range = other.raytrace_max_range;
            raytrace_min_range = other.raytrace_min_range;
        }
        return *this;
    }

    Observation(Observation&& other)
    {
        *this = std::move(other);
    }

public:
    geometry_msgs::msg::Point origin;
    std::unique_ptr<sensor_msgs::msg::PointCloud2> cloud;
    double obstacle_max_range;
    double obstacle_min_range;
    double raytrace_max_range;
    double raytrace_min_range;
};

} // namespace rf_costmap