#include "rf_costmap/obstacle_layer.hpp"
#include "rf_costmap/cost_values.hpp"
#include "rf_costmap/master_costmap.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <memory>
#include <cmath>

namespace rf_costmap
{

void ObstacleLayer::onInitialize()
{
    observation_buffer_ = std::make_unique<ObservationBuffer>(
        node_, tf_buffer_
    );

    matchSize();

    auto custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = 50;

    laser_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
        node_, "/scan", custom_qos_profile
    );

    tf_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
        *laser_sub_, *tf_buffer_, "map", 50,
        node_->get_node_logging_interface(), node_->get_node_clock_interface(), tf2::durationFromSec(0.05)
    );

    tf_filter_->registerCallback(
        std::bind(&ObstacleLayer::laserCallback, this, std::placeholders::_1)
    );
}

void ObstacleLayer::laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = msg->header;

    try {
        projector_.transformLaserScanToPointCloud(msg->header.frame_id, *msg, cloud, *tf_buffer_);
    } catch (tf2::TransformException& ex) {
        COSTMAP_WARN("Failed to transform scan: %s", ex.what());
        projector_.projectLaser(*msg, cloud);
    } catch (std::runtime_error& ex) {
        COSTMAP_WARN("Failed to projectLaser scan: %s", ex.what());
        return;
    }

    observation_buffer_->bufferCloud(cloud);
}

void ObstacleLayer::reset()
{
    observation_buffer_->resetLastUpdated();
}

void ObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y)
{
    (void) robot_yaw;

    if (master_costmap_->isRollingWindow()) {
        double new_origin_x = robot_x - costmap_->getSizeX() * costmap_->getResolution() / 2.0;
        double new_origin_y = robot_y - costmap_->getSizeY() * costmap_->getResolution() / 2.0;
        costmap_->updateOrigin(new_origin_x, new_origin_y);
    }

    auto observations = observation_buffer_->getObservations();
    // raytrace freespace
    for (const auto& obs : observations) {
        raytraceFreespace(obs, min_x, min_y, max_x, max_y);
    }

    // put obstacles
    for (const auto& obs : observations) {
        double sq_obstacle_max_range = obs.obstacle_max_range * obs.obstacle_max_range;
        double sq_obstacle_min_range = obs.obstacle_min_range * obs.obstacle_min_range;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*obs.cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*obs.cloud, "y");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
            double wx = *iter_x;
            double wy = *iter_y;

            double sq_dist = (wx - obs.origin.x) * (wx - obs.origin.x) +
                (wy - obs.origin.y) * (wy - obs.origin.y);

            if (sq_dist < sq_obstacle_min_range || sq_dist > sq_obstacle_max_range) {
                continue;
            }

            unsigned int mx, my;
            if (costmap_->worldToMap(wx, wy, mx, my)) {
                costmap_->setCost(mx, my, LETHAL_OBSTACLE);
                touch(wx, wy, min_x, min_y, max_x, max_y);
            }
        }
    }
}

void ObstacleLayer::raytraceFreespace(const Observation& obs,
    double* min_x, double* min_y, double* max_x, double* max_y)
{
    double map_origin_x = costmap_->getOriginX();
    double map_origin_y = costmap_->getOriginY();
    double map_end_x = costmap_->getOriginX() + costmap_->getSizeX() * costmap_->getResolution();
    double map_end_y = costmap_->getOriginY() + costmap_->getSizeY() * costmap_->getResolution();

    double origin_x = obs.origin.x;
    double origin_y = obs.origin.y;

    unsigned int x0, y0;
    if (!costmap_->worldToMap(origin_x, origin_y, x0, y0)) {
        COSTMAP_WARN("Raytrace freespace origin ({.2f}, {.2f}) is out of bounds"
            "({.2f}, {.2f}) - ({.2f, .2f})"
            , origin_x, origin_y, map_origin_x, map_origin_y,
            map_end_x, map_end_y);
        return;
    }

    touch(origin_x, origin_y, min_x, min_y, max_x, max_y);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*obs.cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*obs.cloud, "y");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
        double wx = *iter_x;
        double wy = *iter_y;

        double a = wx - origin_x;
        double b = wy - origin_y;

        // To kepp the ray within the map
        if (wx < origin_x) {
            double t = (map_origin_x - origin_x) / a;
            wx = map_origin_x;
            wy = origin_y + t * b;
        }

        if (wy < origin_y) {
            double t = (map_origin_y - origin_y) / b;
            wx = origin_x + t * a;
            wy = map_origin_y;
        }

        if (wx > map_end_x) {
            double t = (map_end_x - origin_x) / a;
            wx = map_end_x - 0.001;
            wy = origin_y + t * b;
        }

        if (wy > map_end_y) {
            double t = (map_end_y - origin_y) / b;
            wx = origin_x + t * a;
            wy = map_end_y - 0.001;
        }

        // clip to raytrace range
        double full_x = wx - origin_x;
        double full_y = wy - origin_y;
        double full_dist = std::hypot(full_x, full_y);

        if (full_dist < obs.raytrace_min_range) {
            return;
        }

        unsigned final_x0 = x0;
        unsigned final_y0 = y0;

        if (obs.raytrace_min_range > 1e-3) {
            double t = obs.raytrace_min_range / full_dist;
            double final_origin_x = origin_x + t * full_x;
            double final_origin_y = origin_y + t * full_y;

            if (!costmap_->worldToMap(final_origin_x, final_origin_y, final_x0, final_y0)) {
                COSTMAP_WARN("Raytrace freespace min range point ({.2f}, {.2f}) is out of bounds"
                    "({.2f}, {.2f}) - ({.2f, .2f}), which should not happen."
                    , final_origin_x, final_origin_y, map_origin_x, map_origin_y,
                    map_end_x, map_end_y);
                continue;
            }
        }

        if (full_dist > obs.raytrace_max_range) {
            double t = obs.raytrace_max_range / full_dist;
            wx = origin_x + t * full_x;
            wy = origin_y + t * full_y;
        }

        unsigned int x1, y1;
        if (!costmap_->worldToMap(wx, wy, x1, y1)) {
            COSTMAP_WARN("Raytrace freespace point ({.2f}, {.2f}) is out of bounds"
                "({.2f}, {.2f}) - ({.2f, .2f}), which should not happen."
                , wx, wy, map_origin_x, map_origin_y,
                map_end_x, map_end_y);
            continue;
        }

        costmap_->raytraceLine(final_x0, final_y0, x1, y1, FREE_SPACE);
        touch(wx, wy, min_x, min_y, max_x, max_y);
    }
}

void ObstacleLayer::updateCosts(Costmap2D& master_grid,
                    unsigned int min_i, unsigned int min_j,
                    unsigned int max_i, unsigned int max_j)
{
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

} // namespace rf_costmap