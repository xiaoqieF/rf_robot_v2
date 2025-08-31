#pragma once

#include "rf_map_builder/config_options.hpp"
#include "rf_map_builder/msg_conversion.hpp"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/common/time.h"
#include <memory>
#include <rclcpp/time.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>

namespace rf_map_builder
{

class TfBridge
{
public:
    TfBridge(const std::string& tracking_frame,
        double lookup_transform_timeout_sec,
        const tf2_ros::Buffer* const tf_buffer)
        : tracking_frame_(tracking_frame),
          lookup_transform_timeout_sec_(lookup_transform_timeout_sec),
          tf_buffer_(tf_buffer) {}
    ~TfBridge() = default;

    TfBridge(const TfBridge&) = delete;
    TfBridge& operator=(const TfBridge&) = delete;

    std::unique_ptr<cartographer::transform::Rigid3d> lookupToTracking(
        cartographer::common::Time time, const std::string& frame_id) const
    {
        tf2::Duration timeout(tf2::durationFromSec(lookup_transform_timeout_sec_));
        std::unique_ptr<cartographer::transform::Rigid3d> frame_id_to_tracking;

        try {
            // First check if the latest available transform is newer than the requested time.
            const rclcpp::Time lastest_tf_time = tf_buffer_->lookupTransform(tracking_frame_, frame_id,
                rclcpp::Time(0), timeout).header.stamp;
            const rclcpp::Time requested_time = toRos(time);
            if (lastest_tf_time >= requested_time) {
                timeout = tf2::durationFromSec(0.0);
            }

            return std::make_unique<cartographer::transform::Rigid3d>(
                toRigid3d(tf_buffer_->lookupTransform(tracking_frame_, frame_id,
                    requested_time, timeout)));
        } catch (const tf2::TransformException& ex) {
            MAP_BUILDER_WARN("{}", ex.what());
        }
        return nullptr;
    }

private:
    const std::string tracking_frame_;
    const double lookup_transform_timeout_sec_;
    const tf2_ros::Buffer* const tf_buffer_;
};

} // namespace rf_map_builder