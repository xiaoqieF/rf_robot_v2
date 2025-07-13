#include "elog/elog.h"
#include "rf_util/robot_utils.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/time.hpp>

namespace rf_util
{
bool getCurrentPose(
    geometry_msgs::msg::PoseStamped& global_pose,
    tf2_ros::Buffer& tf_buffer,
    const std::string& global_frame,
    const std::string& robot_frame,
    const double timeout,
    const rclcpp::Time stamp)
{
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    global_pose.header.frame_id = robot_frame;
    global_pose.header.stamp = stamp;

    // Transform Identity Pose from robot frame to global frame
    return transformPoseInTargetFrame(global_pose, global_pose,
        tf_buffer, global_frame, timeout);
}

bool transformPoseInTargetFrame(
    const geometry_msgs::msg::PoseStamped& pose_in,
    geometry_msgs::msg::PoseStamped& pose_out,
    tf2_ros::Buffer& tf_buffer,
    const std::string& target_frame,
    const double timeout)
{
    try {
        pose_out = tf_buffer.transform(pose_in,
            target_frame, tf2::durationFromSec(timeout));
        return true; // Return true if the transformation was successful
    } catch (tf2::LookupException & ex) {
        elog::error("No Transform available Error looking up target frame: {}", ex.what());
    } catch (tf2::ConnectivityException & ex) {
        elog::error("Connectivity Error looking up target frame: {}", ex.what());
    } catch (tf2::ExtrapolationException & ex) {
        elog::error("Extrapolation Error looking up target frame: {}", ex.what());
    } catch (tf2::TimeoutException & ex) {
        elog::error("Transform timeout with tolerance: {}", timeout);
    } catch (tf2::TransformException & ex) {
        elog::error("Failed to transform from {} to {}", pose_in.header.frame_id.c_str(), target_frame.c_str());
    }

    return false;
}

} // namespace rf_util