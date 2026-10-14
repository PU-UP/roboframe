#pragma once
#include <roboframe/datatype/geometry2d.hpp>
namespace roboframe {
// T_parent_child converts child-frame coordinates into parent-frame coordinates.
inline Pose2D Compose(const Pose2D& parent_from_middle, const Pose2D& middle_from_child) {
    const double c = std::cos(parent_from_middle.yaw), s = std::sin(parent_from_middle.yaw);
    return {parent_from_middle.x + c*middle_from_child.x - s*middle_from_child.y,
            parent_from_middle.y + s*middle_from_child.x + c*middle_from_child.y,
            WrapAngle(parent_from_middle.yaw + middle_from_child.yaw)};
}
inline Pose2D Inverse(const Pose2D& pose) {
    const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
    return {-c*pose.x - s*pose.y, s*pose.x - c*pose.y, WrapAngle(-pose.yaw)};
}
// Distinct from TimedPose2D (T_odom_base); time is the sensor acquisition clock.
struct MapPose2D { std::int64_t stamp_us = 0; Pose2D pose; };
struct MapCorrection2D { std::int64_t stamp_us = 0; Pose2D pose; }; // T_map_odom
struct ScanTransforms2D {
    TimedPose2D odom_from_base;
    MapCorrection2D map_from_odom; // Latest causal correction; PoseGraph enforces maximum age.
};
inline void ValidateTransforms(const ScanTransforms2D& tf) {
    ValidatePose(tf.odom_from_base);
    const auto& correction = tf.map_from_odom;
    if (correction.stamp_us < 0 || !std::isfinite(correction.pose.x) ||
        !std::isfinite(correction.pose.y) || !std::isfinite(correction.pose.yaw))
        throw std::invalid_argument("invalid map correction");
    if (tf.map_from_odom.stamp_us > tf.odom_from_base.stamp_us)
        throw std::invalid_argument("future map correction");
}
inline Pose2D MapFromBase(const ScanTransforms2D& tf) {
    ValidateTransforms(tf);
    return Compose(tf.map_from_odom.pose, tf.odom_from_base.pose);
}
struct TransformStatus2D { bool valid = false; ScanTransforms2D transforms; };
} // namespace roboframe
