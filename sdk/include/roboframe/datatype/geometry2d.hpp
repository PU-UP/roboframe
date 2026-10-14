#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace roboframe {
// Metres/radians, right-handed x-forward/y-left. T_odom_base at acquisition time.
struct Pose2D { double x = 0, y = 0, yaw = 0; };
struct TimedPose2D { std::int64_t stamp_us = 0; Pose2D pose; };
inline double WrapAngle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }
inline void ValidatePose(const TimedPose2D& sample) {
    if (sample.stamp_us < 0 || !std::isfinite(sample.pose.x) || !std::isfinite(sample.pose.y) ||
        !std::isfinite(sample.pose.yaw) || std::abs(sample.pose.x) > 1e6 || std::abs(sample.pose.y) > 1e6)
        throw std::invalid_argument("invalid odometry timestamp or pose");
}
} // namespace roboframe
