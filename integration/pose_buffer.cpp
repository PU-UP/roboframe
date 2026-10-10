#include "integration/pose_buffer.hpp"
#include <algorithm>
#include <iterator>
#include <stdexcept>

namespace roboframe {
PoseBuffer::PoseBuffer(PoseBufferConfig config) : config_(config) {
    if (config.capacity < 2 || config.capacity > 100000 || config.max_gap_us <= 0 ||
        config.max_gap_us > 10000000) throw std::invalid_argument("invalid odometry buffer configuration");
}
void PoseBuffer::PushOdometry(const roboframe::TimedPose2D& sample) {
    roboframe::ValidatePose(sample);
    if (!samples_.empty() && sample.stamp_us <= samples_.back().stamp_us)
        throw std::invalid_argument("odometry timestamps must strictly increase");
    samples_.push_back(sample);
    if (samples_.size() > config_.capacity) samples_.pop_front();
}
PoseMatch PoseBuffer::At(std::int64_t stamp, roboframe::TimedPose2D& pose) const {
    if (stamp < 0) throw std::invalid_argument("negative scan time");
    if (samples_.empty() || stamp > samples_.back().stamp_us) return PoseMatch::waiting;
    if (stamp < samples_.front().stamp_us) return PoseMatch::unavailable;
    const auto right = std::lower_bound(samples_.begin(), samples_.end(), stamp,
        [](const roboframe::TimedPose2D& p, std::int64_t t) { return p.stamp_us < t; });
    if (right->stamp_us == stamp) { pose = *right; return PoseMatch::ready; }
    const auto& left = *std::prev(right);
    const auto gap = right->stamp_us - left.stamp_us;
    if (gap > config_.max_gap_us) return PoseMatch::unavailable;
    const double fraction = double(stamp - left.stamp_us) / double(gap);
    pose = {stamp, {left.pose.x + fraction * (right->pose.x - left.pose.x),
                   left.pose.y + fraction * (right->pose.y - left.pose.y),
                   roboframe::WrapAngle(left.pose.yaw + fraction *
                       roboframe::WrapAngle(right->pose.yaw - left.pose.yaw))}};
    return PoseMatch::ready;
}
} // namespace roboframe
