#include <odom_buffer/component.hpp>
#include <algorithm>
#include <iterator>
#include <stdexcept>

namespace odom_buffer {
static_assert(roboframe::Compatible(roboframe::sdk_version, {1, 1, 0}), "OdomBuffer requires SDK >=1.1, same major");
roboframe::ComponentInfo GetComponentInfo() {
    return {"odom-buffer", ROBOFRAME_COMPONENT_VERSION, roboframe::sdk_version, contract_version};
}
Component::Component(Config config) : config_(config) {
    if (config.capacity < 2 || config.capacity > 100000 || config.max_gap_us <= 0 ||
        config.max_gap_us > 10000000) throw std::invalid_argument("invalid odometry buffer configuration");
}
void Component::PushOdometry(const roboframe::TimedPose2D& sample) {
    if (lifecycle_ != roboframe::Lifecycle::active) throw std::logic_error("odometry inactive");
    roboframe::ValidatePose(sample);
    if (!samples_.empty() && sample.stamp_us <= samples_.back().stamp_us)
        throw std::invalid_argument("odometry timestamps must strictly increase");
    samples_.push_back(sample);
    if (samples_.size() > config_.capacity) samples_.pop_front();
}
Match Component::At(std::int64_t stamp, roboframe::TimedPose2D& pose) const {
    if (lifecycle_ != roboframe::Lifecycle::active) throw std::logic_error("odometry inactive");
    if (stamp < 0) throw std::invalid_argument("negative scan time");
    if (samples_.empty() || stamp > samples_.back().stamp_us) return Match::waiting;
    if (stamp < samples_.front().stamp_us) return Match::unavailable;
    const auto right = std::lower_bound(samples_.begin(), samples_.end(), stamp,
        [](const roboframe::TimedPose2D& p, std::int64_t t) { return p.stamp_us < t; });
    if (right->stamp_us == stamp) { pose = *right; return Match::ready; }
    const auto& left = *std::prev(right);
    const auto gap = right->stamp_us - left.stamp_us;
    if (gap > config_.max_gap_us) return Match::unavailable;
    const double fraction = double(stamp - left.stamp_us) / double(gap);
    pose = {stamp, {left.pose.x + fraction * (right->pose.x - left.pose.x),
                   left.pose.y + fraction * (right->pose.y - left.pose.y),
                   roboframe::WrapAngle(left.pose.yaw + fraction *
                       roboframe::WrapAngle(right->pose.yaw - left.pose.yaw))}};
    return Match::ready;
}
} // namespace odom_buffer
