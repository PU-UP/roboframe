#include <localmapbuilder/local_map_component.hpp>
namespace localmapbuilder {
using namespace roboframe;
ComponentInfo GetComponentInfo() { return {component_name, source_id, sdk_version}; }
LocalMapComponent::LocalMapComponent(std::size_t limit) : limit_(limit) {
    if (!limit || limit > 64) throw std::invalid_argument("submap window must be 1..64 scans");
}
void LocalMapComponent::Reset() { current_.reset(); next_id_ = 0; last_stamp_ = -1; }
Submap2DPtr LocalMapComponent::PushScan(Scan2DPtr scan, const TimedPose2D& odom, std::uint64_t node_id, bool new_anchor) {
    if (lifecycle_ != Lifecycle::active) throw std::logic_error("localmapbuilder inactive");
    if (!scan || !node_id) throw std::invalid_argument("scan and node identity required");
    if (scan->hits.size() > 2048) throw std::length_error("local scan exceeds point budget");
    ValidateScan2D(*scan); ValidatePose(odom);
    if (scan->stamp_us != odom.stamp_us || scan->stamp_us <= last_stamp_)
        throw std::invalid_argument("scan/odometry time mismatch or unordered scan");
    auto next = std::make_shared<Submap2D>();
    Pose2D anchor = odom_from_local_;
    if (new_anchor || !current_ || current_->scans.size() == limit_) {
        next->id = next_id_ + 1; next->anchor_id = node_id; anchor = odom.pose;
    } else *next = *current_;
    next->scans.push_back({std::move(scan), Compose(Inverse(anchor), odom.pose)});
    ++next->revision;
    last_stamp_ = odom.stamp_us; next_id_ = next->id; odom_from_local_ = anchor;
    current_ = next; return next;
}
} // namespace localmapbuilder
