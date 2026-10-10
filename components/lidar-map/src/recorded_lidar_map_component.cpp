#include <lidar_map/recorded_lidar_map_component.hpp>
#include <utility>
namespace lidar_map {
roboframe::ComponentInfo GetComponentInfo() {
    return {component_name, source_id, roboframe::sdk_version};
}
RecordedLidarMapComponent::RecordedLidarMapComponent(Output output) : output_(std::move(output)) {
    if (!output_) throw std::invalid_argument("map correction output required");
}
void RecordedLidarMapComponent::Reset() {
    references_.clear(); last_scan_us_ = last_reference_us_ = -1;
    received_ = published_ = missed_ = 0;
}
void RecordedLidarMapComponent::PushReferencePose(const roboframe::MapPose2D& pose) {
    if (lifecycle_ != roboframe::Lifecycle::active) throw std::logic_error("lidar-map inactive");
    roboframe::ValidatePose({pose.stamp_us, pose.pose});
    if (pose.stamp_us <= last_reference_us_ || pose.stamp_us <= last_scan_us_)
        throw std::invalid_argument("reference must precede its scan, in timestamp order");
    if (references_.size() == 64) throw std::length_error("reference queue full");
    references_.push_back(pose); last_reference_us_ = pose.stamp_us;
}
void RecordedLidarMapComponent::PushScan(const roboframe::Scan2DPtr& scan,
                                         const roboframe::TimedPose2D& odometry) {
    if (lifecycle_ != roboframe::Lifecycle::active) throw std::logic_error("lidar-map inactive");
    if (!scan) throw std::invalid_argument("null scan");
    roboframe::ValidateScan2D(*scan); roboframe::ValidatePose(odometry);
    if (scan->stamp_us != odometry.stamp_us || scan->stamp_us <= last_scan_us_)
        throw std::invalid_argument("scan/odometry time mismatch or unordered scan");
    last_scan_us_ = scan->stamp_us; ++received_;
    while (!references_.empty() && references_.front().stamp_us < scan->stamp_us) {
        references_.pop_front(); ++missed_;
    }
    if (references_.empty() || references_.front().stamp_us != scan->stamp_us) return;
    const roboframe::MapCorrection2D correction{scan->stamp_us,
        roboframe::Compose(references_.front().pose, roboframe::Inverse(odometry.pose))};
    references_.pop_front(); ++published_;
    output_(correction);
}
} // namespace lidar_map
