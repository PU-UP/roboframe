#include "integration/sensor_flow.hpp"
#include <map_updater/occupancy_map_component.hpp>
namespace roboframe {
SensorFlow::SensorFlow(MappingOptions options, Sink sink)
    : options_(options), sink_(std::move(sink)), poses_({512, options.max_pose_gap_us}),
      odometry_([this](const TimedPose2D& pose) {
          poses_.PushOdometry(pose);
          events_.odom = pose; events_.has_odom = true;
          if (phase_ == MappingPhase::mapping || phase_ == MappingPhase::ready) Drain(false);
      }), lidar_([this](const MapCorrection2D& correction) {
          correction_ = correction; has_correction_ = true;
          events_.correction = correction; events_.has_correction = true;
      }) {
    if (options.max_correction_age_us <= 0 || options.max_correction_age_us > 60000000)
        throw std::invalid_argument("invalid correction maximum age");
    odometry_.Activate(); lidar_.Activate();
}
void SensorFlow::RequireOpen() const { if (closed_) throw std::logic_error("sensor inputs closed"); }
void SensorFlow::PushOdometry(const TimedPose2D& pose) {
    std::lock_guard<std::mutex> lock(mutex_); RequireOpen(); odometry_.PushOdometry(pose);
}
void SensorFlow::PushReference(const MapPose2D& pose) {
    std::lock_guard<std::mutex> lock(mutex_); RequireOpen();
    if (options_.grid.frame != GridFrame::map || phase_ == MappingPhase::idle)
        throw std::logic_error("reference requires active map-frame mapping");
    if (phase_ != MappingPhase::finishing) lidar_.PushReferencePose(pose);
}
void SensorFlow::PushScan(Scan2DPtr scan) {
    if (!scan) throw std::invalid_argument("null scan");
    map_updater::ValidateScan(*scan);
    std::lock_guard<std::mutex> lock(mutex_); RequireOpen();
    if (scan->stamp_us <= last_scan_) throw std::invalid_argument("scan timestamps must strictly increase");
    last_scan_ = scan->stamp_us;
    if (phase_ == MappingPhase::idle || phase_ == MappingPhase::finishing) { ++counts_.ignored; return; }
    if (phase_ == MappingPhase::mapping) ++counts_.received;
    if (pending_.size() == 64) { pending_.pop_front(); Drop(false); }
    pending_.push_back(std::move(scan)); Drain(false);
}
void SensorFlow::Drop(bool sync) {
    if (phase_ == MappingPhase::mapping) {
        ++counts_.dropped;
        if (sync) ++counts_.drop_sync; else ++counts_.drop_queue;
    } else ++resources_.live_dropped;
}
void SensorFlow::Drain(bool finishing) {
    while (!pending_.empty()) {
        TimedPose2D pose;
        const auto match = poses_.At(pending_.front()->stamp_us, pose);
        if (match == PoseMatch::waiting && !finishing) break;
        if (match != PoseMatch::ready) { Drop(true); pending_.pop_front(); continue; }
        lidar_.PushScan(pending_.front(), pose);
        transforms_ = {};
        const bool map_frame = options_.grid.frame == GridFrame::map;
        if (map_frame && (!has_correction_ || correction_.stamp_us > pose.stamp_us ||
                         pose.stamp_us - correction_.stamp_us > options_.max_correction_age_us)) {
            ++resources_.tf_dropped; Drop(true);
        } else {
            transforms_ = {true, {pose, map_frame ? correction_ : MapCorrection2D{pose.stamp_us, {}}}};
            const auto accepted = sink_(generation_, pending_.front(), transforms_.transforms);
            if (accepted == Admission::accepted) ++submitted_;
            else { ++resources_.compute_rejected; Drop(false); }
        }
        events_.tf = transforms_; events_.has_tf = true;
        pending_.pop_front();
    }
}
void SensorFlow::Begin(std::uint64_t generation) {
    std::lock_guard<std::mutex> lock(mutex_); RequireOpen();
    generation_ = generation; submitted_ = 0; counts_ = {}; pending_.clear();
    phase_ = MappingPhase::mapping;
    lidar_.Reset(); has_correction_ = false; transforms_ = {};
    events_.has_correction = events_.has_tf = false;
}
SensorFlow::State SensorFlow::Finish() {
    std::lock_guard<std::mutex> lock(mutex_); RequireOpen();
    Drain(true); phase_ = MappingPhase::finishing; return StateLocked();
}
void SensorFlow::ResumeMapping() { std::lock_guard<std::mutex> lock(mutex_); phase_ = MappingPhase::mapping; }
void SensorFlow::Live() { std::lock_guard<std::mutex> lock(mutex_); phase_ = MappingPhase::ready; }
SensorFlow::State SensorFlow::Close() {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto previous = StateLocked();
    phase_ = MappingPhase::idle; pending_.clear();
    lidar_.Reset(); has_correction_ = false; transforms_ = {};
    events_.has_correction = events_.has_tf = false;
    return previous;
}
void SensorFlow::Shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true; pending_.clear(); poses_.Clear();
    transforms_ = {}; events_ = {}; odometry_.Deactivate(); lidar_.Deactivate();
}
SensorFlow::State SensorFlow::StateLocked() const {
    auto r = resources_;
    r.lidar_scans = lidar_.received(); r.correction_outputs = lidar_.published();
    r.missed_references = lidar_.missed_references();
    return {counts_, r, pending_.size(), submitted_, transforms_};
}
SensorFlow::State SensorFlow::status() const { std::lock_guard<std::mutex> lock(mutex_); return StateLocked(); }
SensorFlow::Events SensorFlow::TakeEvents() {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto result = events_; events_ = {}; return result;
}
} // namespace roboframe
