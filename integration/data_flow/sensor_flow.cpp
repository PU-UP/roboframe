#include "integration/data_flow/sensor_flow.hpp"
#include <map_updater/occupancy_map_component.hpp>
namespace roboframe {
SensorFlow::SensorFlow(Options options, Sink sink)
    : options_(options), sink_(std::move(sink)), poses_({512, options.max_pose_gap_us}),
      odometry_([this](const TimedPose2D& pose) {
          poses_.PushOdometry(pose);
          events_.odom = pose; events_.has_odom = true;
          if (admission_ == AdmissionState::accepting) Drain(false);
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
    if (options_.frame != GridFrame::map || admission_ == AdmissionState::closed)
        throw std::logic_error("reference requires an open map-frame data context");
    if (admission_ == AdmissionState::accepting) lidar_.PushReferencePose(pose);
}
void SensorFlow::PushScan(Scan2DPtr scan) {
    if (!scan) throw std::invalid_argument("null scan");
    map_updater::ValidateScan(*scan);
    std::lock_guard<std::mutex> lock(mutex_); RequireOpen();
    if (scan->stamp_us <= last_scan_) throw std::invalid_argument("scan timestamps must strictly increase");
    last_scan_ = scan->stamp_us;
    if (admission_ != AdmissionState::accepting) { ++counts_.ignored; return; }
    ++counts_.received;
    if (pending_.size() == 64) { pending_.pop_front(); Drop(false); }
    pending_.push_back(std::move(scan)); Drain(false);
}
void SensorFlow::Drop(bool sync) {
    ++counts_.dropped;
    if (sync) ++counts_.drop_sync; else ++counts_.drop_queue;
}
void SensorFlow::Drain(bool flush) {
    while (!pending_.empty()) {
        TimedPose2D pose;
        const auto match = poses_.At(pending_.front()->stamp_us, pose);
        if (match == PoseMatch::waiting && !flush) break;
        if (match != PoseMatch::ready) { Drop(true); pending_.pop_front(); continue; }
        lidar_.PushScan(pending_.front(), pose);
        transforms_ = {};
        const bool map_frame = options_.frame == GridFrame::map;
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
SensorFlow::State SensorFlow::Open(std::uint64_t generation) {
    std::lock_guard<std::mutex> lock(mutex_); RequireOpen();
    if (!generation) throw std::invalid_argument("data generation must be nonzero");
    generation_ = generation; submitted_ = 0; pending_.clear();
    admission_ = AdmissionState::accepting;
    lidar_.Reset(); has_correction_ = false; transforms_ = {};
    events_.has_correction = events_.has_tf = false;
    return StateLocked();
}
SensorFlow::State SensorFlow::Pause() {
    std::lock_guard<std::mutex> lock(mutex_); RequireOpen();
    if (admission_ != AdmissionState::accepting) throw std::logic_error("data context is not accepting");
    Drain(true); admission_ = AdmissionState::paused; return StateLocked();
}
void SensorFlow::Resume() {
    std::lock_guard<std::mutex> lock(mutex_); RequireOpen();
    if (admission_ != AdmissionState::paused) throw std::logic_error("data context is not paused");
    admission_ = AdmissionState::accepting;
}
SensorFlow::State SensorFlow::Close() {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto previous = StateLocked();
    admission_ = AdmissionState::closed; pending_.clear();
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
