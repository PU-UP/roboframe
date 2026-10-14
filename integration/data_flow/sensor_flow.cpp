#include "integration/data_flow/sensor_flow.hpp"

namespace roboframe {
SensorFlow::SensorFlow(Options options, Sink sink)
    : options_(options), sink_(std::move(sink)), poses_({512, options.max_pose_gap_us}),
      odometry_([this](const TimedPose2D& pose) {
          poses_.PushOdometry(pose); graph_.PushOdometry(pose);
          const auto tf = graph_.current();
          if (admission_ != AdmissionState::closed) { events_.tf = tf; events_.has_tf = true; }
          events_.odom = pose; events_.has_odom = true;
          if (admission_ == AdmissionState::accepting) Drain(false);
      }), slam_([this](const MapPose2D& pose) {
          graph_.PushLocalization(pose, matched_odom_);
          events_.correction = graph_.correction(); events_.has_correction = true;
      }), graph_(options.max_correction_age_us, options.frame == GridFrame::map) {
    if (options.max_correction_age_us <= 0 || options.max_correction_age_us > 60000000)
        throw std::invalid_argument("invalid correction maximum age");
    odometry_.Activate(); slam_.Activate(); graph_.Activate();
}
void SensorFlow::RequireOpen() const { if (closed_) throw std::logic_error("sensor inputs closed"); }
void SensorFlow::PushOdometry(const TimedPose2D& pose) {
    std::lock_guard<std::mutex> lock(mutex_); RequireOpen(); odometry_.PushOdometry(pose);
}
void SensorFlow::PushReference(const MapPose2D& pose) {
    std::lock_guard<std::mutex> lock(mutex_); RequireOpen();
    if (options_.frame != GridFrame::map || admission_ == AdmissionState::closed)
        throw std::logic_error("reference requires an open map-frame data context");
    if (admission_ == AdmissionState::accepting) slam_.PushReferencePose(pose);
}
void SensorFlow::PushScan(Scan2DPtr scan) {
    if (!scan) throw std::invalid_argument("null scan");
    ValidateScan2D(*scan);
    if (scan->hits.size() > 2048) throw std::invalid_argument("scan exceeds product capacity");
    for (const auto& hit : scan->hits)
        if (std::hypot(hit.x, hit.y) > 20.001 || std::hypot(hit.x, hit.y) < 0.01)
            throw std::invalid_argument("invalid scan endpoint");
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
        matched_odom_ = pose;
        slam_.PushScan(pending_.front(), pose);
        transforms_ = graph_.Resolve(pose);
        if (!transforms_.valid) {
            ++resources_.tf_dropped; Drop(true);
        } else {
            if (graph_.node_count() == posegraph::PoseGraphComponent::max_nodes) {
                ++resources_.history_rejected; Drop(false); pending_.pop_front(); continue;
            }
            const auto accepted = sink_(generation_, pending_.front(), transforms_.transforms);
            if (accepted == Admission::accepted) {
                graph_.RegisterNode(pose); // Lookup on the compute thread waits for this short lock.
                ++submitted_;
            }
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
    slam_.Reset(); graph_.Reset(); transforms_ = {};
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
    slam_.Reset(); graph_.Reset(); transforms_ = {};
    events_.has_correction = events_.has_tf = false;
    return previous;
}
void SensorFlow::Shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true; pending_.clear(); poses_.Clear();
    transforms_ = {}; events_ = {}; odometry_.Deactivate(); slam_.Deactivate(); graph_.Deactivate();
}
SensorFlow::State SensorFlow::StateLocked() const {
    auto r = resources_;
    r.slam_scans = slam_.received(); r.correction_outputs = slam_.published();
    r.pose_nodes = graph_.node_count();
    r.missed_references = slam_.missed_references();
    return {counts_, r, pending_.size(), submitted_, transforms_};
}
SensorFlow::State SensorFlow::status() const { std::lock_guard<std::mutex> lock(mutex_); return StateLocked(); }
PoseNode2D SensorFlow::Node(std::uint64_t generation, std::uint64_t id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation != generation_ || admission_ == AdmissionState::closed)
        throw std::logic_error("stale node lookup");
    return graph_.Node(id);
}
SensorFlow::Events SensorFlow::TakeEvents() {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto result = events_; events_ = {}; return result;
}
} // namespace roboframe
