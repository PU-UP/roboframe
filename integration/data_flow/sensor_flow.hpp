#pragma once
#include "integration/data_flow/pose_buffer.hpp"
#include <roboframe/grid.hpp>
#include <roboframe/data_flow.hpp>
#include <roboframe/transforms2d.hpp>
#include <roboframe/execution.hpp>
#include <odom/passthrough_odom_component.hpp>
#include <slam/recorded_slam_component.hpp>
#include <posegraph/pose_graph_component.hpp>
#include <functional>
#include <mutex>
namespace roboframe {
// This product's related sensor routes share TF/cache state. The short lock never covers
// heavy algorithms, external callbacks, file I/O or waiting for execution capacity.
class SensorFlow {
public:
    using Sink = std::function<Admission(std::uint64_t, Scan2DPtr, ScanTransforms2D)>;
    struct Options {
        GridFrame frame = GridFrame::odom;
        std::int64_t max_pose_gap_us = 250000, max_correction_age_us = 20000000;
    };
    // Lifetime input counters; only the command side attributes them to a business task.
    struct Counts {
        std::uint64_t received = 0, ignored = 0, dropped = 0, drop_sync = 0, drop_queue = 0;
    };
    struct State {
        Counts counts;
        DataFlowStatus resources;
        std::size_t pending = 0;
        std::uint64_t submitted = 0;
        TransformStatus2D transforms;
    };
    struct Events {
        bool has_odom = false, has_correction = false, has_tf = false;
        TimedPose2D odom;
        MapCorrection2D correction;
        TransformStatus2D tf;
    };
    SensorFlow(Options options, Sink sink);
    void PushOdometry(const TimedPose2D& pose);
    void PushReference(const MapPose2D& pose);
    void PushScan(Scan2DPtr scan);
    State Open(std::uint64_t generation); // New data context, reset TF and pending inputs.
    State Pause(); // Stop new scan/reference admission; resolve remaining timestamp matches.
    void Resume(); // Reopen the same context without resetting data or counters.
    State Close(); // Discard pending inputs and invalidate TF; return pre-close accounting.
    void Shutdown();
    State status() const;
    PoseNode2D Node(std::uint64_t generation, std::uint64_t id) const;
    Events TakeEvents(); // Product publication only; downstream routing already happened at ingress.
private:
    void Drain(bool flush);
    void Drop(bool sync);
    State StateLocked() const;
    void RequireOpen() const;
    mutable std::mutex mutex_;
    const Options options_;
    const Sink sink_;
    PoseBuffer poses_;
    odom::PassthroughOdomComponent odometry_;
    slam::RecordedSlamComponent slam_;
    posegraph::PoseGraphComponent graph_;
    TimedPose2D matched_odom_;
    std::deque<Scan2DPtr> pending_;
    enum class AdmissionState { closed, accepting, paused };
    AdmissionState admission_ = AdmissionState::closed;
    std::uint64_t generation_ = 0, submitted_ = 0;
    Counts counts_;
    DataFlowStatus resources_;
    TransformStatus2D transforms_;
    bool closed_ = false;
    std::int64_t last_scan_ = -1;
    Events events_;
};
} // namespace roboframe
