#pragma once
#include "integration/pose_buffer.hpp"
#include <roboframe/mapping.hpp>
#include <roboframe/data_flow.hpp>
#include <roboframe/transforms2d.hpp>
#include <roboframe/execution.hpp>
#include <odom/passthrough_odom_component.hpp>
#include <lidar_map/recorded_lidar_map_component.hpp>
#include <functional>
#include <mutex>
namespace roboframe {
// This product's related sensor routes share TF/cache state. The short lock never covers
// heavy algorithms, external callbacks, file I/O or waiting for execution capacity.
class SensorFlow {
public:
    using Sink = std::function<Admission(std::uint64_t, Scan2DPtr, ScanTransforms2D)>;
    struct State {
        MappingStatus counts;
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
    SensorFlow(MappingOptions options, Sink sink);
    void PushOdometry(const TimedPose2D& pose);
    void PushReference(const MapPose2D& pose);
    void PushScan(Scan2DPtr scan);
    void Begin(std::uint64_t generation);
    State Finish(); // Close scan admission and resolve remaining timestamp matches.
    void ResumeMapping(); // Empty FINISH rejected without ending the session.
    void Live();
    State Close();
    void Shutdown();
    State status() const;
    Events TakeEvents(); // Product publication only; downstream routing already happened at ingress.
private:
    void Drain(bool finishing);
    void Drop(bool sync);
    State StateLocked() const;
    void RequireOpen() const;
    mutable std::mutex mutex_;
    const MappingOptions options_;
    const Sink sink_;
    PoseBuffer poses_;
    odom::PassthroughOdomComponent odometry_;
    lidar_map::RecordedLidarMapComponent lidar_;
    std::deque<Scan2DPtr> pending_;
    MappingPhase phase_ = MappingPhase::idle;
    std::uint64_t generation_ = 0, submitted_ = 0;
    MappingStatus counts_;
    DataFlowStatus resources_;
    TransformStatus2D transforms_;
    MapCorrection2D correction_;
    bool has_correction_ = false, closed_ = false;
    std::int64_t last_scan_ = -1;
    Events events_;
};
} // namespace roboframe
